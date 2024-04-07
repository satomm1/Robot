/****************************************************************************
 Module
   IMU_SM.c

 Description
   This is a file for implementing reading from the ICM-42688-P 6-axis IMU

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "IMU_SM.h"
#include <sys/attribs.h>
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define READ  0b10000000
#define WRITE 0b00000000
#define FIFO_PACKET_SIZE 16
#define ACCEL_SENSITIVITY 4096 // LSB/g
#define GYRO_SENSITIVITY 131 // LSB/(deg/s)
#define ACCEL_MAX 8  // 8g
#define GYRO_MAX 250 // deg/sec

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitIMU(void);
void ResetIMU(void);
void WriteIMU(uint8_t Address, uint8_t LowerByte, uint8_t UpperByte, uint8_t NumBytes);
void WriteIMU2(uint8_t Address, AccelGyroData_t data);
void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2);
void ReadFIFO(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static ImuState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static uint8_t FIFO_index = 0;
static uint8_t Active_FIFO = 1;
static uint8_t Readable_FIFO;
static int8_t IMU_FIFO[2][16];

static AccelGyroData_t Accel[3]; // Holds the current acceleration data from IMU
static float Accel_g[3]; // Holds acceleration in g's
static AccelGyroData_t Gyro[3]; // Holds the current gyroscope readings from IMU
static float Gyro_deg_s[3]; // Holds the angular velocity in deg/s

static bool IntStatusReading = false;
static bool FifoReading = false;
static uint8_t rx_data[16];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitImuSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitImuSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set interrupt pins to inputs
  TRISDSET = _TRISD_TRISD12_MASK | _TRISD_TRISD13_MASK;
  
  INT2R = 0b1010; // Map RD12 -> External interrupt 2
  
  // Set SPI1 Pins to correct input or output setting
  TRISDCLR = _TRISD_TRISD1_MASK | _TRISD_TRISD3_MASK | _TRISD_TRISD4_MASK; // Set SCK1, SS1, SDO1 to output
  TRISDSET = _TRISD_TRISD2_MASK; // Set SDI1 to Input
  
  
  TRISHCLR = _TRISH_TRISH5_MASK | _TRISH_TRISH4_MASK | _TRISH_TRISH6_MASK |_TRISH_TRISH7_MASK;
  ANSELHCLR = _ANSELH_ANSH5_MASK |_ANSELH_ANSH4_MASK|_ANSELH_ANSH6_MASK;
  LATHbits.LATH5 = 0;
  
  // Map SPI1 Pins to correct function
  // RD1 is mapped to CLK1 by default
//  RPD4R = 0b0101; // Map RD4 -> SS1
  RPD3R = 0b0101; // Map RD3 -> SDO1
  SDI1R = 0b0000; // Map SDI1 -> RD2
  
  LATDbits.LATD4 = 1; // Set CS line high
  
  // Initialize SPI1
  SPI1CON = 0; // Reset SPI1CON settings
  SPI1CONbits.FRMEN = 0; // Disable framed SPI support
  SPI1CONbits.FRMPOL = 0; // SS1 is active low
  SPI1CONbits.MSSEN = 0; // SS is automatically driven
  SPI1CONbits.MCLKSEL = 0; // Use PBCLK2 for the Baud Rate Generator (50 MHz)
  SPI1CONbits.ENHBUF = 1; // Enhance buffer enabled (use FIFOs)
  SPI1CONbits.DISSDO = 0; // SDO1 is used by the module
  SPI1CONbits.MODE32 = 0; // 8 bit mode
  SPI1CONbits.MODE16 = 0; // 8 bit mode
  SPI1CONbits.SMP = 1; // Data sampled at middle of data output time
  SPI1CONbits.CKE = 1; // Serial output data changes on transition from active clock state to idle clock state
  SPI1CONbits.CKP = 1; // Idle state for the clock is high level
  SPI1CONbits.MSTEN = 1; // Host mode
  SPI1CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI1CONbits.STXISEL = 0b00; // Interrupt generated when last transfer shifted out of SPISR and transmit operations are complete
  SPI1CONbits.SRXISEL = 0b01; // Interrupt is generated when the buffer is not empty

  SPI1CON2 = 0; // Reset SPI1CON2 register settings
  SPI1CON2bits.AUDEN = 0; // Audio protocol is disabled
  
  while (!SPI1STATbits.SPIRBE){
      uint8_t ClearData = SPI1BUF;
  }
  SPI1STATbits.SPIROV = 0; // Clear the Receive overflow bit
  
  SPI1BRG = 15; //15; // 1.56 MHz clock frequency, IMU Has max frequency of 10 MHz
    
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC27bits.SPI1TXIP = 7; // SPI2TX
  IPC27bits.SPI1RXIP = 7; // SPI2RX
  IPC3bits.INT2IP = 7; // External Interrupt 2
  IPC7bits.T6IP = 7; // T6
  
  // Clear interrupt flags
  IFS3CLR = _IFS3_SPI1RXIF_MASK | _IFS3_SPI1TXIF_MASK;
  IFS0CLR = _IFS0_INT2IF_MASK;
  
  // Disable the RX interrupt
  IEC3CLR = _IEC3_SPI1RXIE_MASK;
  
  // Disable TX interrupt
  IEC3CLR = _IEC3_SPI1TXIE_MASK;
  
  // Disable Int 2 Interrupt
  INTCONbits.INT2EP = 0; // Interrupt on falling edge
  IEC0CLR = _IEC0_INT2IE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
  SPI1CONbits.ON = 1; // Finally turn SPI1 on
    
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_IMU;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostImuSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostImuSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunImuSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
****************************************************************************/
ES_Event_t RunImuSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_IMU:       
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        ResetIMU(); // Perform a soft reset of the IMU
        
        // now put the machine into the actual initial state
        CurrentState = IMUReset;
        ES_Timer_InitTimer(IMU_TIMER, 500);
      }
    }
    break;

    case IMUReset:      
    {
      switch (ThisEvent.EventType)
      {
               
        case ES_TIMEOUT:
        {
            InitIMU(); // After giving some time from resetting, apply custom settings
            CurrentState = IMUWait;
            ES_Timer_InitTimer(IMU_TIMER, 500);
        }
        break;
        
        default:
          ;
      }
      
    }
    break;
    
    case IMUWait:      
    {
      switch (ThisEvent.EventType)
      {        
          case ES_TIMEOUT:
          {             
            // After giving time for setup, enable accelerometer and gyroscope
              
            // Setup the accelerometer/gyro settings for the IMU, do a burst write since addresses are consecutive
            AccelGyroData_t data2send;
            AccelGyroData_t data2send2;
            
            // Accel
            data2send.DataStruct.LowerByte = 0b00010110; // cutoff = acc_odr/2, acc_range = +/- 4gh, 8.19 LSB/mg, Sample Rate = 25 Hz
            data2send.DataStruct.UpperByte = 0b01000000; // Normal mode, no averaging

            // Gyro
            data2send2.DataStruct.LowerByte = 0b00010110; // cutoff = gyr_odr/2, gyr_range = +/- 250 deg/s, 131.2 LSB/deg/s, Sample Rate = 25 Hz
            data2send2.DataStruct.UpperByte = 0b01000000; // Normal mode, no averaging
            WriteIMU2Transfer(0x20, data2send, data2send2);
              
            CurrentState = IMURun;
            ES_Timer_InitTimer(IMU_TIMER, 1000);
          }
          break;
        
        default:
          ;
      } 
    }
    break;
    
    case IMURun:      
    {
      switch (ThisEvent.EventType)
      {
               
        case ES_TIMEOUT:
        {
            // Periodically print out gyro values.
            DB_printf("Gyro Z: %d\r\n", Gyro[2].FullData);
            ES_Timer_InitTimer(IMU_TIMER, 1000);
        }
        break;

        default:
          ;
      }
      
    }
    break;
    
    default:
      ;
  }                                  
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryImuSM

 Parameters
     None

 Returns
     ImuState_t The current state of the Imu state machine

 Description
     returns the current state of the Imu state machine
****************************************************************************/
ImuState_t QueryImuSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     WriteImuToSPI

 Parameters
     uint32_t Buffer: the SPI buffer address to write the IMU data to

 Returns
     None

 Description
     Writes the current IMU data to the specified SPI buffer in a single 16-byte
     message
****************************************************************************/
void WriteImuToSPI(uint32_t Buffer)
{
    Buffer = 9; // Header to indicate this is IMU data
    
    // Send Accel sensitivity (byte 2/3)
    uint16_t accel_sensitivity = ACCEL_SENSITIVITY;
    Buffer = (uint8_t)(accel_sensitivity >> 8); // Upper 8 bits of sensitivity 
    Buffer = (uint8_t)(accel_sensitivity & 0xFF); // Lower 8 bits of sensitivity
    
    // Send the accel data
    Buffer =  Accel[0].DataStruct.UpperByte; // byte 4 
    Buffer =  Accel[0].DataStruct.LowerByte;
    Buffer =  Accel[1].DataStruct.UpperByte;
    Buffer =  Accel[1].DataStruct.LowerByte;
    Buffer =  Accel[2].DataStruct.UpperByte;
    Buffer =  Accel[2].DataStruct.LowerByte; // byte 9
    
    // Send Gyro Sensitivity (byte 10)
    Buffer = (uint8_t)GYRO_SENSITIVITY;
    
    // Send the gyro data
    Buffer = Gyro[0].DataStruct.UpperByte; // byte 11
    Buffer = Gyro[0].DataStruct.LowerByte;
    Buffer = Gyro[1].DataStruct.UpperByte;
    Buffer = Gyro[1].DataStruct.LowerByte;
    Buffer = Gyro[2].DataStruct.UpperByte;
    Buffer = Gyro[2].DataStruct.LowerByte; // byte 16
}

/****************************************************************************
 Function
     WriteAccelToSPI

 Parameters
     uint32_t Buffer: the SPI buffer address to write the accel data to

 Returns
     None

 Description
     Writes the current accel data to the specified SPI buffer
****************************************************************************/
void WriteAccelToSPI(uint32_t Buffer)
{
    Buffer = 10; // Header to indicate this is Accel data (byte 1)
    
    // Send the acceleration. Acceleration are floats which can be sent as 
    // 4 packets of 8 bits. (bytes 2-13)
    for (uint8_t i=0; i<3; i++) { // iterate through x,y,z
        uint32_t accel_as_int = *((uint32_t*)&Accel_g[i]);
        for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
            Buffer = (accel_as_int >> (24-8*j)) & 0xFF;
        }
    }
}

/****************************************************************************
 Function
     WriteGyroToSPI

 Parameters
     uint32_t Buffer: the SPI buffer address to write the gyro data to

 Returns
     None

 Description
     Writes the current gyro data to the specified SPI buffer
****************************************************************************/
void WriteGyroToSPI(uint32_t Buffer)
{
    Buffer = 11; // Header to indicate this is gyro data (byte 1)
    
    // Send the gyro. gyro data are floats which can be sent as 
    // 4 packets of 8 bits. (bytes 2-13)
    for (uint8_t i=0; i<3; i++) { // iterate through x,y,z
        uint32_t gyro_as_int = *((uint32_t*)&Gyro_deg_s[i]);
        for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
            Buffer = (gyro_as_int >> (24-8*j)) & 0xFF;
        }
    }
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     InitImu

 Parameters
     None

 Returns
     None

 Description
     Sends the correct sequence of writes to the IMU via SPI to correctly set 
     up the IMU and the IMU FIFO
****************************************************************************/
void InitIMU(void)
{
    AccelGyroData_t data2send;
    uint16_t data = ReadIMU16(0x00); // Dummy call to set up SPI
    
//    // Reset IMU
//    data2send.FullData = 0xDEAF;
//    WriteIMU2(0x7E, data2send);
//    
//    data = ReadIMU16(0x00); // Dummy call to set up SPI
    
    
    data = ReadIMU8(0x00); // Get chip ID
    while (data != 0b01000011) {
        DB_printf("Incorrect Chip ID: %d\r\n", data);
        data = ReadIMU16(0x00); // Get chip ID
    }
    DB_printf("Chip ID: %d\r\n", data);

    // Get the status of the chip
    data = ReadIMU16(0x02);
//    DB_printf("Status: %d\r\n", data);

    //////////////////// Set up the FIFO /////////////////////////////////
    data2send.FullData = 6; // Number of words in FIFO to trigger interrupt
    WriteIMU2(0x35, data2send);
    data = ReadIMU16(0x35);
    DB_printf("FIFO Watermark Level: %d\r\n", data);

    data2send.DataStruct.LowerByte = 0b00000000; // Overwrite when full
    data2send.DataStruct.UpperByte = 0b00000110; // Write accel and gyro data
    WriteIMU2(0x36, data2send);
    data = ReadIMU16(0x36);
//    DB_printf("FIFO CONFIG: %d\r\n", data);

    ////////////////////// Set up interrupts /////////////////////////////
    data2send.DataStruct.LowerByte = 0b00000000;
    data2send.DataStruct.UpperByte = 0b00010000; // Map watermark interrupt to INT1
    WriteIMU2(0x3B, data2send);
    data = ReadIMU16(0x3B);
//    DB_printf("Int Map 2: %d\r\n", data);
              
    // Enable interrupt 1 
    data2send.DataStruct.LowerByte = 0b00000100; // Enable int1, active low, push-pull
    data2send.DataStruct.UpperByte = 0b00000000; // disable int2, active low, push-pull
    WriteIMU2(0x38, data2send);
    data = ReadIMU16(0x38);
//    DB_printf("Int ctrl: %d\r\n", data);
    
    uint16_t fill_level = ReadIMU16(0x15);
//    DB_printf("Fill Level: %d\r\n", fill_level);

    // Enable PIC External Interrupt 2
    IEC0SET = _IEC0_INT2IE_MASK;

    
//    AccelGyroData_t data2send2;
    // Setup the accelerometer/gyro settings for the IMU, do a burst write since addresses are consecutive
    
    // Accel
//    data2send.DataStruct.LowerByte = 0b00010110; // cutoff = acc_odr/2, acc_range = +/- 4gh, 8.19 LSB/mg, Sample Rate = 25 Hz
//    data2send.DataStruct.UpperByte = 0b01000000; // Normal mode, no averaging
//    
//    // Gyro
//    data2send2.DataStruct.LowerByte = 0b00010110; // cutoff = gyr_odr/2, gyr_range = +/- 250 deg/s, 131.2 LSB/deg/s, Sample Rate = 25 Hz
//    data2send2.DataStruct.UpperByte = 0b01000000; // Normal mode, no averaging
//    WriteIMU2Transfer(0x20, data2send, data2send2);
//    data = ReadIMU16(0x21);
//    DB_printf("Gyro ctrl: %d\r\n", data);
    return;
}

void ResetIMU(void) {
    AccelGyroData_t data2send;
    uint16_t data = ReadIMU16(0x00); // Dummy call to set up SPI
    
    // Reset IMU
    data2send.FullData = 0xDEAF;
    WriteIMU2(0x7E, data2send);
    
    return;
}

/****************************************************************************
 Function
     WriteIMU

 Parameters
     uint8_t Address: The address of the register to write to
     uint8_t Data: The data to write to the register

 Returns
     None

 Description
     Sends the correct sequence of bytes to the IMU via SPI to write to the 
     desired register. 
 
    ***Assumes the Bank is already correctly selected***
****************************************************************************/
void WriteIMU(uint8_t Address, uint8_t LowerByte, uint8_t UpperByte, uint8_t NumBytes)
{
    __builtin_disable_interrupts();
    LATDbits.LATD4 = 0;
    SPI1BUF = Address;
    SPI1BUF = LowerByte;
    if (NumBytes == 2) {
        SPI1BUF = UpperByte;
    }
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    LATDbits.LATD4 = 1;
    
    uint8_t data1;
    while (!SPI1STATbits.SPIRBE) {
        data1 = SPI1BUF;
    }
    return;
}

void WriteIMU2(uint8_t Address, AccelGyroData_t data)
{
    __builtin_disable_interrupts();
    LATDbits.LATD4 = 0;
    SPI1BUF = Address;
    SPI1BUF = data.DataStruct.LowerByte;
    SPI1BUF = data.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    LATDbits.LATD4 = 1;
    
    uint8_t data1;
    while (!SPI1STATbits.SPIRBE) {
        data1 = SPI1BUF;
    }
    return;
}

void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2)
{
    __builtin_disable_interrupts();
    LATDbits.LATD4 = 0;
    SPI1BUF = Address;
    SPI1BUF = data1.DataStruct.LowerByte;
    SPI1BUF = data1.DataStruct.UpperByte;
    SPI1BUF = data2.DataStruct.LowerByte;
    SPI1BUF = data2.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    LATDbits.LATD4 = 1;
    
    uint8_t data;
    while (!SPI1STATbits.SPIRBE) {
        data = SPI1BUF;
    }
    return;
}

/**
 * Reads 1 byte (the lower byte) from the specified address
 * 
 * @param Address
 * @return 
 */
uint8_t ReadIMU8(uint8_t Address)
{
    while (!SPI1STATbits.SPIRBE) {
        uint8_t temp = SPI1BUF;
    }
    
    __builtin_disable_interrupts();
    LATDbits.LATD4 = 0;
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    LATDbits.LATD4 = 1;

    
    uint8_t temp = SPI1BUF;
    uint8_t dummy_data = SPI1BUF;
    uint8_t data = SPI1BUF;
    return data;
}

/**
 * Reads both the lower byte and the upper byte from the specified address
 * 
 * @param Address
 * @return 
 */
uint16_t ReadIMU16(uint8_t Address)
{
    while (!SPI1STATbits.SPIRBE) {
        uint8_t temp = SPI1BUF;
    }
    
    __builtin_disable_interrupts();
    LATDbits.LATD4 = 0;
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    SPI1BUF = 0x00;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    LATDbits.LATD4 = 1;
    
    uint8_t temp = SPI1BUF;
    uint8_t dummy_data = SPI1BUF;
    
    AccelGyroData_t data;
    data.DataStruct.LowerByte = SPI1BUF;
    data.DataStruct.UpperByte = SPI1BUF;
    return data.FullData;
}

/****************************************************************************
 Function
     ReadFIFO

 Parameters
     None

 Returns
     None

 Description
     Sends the correct sequence of bytes to read from the IMU FIFO
 
    ***Assumes the Bank is already correctly selected***
****************************************************************************/
void ReadFIFO(void)
{    
    __builtin_disable_interrupts();
    IEC3SET = _IEC3_SPI1TXIE_MASK;
    LATDbits.LATD4 = 0;
    SPI1BUF = READ | 0x16; // the FIFO data buffer
    SPI1BUF = 0x00; // This is for the dummy message
    
    // Transmission of 6 16-bit words
    for (uint8_t i=0; i<12; i++) {
        SPI1BUF = 0x00;
    }
    __builtin_enable_interrupts();
}

void __ISR(_SPI1_RX_VECTOR, IPL7SRS) SPI1RXHandler(void)
{
    static uint8_t data_read = 0;
    
    while (SPI1STATbits.SPIRBE == 0){
        rx_data[data_read] = SPI1BUF;
        IFS3CLR = _IFS3_SPI1RXIF_MASK; // clear the interrupt flag 
        data_read += 1;
        
        if (IntStatusReading && data_read == 4) {
            // Now start the fifo reading
            data_read = 0;
            IntStatusReading = false;
            FifoReading = true;
                        
            ReadFIFO();
        } else if (FifoReading && data_read == 14) {
            Accel[0].DataStruct.LowerByte = rx_data[2];
            Accel[0].DataStruct.UpperByte = rx_data[3];
            Accel[1].DataStruct.LowerByte = rx_data[4];
            Accel[1].DataStruct.UpperByte = rx_data[5];
            Accel[2].DataStruct.LowerByte = rx_data[6];
            Accel[2].DataStruct.UpperByte = rx_data[7];
            Gyro[0].DataStruct.LowerByte = rx_data[8];
            Gyro[0].DataStruct.UpperByte = rx_data[9];
            Gyro[1].DataStruct.LowerByte = rx_data[10];
            Gyro[1].DataStruct.UpperByte = rx_data[11];
            Gyro[2].DataStruct.LowerByte = rx_data[12];
            Gyro[2].DataStruct.UpperByte = rx_data[13];
            DB_printf("Gyro Z: %d\r\n", Gyro[2].FullData);
            FifoReading = false;
            LATHbits.LATH4 = 1;
            DB_printf("HERE!\r\n");
        }
    }
}

void __ISR(_SPI1_TX_VECTOR, IPL7SRS) SPI1TXHandler(void)
{
    LATDbits.LATD4 = 1; // Set CS High again
    IEC3CLR = _IEC3_SPI1TXIE_MASK; // Disable the interrupt
    IFS3CLR = _IFS3_SPI1TXIF_MASK; // clear the interrupt flag 
}

// Interrupt indicates new IMU data is read: Initiate FIFO reading
void __ISR(_EXTERNAL_2_VECTOR, IPL7SRS) External2Handler(void)
{
    static uint8_t counts = 0;
    counts += 1;
    if (counts>10) {
        LATHbits.LATH7 = 1;
    }
    
    DB_printf("Ext Interrupt\r\n");
    IFS0CLR = _IFS0_INT2IF_MASK; // Clear the interrupt flag
    
    LATHbits.LATH5 = 1;
    
    __builtin_disable_interrupts();
    IEC3SET = _IEC3_SPI1TXIE_MASK;
    LATDbits.LATD4 = 0;
    SPI1BUF = READ | 0x0D; // INT1 Status register
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    SPI1BUF = 0x00;
    __builtin_enable_interrupts();
    
    // Enable SPIRX Interrupt  
    IntStatusReading = true;
    IEC3SET = _IEC3_SPI1RXIE_MASK;
}