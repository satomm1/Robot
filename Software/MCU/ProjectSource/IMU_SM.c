/****************************************************************************
 Module
   IMU_SM.c

 Description
   This is a file for implementing reading from the Bosch BMI323 6-axis IMU

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
  RPD4R = 0b0101; // Map RD4 -> SS1
  RPD3R = 0b0101; // Map RD3 -> SDO1
  SDI1R = 0b0000; // Map SDI1 -> RD2
    
  // Initialize SPI1
  SPI1CON = 0; // Reset SPI1CON settings
  SPI1CONbits.FRMEN = 0; // Disable framed SPI support
  SPI1CONbits.FRMPOL = 0; // SS1 is active low
  SPI1CONbits.MSSEN = 1; // SS is automatically driven
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
    
  
  // Setup Timer 6
  T6CON = 0;
  T6CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T6CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR6 = 3906; // ~50 Hz
  TMR6 = 0; // Set TMR5 to 0
  
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC27bits.SPI1TXIP = 7; // SPI2TX
  IPC27bits.SPI1RXIP = 7; // SPI2RX
  IPC7bits.T6IP = 7; // T6
  
  // Clear interrupt flags
  IFS3CLR = _IFS3_SPI1RXIF_MASK | _IFS3_SPI1TXIF_MASK;
  IFS0CLR = _IFS0_T6IF_MASK;
  
  // Disable the RX/TX interrupt
  IEC3CLR = _IEC3_SPI1RXIE_MASK | _IEC3_SPI1TXIE_MASK;
  
  // Enable the T6 interrupt
  IEC0SET = _IEC0_T6IE_MASK;
  
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
            T6CONbits.ON = 1; // Turn T6 on
            IEC3SET = _IEC3_SPI1RXIE_MASK; // Enable the RX interrupt
            ES_Timer_InitTimer(IMU_TIMER, 1000); // Init timer
            CurrentState = IMURun;
        }
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
//            DB_printf("Status: %d\r\n", ReadIMU16(0x02));
//                    
//            int16_t signed_data;
//            if (Accel[0].FullData & 0x8000) {
//                signed_data = -((~Accel[0].FullData & 0xFFFF) + 1);
//            } else {
//                signed_data = Accel[0].FullData;
//            }
//            float x_accel = (float)signed_data / 8.19 * 9.81 / 1000;
//            DB_printf("Accel x: %d m/s^2\r\n", (int16_t)x_accel);
//            
//            if (Accel[1].FullData & 0x8000) {
//                signed_data = -((~Accel[1].FullData & 0xFFFF) + 1);
//            } else {
//                signed_data = Accel[1].FullData;
//            }
//            float y_accel = (float)signed_data / 8.19 * 9.81 / 1000;
//            DB_printf("Accel y: %d m/s^2\r\n", (int16_t)y_accel);
//            
//            
//            if (Gyro[2].FullData & 0x8000) {
//                signed_data = -((~Gyro[2].FullData & 0xFFFF) + 1);
//            } else {
//                signed_data = Gyro[2].FullData;
//            }
//            float z_vel = (float)signed_data / 131.2;
//            DB_printf("Vel z: %d deg/sec\r\n\r\n", (int16_t)z_vel);
//            
//            ES_Timer_InitTimer(IMU_TIMER, 1000);
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

void GetIMUData(float *ImuResults)
{
    int16_t signed_data;
    
    uint16_t accel_x_unsigned = Accel[0].FullData;
    uint16_t accel_y_unsigned = Accel[1].FullData;
    uint16_t vel_z_unsigned = Gyro[2].FullData;
    
    // X acceleration
    if (accel_x_unsigned & 0x8000) {
        signed_data = -((~accel_x_unsigned & 0xFFFF) + 1);
    } else {
        signed_data = accel_x_unsigned;
    }
    float x_accel = (float)signed_data / 8.19 * 9.81 / 1000;
//    DB_printf("Accel x: %d m/s^2\r\n", (int16_t)x_accel);

    // Y acceleration
    if (accel_y_unsigned & 0x8000) {
        signed_data = -((~accel_y_unsigned & 0xFFFF) + 1);
    } else {
        signed_data = accel_y_unsigned;
    }
    float y_accel = (float)signed_data / 8.19 * 9.81 / 1000;
//    DB_printf("Accel y: %d mG's\r\n", (int16_t)y_accel);

    // Z angular velocity
    if (vel_z_unsigned & 0x8000) {
        signed_data = -((~vel_z_unsigned & 0xFFFF) + 1);
    } else {
        signed_data = vel_z_unsigned;
    }
    float z_vel = (float)signed_data / 131.2;
//    DB_printf("Vel z: %d deg/sec\r\n\r\n", (int16_t)z_vel);
    
    ImuResults[0] = x_accel;
    ImuResults[1] = y_accel;
    ImuResults[2] = z_vel;
    
    return;
}

void WriteImuToSPI(uint8_t *Message2Send)
{
    float imu_data[3];
    GetIMUData(imu_data);
    
    Message2Send[0] = 9; // 9 indicates we are imu data (byte 1)
    
  // The x/y accel and z vel data are all floats. The floats can be sent as 4 
  // chunks of 8 bits.
  
  // Now write the x accel data (bytes 2-5)
  uint32_t x_as_int = *((uint32_t*)&imu_data[0]);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+1] = (x_as_int >> (24-8*j)) & 0xFF;
  }
  
  // Now write the y accel data (bytes 6-9)
  uint32_t y_as_int = *((uint32_t*)&imu_data[1]);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+5] = (y_as_int >> (24-8*j)) & 0xFF;
  }
  
  // Now write the z angular velocity data (bytes 10-13)
  uint32_t z_as_int = *((uint32_t*)&imu_data[2]);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+9] = (z_as_int >> (24-8*j)) & 0xFF;
  }
  
  for (uint8_t j = 0; j < 3; j++) {
    Message2Send[j+13] = 0; // Fill rest of buffer with 0's
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
    AccelGyroData_t data2send2;
    
    uint16_t data = ReadIMU16(0x00); // Dummy call to set up SPI  
    
    data = ReadIMU8(0x00); // Get chip ID
    while (data != 0b01000011) {
        DB_printf("Incorrect Chip ID: %d\r\n", data);
        data = ReadIMU16(0x00); // Get chip ID
    }
    DB_printf("Chip ID: %d\r\n", data);

    // Get the status of the chip
    data = ReadIMU16(0x02);
    DB_printf("IMU Status: %d\r\n", data);
                  
    // Setup the accelerometer/gyro settings for the IMU, do a burst write since 
    // addresses are consecutive:
    // Accel
    data2send.DataStruct.LowerByte = 0b00011000; // cutoff = acc_odr/2, acc_range = +/- 4g, 8.19 LSB/mg, Sample Rate = 100 Hz
    data2send.DataStruct.UpperByte = 0b01000001; // Normal mode, averaging of 2 samples

    // Gyro
    data2send2.DataStruct.LowerByte = 0b00011000; // cutoff = gyr_odr/2, gyr_range = +/- 250 deg/s, 131.2 LSB/deg/s, Sample Rate = 100 Hz
    data2send2.DataStruct.UpperByte = 0b01000001; // Normal mode, averaging of 2 samples
    WriteIMU2Transfer(0x20, data2send, data2send2);

    return;
}

/****************************************************************************
 Function
     ResetImu

 Parameters
     None

 Returns
     None

 Description
     Sends the correct sequence of writes to the IMU via SPI to correctly soft
     reset the IMU
****************************************************************************/
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
    SPI1BUF = Address;
    SPI1BUF = LowerByte;
    if (NumBytes == 2) {
        SPI1BUF = UpperByte;
    }
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    
    uint8_t data1;
    while (!SPI1STATbits.SPIRBE) {
        data1 = SPI1BUF;
    }
    return;
}

void WriteIMU2(uint8_t Address, AccelGyroData_t data)
{
    __builtin_disable_interrupts();
    SPI1BUF = Address;
    SPI1BUF = data.DataStruct.LowerByte;
    SPI1BUF = data.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
    uint8_t data1;
    while (!SPI1STATbits.SPIRBE) {
        data1 = SPI1BUF;
    }
    return;
}

void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2)
{
    __builtin_disable_interrupts();
    SPI1BUF = Address;
    SPI1BUF = data1.DataStruct.LowerByte;
    SPI1BUF = data1.DataStruct.UpperByte;
    SPI1BUF = data2.DataStruct.LowerByte;
    SPI1BUF = data2.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
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
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }

    
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
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    SPI1BUF = 0x00;
    __builtin_enable_interrupts();
    
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
    uint8_t temp = SPI1BUF;
    uint8_t dummy_data = SPI1BUF;
    
    AccelGyroData_t data;
    data.DataStruct.LowerByte = SPI1BUF;
    data.DataStruct.UpperByte = SPI1BUF;
    return data.FullData;
}

void __ISR(_SPI1_RX_VECTOR, IPL7SRS) SPI1RXHandler(void)
{
    static uint8_t data_read = 0;
    
    while (SPI1STATbits.SPIRBE == 0){
        rx_data[data_read] = SPI1BUF;
        data_read += 1;
        if (data_read == 14) {
            data_read = 0;
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
        }
    }
    IFS3CLR = _IFS3_SPI1RXIF_MASK; // clear the interrupt flag 
}

void __ISR(_SPI1_TX_VECTOR, IPL7SRS) SPI1TXHandler(void)
{
    IEC3CLR = _IEC3_SPI1TXIE_MASK; // Disable the interrupt
    IFS3CLR = _IFS3_SPI1TXIF_MASK; // clear the interrupt flag 
}

void __ISR(_TIMER_6_VECTOR, IPL7SRS) T6Handler(void)
{
    // Clear the interrupt flag
    IFS0CLR = _IFS0_T6IF_MASK;
    
    // Get the current accel/gyroscope readings
    __builtin_disable_interrupts();
    SPI1BUF = READ | 0x03; // Accel x address
    SPI1BUF = 0x00; // This is for the dummy message
    for (uint8_t i=0; i<12; i++) {
        SPI1BUF = 0x00; // 12 Messages for 12 bytes of accel/gyro data
    }
    __builtin_enable_interrupts();
}