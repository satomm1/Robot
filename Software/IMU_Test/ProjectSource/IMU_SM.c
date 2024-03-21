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
void WriteIMU(uint8_t Address, uint8_t LowerByte, uint8_t UpperByte, uint8_t NumBytes);
uint8_t ReadIMU1(uint8_t Address);
uint16_t ReadIMU2(uint8_t Address);
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
  SPI1CONbits.SMP = 0; // Data sampled at middle of data output time
  SPI1CONbits.CKE = 1; // Serial output data changes on transition from active clock state to idle clock state
  SPI1CONbits.CKP = 1; // Idle state for the clock is high level
  SPI1CONbits.MSTEN = 1; // Host mode
  SPI1CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI1CONbits.STXISEL = 0b00; // Interrupt generated when last transfer shifted out of SPISR and transmit operations are complete
  SPI1CONbits.SRXISEL = 0b11; // Interrupt is generated when the buffer is full

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
  IEC0CLR = _IEC0_INT2IE_MASK;
  INTCONbits.INT2EP = 0; // Interrupt on falling edge
  
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
        uint8_t data = ReadIMU1(0x4F);
        DB_printf("Chip ID: %d", data);
        
        InitIMU();        
        
        // now put the machine into the actual initial state
        CurrentState = IMUWait;
      }
    }
    break;

    case IMUWait:      
    {
      switch (ThisEvent.EventType)
      {
        case EV_IMU_DATA_UPDATE:  
        { 
            DB_printf("Received New IMU Data\r\n");
            
          uint8_t indx = ThisEvent.EventParam;
          Accel[0].DataStruct.UpperByte = IMU_FIFO[indx][1];
          Accel[0].DataStruct.LowerByte = IMU_FIFO[indx][2];
          Accel[1].DataStruct.UpperByte = IMU_FIFO[indx][3];
          Accel[1].DataStruct.LowerByte = IMU_FIFO[indx][4];
          Accel[2].DataStruct.UpperByte = IMU_FIFO[indx][5];
          Accel[2].DataStruct.LowerByte = IMU_FIFO[indx][6];
               
          Gyro[0].DataStruct.UpperByte = IMU_FIFO[indx][7];
          Gyro[0].DataStruct.LowerByte = IMU_FIFO[indx][8];
          Gyro[1].DataStruct.UpperByte = IMU_FIFO[indx][9];
          Gyro[1].DataStruct.LowerByte = IMU_FIFO[indx][10];
          Gyro[2].DataStruct.UpperByte = IMU_FIFO[indx][11];
          Gyro[2].DataStruct.LowerByte = IMU_FIFO[indx][12];       
          
          for (uint8_t i=1; i<3; i++) {
            Accel_g[i] = Accel[i].FullData / ACCEL_SENSITIVITY;
            Gyro_deg_s[i] = Gyro[i].FullData / GYRO_SENSITIVITY;
          }
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
    // Get the chip id
    uint8_t chipID = ReadIMU1(0x00);
    
    // Configure accelerometer: ODR rate to 25 Hz and sensitivity to +/- 8g
    WriteIMU(0x20, 0b00100110, 0b01110000, 2); 
    
    // Configure gyroscope: ODR rate to 25 Hz and sensitivity to +/- 250 deg/sec
    WriteIMU(0x21, 0b00010110, 0b01110000, 2); 
    
    // TODO Other imu settings
    
    // Enable the Int 2 Interrupt on the PIC32
    IFS0CLR = _IFS0_INT2IF_MASK; // Clear the interrupt flag
    IEC0SET = _IEC0_INT2IE_MASK;
    
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
    SPI1BUF = WRITE | Address;
    SPI1BUF = LowerByte;
    if (NumBytes == 2) {
        SPI1BUF = UpperByte;
    }
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    
    uint8_t data1;
    while (!SPI1STATbits.SPIRBE) {
        data1 = SPI1BUF;
    }
    return;
}

/**
 * Reads 1 byte (the lower byte) from the specified address
 * 
 * @param Address
 * @return 
 */
uint8_t ReadIMU1(uint8_t Address)
{
    while (!SPI1STATbits.SPIRBE) {
        uint8_t temp = SPI1BUF;
    }
    
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    
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
uint16_t ReadIMU2(uint8_t Address)
{
    while (!SPI1STATbits.SPIRBE) {
        uint8_t temp = SPI1BUF;
    }
    
    SPI1BUF = READ | Address; // Specify the address of data we want to receive
    SPI1BUF = 0x00; // This is for the dummy message
    SPI1BUF = 0x00;
    SPI1BUF = 0x00;
    
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
    FIFO_index = 0;
    if (Active_FIFO) {
        Active_FIFO = 0;
    } else {
        Active_FIFO = 1;
    }
    
    // Turn on TX interrupt -- Interrupt will be generated after this is 
    // completely transferred
    IEC3SET = _IEC3_SPI1TXIE_MASK;
    
    // Write To FIFO_Data Register
    SPI1BUF = READ | 0x30;
}

void __ISR(_SPI1_RX_VECTOR, IPL7SRS) SPI1RXHandler(void)
{
    static ES_Event_t NewEvent = {EV_IMU_DATA_UPDATE, 0}; // static for speed
    
    // Read all the data from SPI1Buffer -- Must make sure we visit this before 
    // sending more data to ensure we don't have overflow
    for (uint8_t i=0; i<16; i++) {
        IMU_FIFO[Active_FIFO][FIFO_index] = SPI1BUF;
    }
    
    Readable_FIFO = Active_FIFO; // Update what FIFO contains valid info
    IFS3CLR = _IFS3_SPI1RXIF_MASK; // clear the interrupt flag
    
    NewEvent.EventParam = Active_FIFO;
    PostImuSM(NewEvent);
    
    Accel[0].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][2];
    Accel[0].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][1];
    Accel[1].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][4];
    Accel[1].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][3];
    Accel[2].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][6];
    Accel[2].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][5];

    Gyro[0].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][8];
    Gyro[0].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][7];
    Gyro[1].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][10];
    Gyro[1].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][9];
    Gyro[2].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][12];
    Gyro[2].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][11];
    
    DB_printf("In SPI1RX Handler");
}

void __ISR(_SPI1_TX_VECTOR, IPL7SRS) SPI1TXHandler(void)
{
    uint8_t temp; // static for speed
    
    IEC3CLR = _IEC3_SPI1TXIE_MASK; // Disable TX interrupt
    
    temp = SPI1BUF; // Empty buffer that holds junk data

    IFS3CLR = _IFS3_SPI1TXIF_MASK; // clear the interrupt flag
    
    // Write 16 times via SPI to get data from IMU FIFO
    while (!SPI1STATbits.SPIRBE) {
        for (uint8_t i=0; i<FIFO_PACKET_SIZE; i++) {
            SPI1BUF = 0;
        }
    }
    
    DB_printf("In SPITX Handler");
}

// Interrupt indicates new IMU data is read: Initiate FIFO reading
void __ISR(_EXTERNAL_2_VECTOR, IPL7SRS) External2Handler(void)
{
    IFS0CLR = _IFS0_INT2IF_MASK; // Clear the interrupt flag
    ReadFIFO();   
}