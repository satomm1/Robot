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
#include <math.h>

/*----------------------------- Module Defines ----------------------------*/
#define READ  0b10000000
#define WRITE 0b00000000
#define FIFO_PACKET_SIZE 16
#define ACCEL_SENSITIVITY 4096 // LSB/g
#define GYRO_SENSITIVITY 131 // LSB/(deg/s)
#define ACCEL_MAX 8  // 8g
#define GYRO_MAX 250 // deg/sec

#define TWO_KP 2 * 5
#define TWO_KI 2*0
#define DT 0.00999936

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitIMU(void);
void ResetIMU(void);
void WriteIMU(uint8_t Address, uint8_t LowerByte, uint8_t UpperByte, uint8_t NumBytes);
void WriteIMU2(uint8_t Address, AccelGyroData_t data);
void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2);
void PrintImuData(void);
void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt);;

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

static volatile __SPI1CONbits_t * pSPICON;
static volatile __SPI1CON2bits_t * pSPICON2;
static volatile __SPI1STATbits_t * pSPISTAT;
static volatile uint32_t * pSPIBRG;
static volatile uint32_t * pSPIBUF;

// Quaternion State
static volatile float q0 = 1.0;
static volatile float q1 = 0.0;
static volatile float q2 = 0.0;
static volatile float q3 = 0.0;

static float integralFBx = 0.0;
static float integralFBy = 0.0;
static float integralFBz = 0.0;


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
  
  if (PCB_REV == 1) {
    // Set SPI4 Pins to correct input or output setting
    TRISACLR = _TRISA_TRISA15_MASK;
    TRISDCLR = _TRISD_TRISD9_MASK | _TRISD_TRISD10_MASK; // Set SCK4, SS4, SDO4 to output
    TRISDSET = _TRISD_TRISD11_MASK; // Set SDI4 to input
      
    // Map SPI4 Pins to correct function
    // RD10 is mapped to CLK4 by default
    RPD9R = 0b1000; // Map RD4 -> SS4
    RPA15R = 0b1000; // Map RA15 -> SDO4
    SDI4R = 0b0011; // Map SDI3 -> RD11
      
    pSPICON = (__SPI1CONbits_t *)&SPI4CON;
    pSPICON2 = (__SPI1CON2bits_t *)&SPI4CON2;
    pSPIBRG = &SPI4BRG;
    pSPIBUF = &SPI4BUF;
    pSPISTAT = (__SPI1STATbits_t *)&SPI4STAT;
    
    SPI4CON = 0;
    SPI4CON2 = 0;
  } else if (PCB_REV == 2) {
//     Set interrupt pins to inputs
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
    
    pSPICON = (__SPI1CONbits_t *)&SPI1CON;
    pSPICON2 = (__SPI1CON2bits_t *)&SPI1CON2;
    pSPIBRG = &SPI1BRG;
    pSPIBUF = &SPI1BUF;
    pSPISTAT = (__SPI1STATbits_t *)&SPI1STAT;
    
    SPI1CON = 0;
    SPI1CON2 = 0;
  }
     
  // Initialize SPIxCON
  pSPICON->FRMEN = 0; // Disable framed SPI support
  pSPICON->FRMPOL = 0; // SS1 is active low
  pSPICON->MSSEN = 1; // SS is automatically driven
  pSPICON->MCLKSEL = 0; // Use PBCLK2 for the Baud Rate Generator (50 MHz)
  pSPICON->ENHBUF = 1; // Enhance buffer enabled (use FIFOs)
  pSPICON->DISSDO = 0; // SDO1 is used by the module
  pSPICON->MODE32 = 0; // 8 bit mode
  pSPICON->MODE16 = 0; // 8 bit mode
  pSPICON->SMP = 1; // Data sampled at middle of data output time
  pSPICON->CKE = 1; // Serial output data changes on transition from active clock state to idle clock state
  pSPICON->CKP = 1; // Idle state for the clock is high level
  pSPICON->MSTEN = 1; // Host mode
  pSPICON->DISSDI = 0; // The SDI pin is controlled by the module
  pSPICON->STXISEL = 0b00; // Interrupt generated when last transfer shifted out of SPISR and transmit operations are complete
  pSPICON->SRXISEL = 0b01; // Interrupt is generated when the buffer is not empty

  pSPICON2->AUDEN = 0; // Audio protocol is disabled
  
  while (!pSPISTAT->SPIRBE){
      uint8_t ClearData = *pSPIBUF;
  }
  pSPISTAT->SPIROV = 0; // Clear the Receive overflow bit
  
  *pSPIBRG = 15; // 1.56 MHz clock frequency, IMU Has max frequency of 10 MHz
    
  
  // Setup Timer 6
  T6CON = 0;
  T6CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T6CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR6 = 1953; // ~100 Hz
  TMR6 = 0; // Set TMR6 to 0
  
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  if (PCB_REV == 1) {
    IPC41bits.SPI4TXIP = 7; // SPI4TX
    IPC41bits.SPI4RXIP = 7; // SPI4RX
  } else if (PCB_REV == 2) {
    IPC27bits.SPI1TXIP = 7; // SPI1TX
    IPC27bits.SPI1RXIP = 7; // SPI1RX
  }
  IPC7bits.T6IP = 7; // T6
  
  // Disable the RX/TX interrupt
  if (PCB_REV == 1) {
    IEC5CLR = _IEC5_SPI4RXIE_MASK | _IEC5_SPI4TXIE_MASK; // SPI4
  } else if (PCB_REV == 2) {
    IEC3CLR = _IEC3_SPI1RXIE_MASK | _IEC3_SPI1TXIE_MASK; // SPI1
  }
  
  // Clear interrupt flags
  if (PCB_REV == 1) {
    IFS5CLR = _IFS5_SPI4RXIF_MASK | _IFS5_SPI4TXIF_MASK; // SPI4
  } else if (PCB_REV == 2) {
    IFS3CLR = _IFS3_SPI1RXIF_MASK | _IFS3_SPI1TXIF_MASK; // SPI1
  }
  IFS0CLR = _IFS0_T6IF_MASK; // T6
  
  // Enable the T6 interrupt
   IEC0SET = _IEC0_T6IE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
   pSPICON->ON = 1; // Finally turn the SPI module on
    
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
            if (PCB_REV == 1){
                IEC5SET = _IEC5_SPI4RXIE_MASK;
            } else if (PCB_REV == 2){
                IEC3SET = _IEC3_SPI1RXIE_MASK; // Enable the RX interrupt
            }
//            ES_Timer_InitTimer(IMU_TIMER, 1000); // Init timer
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
            PrintImuData();
            
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

void GetIMUData(float *ImuResults)
{
    static int16_t signed_data; // Static for speed
    
    if (Accel[0].FullData & 0x8000) {
        signed_data = -((~Accel[0].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[0].FullData;
    }
    ImuResults[0] = (float)signed_data / 8.19 * 9.81 / 1000;

    if (Accel[1].FullData & 0x8000) {
        signed_data = -((~Accel[1].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[1].FullData;
    }
    ImuResults[1] = (float)signed_data / 8.19 * 9.81 / 1000;
    
    if (Accel[2].FullData & 0x8000) {
        signed_data = -((~Accel[2].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[2].FullData;
    }
    ImuResults[2] = (float)signed_data / 8.19 * 9.81 / 1000;

    if (Gyro[0].FullData & 0x8000) {
        signed_data = -((~Gyro[0].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[0].FullData;
    }
    ImuResults[3] = (float)signed_data / 131.2;
    
    if (Gyro[1].FullData & 0x8000) {
        signed_data = -((~Gyro[1].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[1].FullData;
    }
    ImuResults[4] = (float)signed_data / 131.2;

    if (Gyro[2].FullData & 0x8000) {
        signed_data = -((~Gyro[2].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[2].FullData;
    }
    ImuResults[5] = (float)signed_data / 131.2;
    
    return;
}

void WriteImuToSPI(uint8_t *Message2Send)
{
  float roll;
  float pitch;
  GetAngles(&roll, &pitch);
    
  Message2Send[0] = 9; // 9 indicates we are imu data (byte 1)
    
  // The roll and pitch data are all floats. The floats can be sent as 4 
  // chunks of 8 bits.
  
  // Now write the roll (bytes 2-5)
  uint32_t roll_as_int = *((uint32_t*)&roll);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+1] = (roll_as_int >> (24-8*j)) & 0xFF;
  }
  
  // Now write the pitch (bytes 6-9)
  uint32_t pitch_as_int = *((uint32_t*)&pitch);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+5] = (pitch_as_int >> (24-8*j)) & 0xFF;
  }
  
  for (uint8_t j = 0; j < 7; j++) {
    Message2Send[j+13] = 0; // Fill rest of buffer with 0's
  }
}

/** 
 * GetAngles
 * 
 * Computes the roll/pitch of the IMU from the Mahony Filter
 * (Degrees/second)
 * 
 * @param roll - the roll in degrees
 * @param pitch - the pitch in degrees
 */
void GetAngles(float* roll, float* pitch)
{
    // Compute pitch and roll (in degrees)
    *roll = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 + q2*q2) * 57.29578;
    *pitch = asinf(2.0 * (q0*q2 - q3*q1)) * 57.29578;
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
    data2send.DataStruct.LowerByte = 0b00011001; // cutoff = acc_odr/2, acc_range = +/- 4g, 8.19 LSB/mg, Sample Rate = 200 Hz
    data2send.DataStruct.UpperByte = 0b01000001; // Normal mode, averaging of 2 samples

    // Gyro
    data2send2.DataStruct.LowerByte = 0b00011001; // cutoff = gyr_odr/2, gyr_range = +/- 250 deg/s, 131.2 LSB/deg/s, Sample Rate = 200 Hz
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
    *pSPIBUF = Address;
    *pSPIBUF = LowerByte;
    if (NumBytes == 2) {
        *pSPIBUF = UpperByte;
    }
    __builtin_enable_interrupts();
    
    while (pSPISTAT->SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    
    uint8_t data1;
    while (!pSPISTAT->SPIRBE) {
        data1 = *pSPIBUF;
    }
    return;
}

void WriteIMU2(uint8_t Address, AccelGyroData_t data)
{
    __builtin_disable_interrupts();
    *pSPIBUF = Address;
    *pSPIBUF = data.DataStruct.LowerByte;
    *pSPIBUF = data.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (pSPISTAT->SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
    uint8_t data1;
    while (!pSPISTAT->SPIRBE) {
        data1 = *pSPIBUF;
    }
    return;
}

void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2)
{
    __builtin_disable_interrupts();
    *pSPIBUF = Address;
    *pSPIBUF = data1.DataStruct.LowerByte;
    *pSPIBUF = data1.DataStruct.UpperByte;
    *pSPIBUF = data2.DataStruct.LowerByte;
    *pSPIBUF = data2.DataStruct.UpperByte;
    __builtin_enable_interrupts();
    
    while (pSPISTAT->SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
    uint8_t data;
    while (!pSPISTAT->SPIRBE) {
        data = *pSPIBUF;
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
    while (!pSPISTAT->SPIRBE) {
        uint8_t temp = *pSPIBUF;
    }

    __builtin_disable_interrupts();
    *pSPIBUF = READ | Address; // Specify the address of data we want to receive
    *pSPIBUF = 0x00; // This is for the dummy message
    *pSPIBUF = 0x00;
    __builtin_enable_interrupts();
    
    while (pSPISTAT->SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }

    
    uint8_t temp = *pSPIBUF;
    uint8_t dummy_data = *pSPIBUF;
    uint8_t data = *pSPIBUF;
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
    while (!pSPISTAT->SPIRBE) {
        uint8_t temp = *pSPIBUF;
    }
    
    __builtin_disable_interrupts();
    *pSPIBUF = READ | Address; // Specify the address of data we want to receive
    *pSPIBUF = 0x00; // This is for the dummy message
    *pSPIBUF = 0x00;
    *pSPIBUF = 0x00;
    __builtin_enable_interrupts();
    
    while (pSPISTAT->SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
    }
    
    uint8_t temp = *pSPIBUF;
    uint8_t dummy_data = *pSPIBUF;
    
    AccelGyroData_t data;
    data.DataStruct.LowerByte = *pSPIBUF;
    data.DataStruct.UpperByte = *pSPIBUF;
    return data.FullData;
}

void PrintImuData(void)
{
    int16_t signed_data;
    if (Accel[0].FullData & 0x8000) {
        signed_data = -((~Accel[0].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[0].FullData;
    }
    float x_accel = (float)signed_data / 8.19 * 9.81 / 1000;
    DB_printf("Accel x: %d m/s^2\r\n", (int16_t)x_accel);

    if (Accel[1].FullData & 0x8000) {
        signed_data = -((~Accel[1].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[1].FullData;
    }
    float y_accel = (float)signed_data / 8.19 * 9.81 / 1000;
    DB_printf("Accel y: %d m/s^2\r\n", (int16_t)y_accel);

    if (Accel[2].FullData & 0x8000) {
        signed_data = -((~Accel[2].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Accel[2].FullData;
    }
    float z_accel = (float)signed_data / 8.19 * 9.81 / 1000;
    DB_printf("Accel z: %d m/s^2\r\n", (int16_t)z_accel);

    if (Gyro[0].FullData & 0x8000) {
        signed_data = -((~Gyro[0].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[0].FullData;
    }
    float x_vel = (float)signed_data / 131.2;
    DB_printf("Vel x: %d deg/sec\r\n", (int16_t)x_vel);
    
    if (Gyro[1].FullData & 0x8000) {
        signed_data = -((~Gyro[1].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[1].FullData;
    }
    float y_vel = (float)signed_data / 131.2;
    DB_printf("Vel y: %d deg/sec\r\n", (int16_t)y_vel);

    if (Gyro[2].FullData & 0x8000) {
        signed_data = -((~Gyro[2].FullData & 0xFFFF) + 1);
    } else {
        signed_data = Gyro[2].FullData;
    }
    float z_vel = (float)signed_data / 131.2;
    DB_printf("Vel z: %d deg/sec\r\n\r\n", (int16_t)z_vel);
}

/**
 * MahonyUpdate
 * This function performs a Mahony filter update using accelerometer and 
 * gyroscope measurements. Code modified from: https://github.com/PaulStoffregen/MahonyAHRS/blob/master/src/MahonyAHRS.cpp#L170
 * 
 * @param ax - acceleration in x direction (acceleration units can be anything, but must be consistent)
 * @param ay - acceleration in y direction
 * @param az - acceleration in z direction
 * @param gx - angular velocity in x direction (rad/sec)
 * @param gy - angular velocity in y direction (rad/sec)
 * @param gz - angular velocity in z direction (rad/sec)
 * @param dt - time step (seconds)
 */
void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    static float recipNorm = 1.0; // static for speed
    static float halfvx = 0.0;
    static float halfvy = 0.0;
    static float halfvz = 0.0;
    static float halfex = 0.0;
    static float halfey = 0.0;
    static float halfez = 0.0;
    static float qa = 0.0;
    static float qb = 0.0;
    static float qc = 0.0;
        
    // Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533;
	gy *= 0.0174533;
	gz *= 0.0174533;
    
    // Normalize accelerometer
    recipNorm = 1/sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimate the direction of gravity
    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5 + q3*q3;
    
    // Error (cross product of estimated and measured direction of gravity)
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    
    // Apply integral feedback
    if (TWO_KI > 0) {
        // integral error scaled by Ki
        integralFBx += TWO_KI * halfex * dt;
        integralFBy += TWO_KI * halfey * dt;
        integralFBz += TWO_KI * halfez * dt;
        gx += integralFBx;	// apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;	// prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    
    // Apply proportional feedback
    gx += TWO_KP * halfex;
    gy += TWO_KP * halfey;
    gz += TWO_KP * halfez;
    
    // Integrate rate of change of quaternion
	gx *= (0.5 * dt);		// pre-multiply common factors
	gy *= (0.5 * dt);
	gz *= (0.5 * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalize quaternion
	recipNorm = 1/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

#if (PCB_REV ==  1)
void __ISR(_SPI4_RX_VECTOR, IPL7SRS) SPI4RXHandler(void)
{
    static uint8_t data_read = 0;
    static float imu_data[6];
    
    while (pSPISTAT->SPIRBE == 0){
        rx_data[data_read] = *pSPIBUF;
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
            
            // Convert IMU Data to signed data
            GetIMUData(imu_data);
            
            // Do a Mahony Filter Update
            MahonyUpdate(imu_data[0], imu_data[1], imu_data[2], imu_data[3],
                            imu_data[4], imu_data[5], DT);       
        }
    }
    IFS5CLR = _IFS5_SPI4RXIF_MASK; // clear the interrupt flag 
}

void __ISR(_SPI4_TX_VECTOR, IPL7SRS) SPI4TXHandler(void)
{
    IEC5CLR = _IEC5_SPI4TXIE_MASK; // Disable the interrupt
    IFS5CLR = _IFS5_SPI4TXIF_MASK; // clear the interrupt flag 
}
#elif (PCB_REV == 2)
void __ISR(_SPI1_RX_VECTOR, IPL7SRS) SPI1RXHandler(void)
{
    static uint8_t data_read = 0;
    static float imu_data[6];
    
    while (pSPISTAT->SPIRBE == 0){
        rx_data[data_read] = *pSPIBUF;
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
            
            // Convert IMU Data to signed data
            GetIMUData(imu_data);
            
            // Do a Mahony Filter Update
            MahonyUpdate(imu_data[0], imu_data[1], imu_data[2], imu_data[3],
                            imu_data[4], imu_data[5], DT);       
        }
    }
    IFS3CLR = _IFS3_SPI1RXIF_MASK; // clear the interrupt flag 
}

void __ISR(_SPI1_TX_VECTOR, IPL7SRS) SPI1TXHandler(void)
{
    IEC3CLR = _IEC3_SPI1TXIE_MASK; // Disable the interrupt
    IFS3CLR = _IFS3_SPI1TXIF_MASK; // clear the interrupt flag 
}
#endif

void __ISR(_TIMER_6_VECTOR, IPL7SRS) T6Handler(void)
{
    // Clear the interrupt flag
    IFS0CLR = _IFS0_T6IF_MASK;
    
    // Get the current accel/gyroscope readings
    __builtin_disable_interrupts();
    *pSPIBUF = READ | 0x03; // Accel x address
    *pSPIBUF = 0x00; // This is for the dummy message
    for (uint8_t i=0; i<12; i++) {
        *pSPIBUF = 0x00; // 12 Messages for 12 bytes of accel/gyro data
    }
    __builtin_enable_interrupts();
}