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
#include "LEDService.h"
#include <sys/attribs.h>
#include "dbprintf.h"
#include <math.h>

/*----------------------------- Module Defines ----------------------------*/
/* BMI323 SPI protocol (BST-BMI323 datasheet) */
#define IMU_SPI_READ_FLAG       0x80u
#define IMU_REG_CHIP_ID         0x00u
#define IMU_REG_STATUS          0x02u
#define IMU_REG_DATA_0          0x03u   /* Accel X — start of accel/gyro burst */
#define IMU_REG_ACC_CONF        0x20u
#define IMU_REG_CMD             0x7Eu
#define BMI323_CHIP_ID_VAL      0x43u
#define BMI323_CMD_SOFT_RESET   0xDEAFu

/* Burst read: cmd echo + SPI dummy + 6 x int16 (3 accel + 3 gyro) */
#define IMU_BURST_RX_BYTES      14u
#define IMU_BURST_DATA_OFFSET   2u      /* First data byte index in rx_data[] */
#define IMU_BURST_DATA_BYTES    12u     /* Clocks after address + dummy */

/* ES_Timer delays (ms) */
#define IMU_RESET_DELAY_MS      500u
#define IMU_DEBUG_PRINT_MS      1000u
#define IMU_INIT_MAX_ATTEMPTS   5u
#define IMU_RECOVERY_MAX_ATTEMPTS 3u

/* T6 period ~10 ms (PR6=1953, prescale 256 @ 50 MHz PBCLK3) */
#define IMU_TRANSFER_STALE_TICKS  3u   /* ~30 ms stuck in SpiTransferBusy */

/* LEDService EventParam for EV_LED_ON / EV_LED_OFF (see LEDService.c) */
#define IMU_FAULT_LED             2u   /* LATH4 IMU init/recovery failed */

/* Must match ACC_CONF / GYR_CONF written in InitIMU (±4g, ±250 dps) */
#define ACCEL_RANGE_G           4
#define GYRO_RANGE_DPS          250
#define ACCEL_LSB_PER_G         8192.0f /* LSB/g at ±4g */
#define GYRO_LSB_PER_DPS        131.2f  /* LSB/(deg/s) at ±250 dps */
#define GRAVITY_MPS2            9.80665f

#define TWO_KP                  (2.0f * 5.0f)
#define TWO_KI                  0.0f
#define DT                      0.00999936f  /* ~100 Hz, matches T6 PR6 */

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
bool InitIMU(void);
void ResetIMU(void);
void WriteIMU(uint8_t Address, uint8_t LowerByte, uint8_t UpperByte, uint8_t NumBytes);
void WriteIMU2(uint8_t Address, AccelGyroData_t data);
void WriteIMU2Transfer(uint8_t Address, AccelGyroData_t data1, AccelGyroData_t data2);
void PrintImuData(void);
void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt);
static void ImuSpiFlushRx(void);
static void ImuSpiPushTxByte(uint8_t byte);
static void ImuSpiStartSampleBurst(void);
static void ImuSpiParseBurstFrame(void);
static void ImuStopSampling(void);
static void ImuBusRecover(void);
static void ImuMahonyReset(void);
static void ImuRequestRecovery(void);
static void ImuEnterRecovery(void);
static void ImuPostLed(ES_EventType_t event_type, uint8_t led);
static void ImuEnterInitFailed(void);
static float RawAccelToMps2(int16_t raw);
static float RawGyroToDegPerS(int16_t raw);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static ImuState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static AccelGyroData_t Accel[3];
static AccelGyroData_t Gyro[3];

static volatile bool SpiTransferBusy = false;
static volatile uint8_t ImuAllowRecovery = 0;
static volatile bool ImuRecoveryPosted = false;
static uint8_t ImuSpiRxCount = 0;
static uint8_t ImuInitAttempts = 0;
static uint8_t ImuRecoveryAttempts = 0;
static uint8_t ImuTransferStaleTicks = 0;
static uint8_t rx_data[IMU_BURST_RX_BYTES];

#if PCB_REV == 1
static volatile __SPI4CONbits_t * pSPICON;
static volatile __SPI4CON2bits_t * pSPICON2;
static volatile __SPI4STATbits_t * pSPISTAT;
#define IMU_SPI_RX_ISR_VECTOR  _SPI4_RX_VECTOR
#define IMU_SPI_TX_ISR_VECTOR  _SPI4_TX_VECTOR
#define ImuSpiClearRxIf()      (IFS5CLR = _IFS5_SPI4RXIF_MASK)
#define ImuSpiClearTxIf()      do { IEC5CLR = _IEC5_SPI4TXIE_MASK; IFS5CLR = _IFS5_SPI4TXIF_MASK; } while (0)
#elif PCB_REV >= 2
static volatile __SPI1CONbits_t * pSPICON;
static volatile __SPI1CON2bits_t * pSPICON2;
static volatile __SPI1STATbits_t * pSPISTAT;
#define IMU_SPI_RX_ISR_VECTOR  _SPI1_RX_VECTOR
#define IMU_SPI_TX_ISR_VECTOR  _SPI1_TX_VECTOR
#define ImuSpiClearRxIf()      (IFS3CLR = _IFS3_SPI1RXIF_MASK)
#define ImuSpiClearTxIf()      do { IEC3CLR = _IEC3_SPI1TXIE_MASK; IFS3CLR = _IFS3_SPI1TXIF_MASK; } while (0)
#endif
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
  
#if PCB_REV == 1
    // Set SPI4 Pins to correct input or output setting
    TRISACLR = _TRISA_TRISA15_MASK;
    TRISDCLR = _TRISD_TRISD9_MASK | _TRISD_TRISD10_MASK; // Set SCK4, SS4, SDO4 to output
    TRISDSET = _TRISD_TRISD11_MASK; // Set SDI4 to input
      
    // Map SPI4 Pins to correct function
    // RD10 is mapped to CLK4 by default
    RPD9R = 0b1000; // Map RD9 -> SS4
    RPA15R = 0b1000; // Map RA15 -> SDO4
    SDI4R = 0b0011; // Map SDI4 -> RD11
       
    pSPICON = (__SPI4CONbits_t *)&SPI4CON;
    pSPICON2 = (__SPI4CON2bits_t *)&SPI4CON2;
    pSPIBRG = &SPI4BRG;
    pSPIBUF = &SPI4BUF;
    pSPISTAT = (__SPI4STATbits_t *)&SPI4STAT;
    
    SPI4CON = 0;
    SPI4CON2 = 0;
#elif PCB_REV >= 2
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
#endif
     
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
  pSPICON->CKE = 0; // Serial output data changes on transition from active clock state to idle clock state
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
  PRISSbits.PRI5SS = 0b0101; // IPL5SRS handlers use shadow set 5 (below motor IPL6/7)
  
  // IMU SPI + sample timer at IPL5 — below motor control (7) and odometry T7 (6)
  if (PCB_REV == 1) {
    IPC41bits.SPI4TXIP = 5;
    IPC41bits.SPI4RXIP = 5;
  } else if (PCB_REV >= 2) {
    IPC27bits.SPI1TXIP = 5;
    IPC27bits.SPI1RXIP = 5;
  }
  IPC7bits.T6IP = 5;
  
  // Disable the RX/TX interrupt
  if (PCB_REV == 1) {
    IEC5CLR = _IEC5_SPI4RXIE_MASK | _IEC5_SPI4TXIE_MASK; // SPI4
  } else if (PCB_REV >= 2) {
    IEC3CLR = _IEC3_SPI1RXIE_MASK | _IEC3_SPI1TXIE_MASK; // SPI1
  }
  
  // Clear interrupt flags
  if (PCB_REV == 1) {
    IFS5CLR = _IFS5_SPI4RXIF_MASK | _IFS5_SPI4TXIF_MASK; // SPI4
  } else if (PCB_REV >= 2) {
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
        ImuInitAttempts = 0;
        ResetIMU();
        CurrentState = IMUReset;
        ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
      }
    }
    break;

    case IMUReset:      
    {
      switch (ThisEvent.EventType)
      {
               
        case ES_TIMEOUT:
        {
            bool init_success = InitIMU();
            if (init_success) {
                ImuInitAttempts = 0;
                CurrentState = IMUWait;
                ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
            } else {
                ImuInitAttempts += 1;
                if (ImuInitAttempts < IMU_INIT_MAX_ATTEMPTS) {
                    ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
                } else {
                    DB_printf("IMU init failed after %u attempts; giving up\r\n",
                              (unsigned)IMU_INIT_MAX_ATTEMPTS);
                    ImuEnterInitFailed();
                }
            }
        }
        break;
        
        default:
          ;
      }
      
    }
    break;

    case IMUInitFailed:
      /* No further init attempts — avoids blocking SPI in RunImuSM */
      break;
    
    case IMUWait:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
        {
            T6CONbits.ON = 1;
            if (PCB_REV == 1) {
                IEC5SET = _IEC5_SPI4RXIE_MASK;
            } else if (PCB_REV >= 2) {
                IEC3SET = _IEC3_SPI1RXIE_MASK;
            }
            CurrentState = IMURun;
            ImuAllowRecovery = 1;
            ImuRecoveryPosted = false;
            ImuTransferStaleTicks = 0;
        }
      }
    }
    break;

    case IMURecovering:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
        {
            if (InitIMU()) {
                ImuRecoveryAttempts = 0;
                CurrentState = IMUWait;
                ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
            } else {
                ImuRecoveryAttempts += 1;
                if (ImuRecoveryAttempts < IMU_RECOVERY_MAX_ATTEMPTS) {
                    ImuBusRecover();
                    ResetIMU();
                    ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
                } else {
                    DB_printf("IMU recovery failed after %u attempts\r\n",
                              (unsigned)IMU_RECOVERY_MAX_ATTEMPTS);
                    ImuEnterInitFailed();
                }
            }
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
        case EV_IMU_RECOVERY:
        {
            ImuRecoveryPosted = false;
            ImuEnterRecovery();
        }
        break;

        case ES_TIMEOUT:
        {
            PrintImuData();
            ES_Timer_InitTimer(IMU_TIMER, IMU_DEBUG_PRINT_MS);
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
    ImuResults[0] = RawAccelToMps2((int16_t)Accel[0].FullData);
    ImuResults[1] = RawAccelToMps2((int16_t)Accel[1].FullData);
    ImuResults[2] = RawAccelToMps2((int16_t)Accel[2].FullData);
    ImuResults[3] = RawGyroToDegPerS((int16_t)Gyro[0].FullData);
    ImuResults[4] = RawGyroToDegPerS((int16_t)Gyro[1].FullData);
    ImuResults[5] = RawGyroToDegPerS((int16_t)Gyro[2].FullData);
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
    Message2Send[j+9] = 0; // Fill rest of buffer with 0's
  }
}

/** 
 * GetAngles
 * 
 * Returns roll and pitch in degrees from the Mahony filter. Only valid while
 * the IMU state machine is in IMURun; otherwise returns zero. Quaternion is
 * copied with interrupts disabled for a consistent snapshot.
 * 
 * @param roll - the roll in degrees
 * @param pitch - the pitch in degrees
 */
void GetAngles(float* roll, float* pitch)
{
    float qa;
    float qb;
    float qc;
    float qd;

    if (QueryImuSM() != IMURun) {
        *roll = 0.0f;
        *pitch = 0.0f;
        return;
    }

    __builtin_disable_interrupts();
    qa = q0;
    qb = q1;
    qc = q2;
    qd = q3;
    __builtin_enable_interrupts();

    *roll = atan2f(qa * qb + qc * qd, 0.5f - qb * qb + qc * qc) * 57.29578f;
    *pitch = asinf(2.0f * (qa * qc - qd * qb)) * 57.29578f;
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
     Configures accelerometer and gyroscope after soft reset.
****************************************************************************/
bool InitIMU(void)
{
    AccelGyroData_t data2send;
    AccelGyroData_t data2send2;
    
    /* Post-reset: full 16-bit read at CHIP_ID (addr + SPI dummy + LSB + MSB) */
    (void)ReadIMU16(IMU_REG_CHIP_ID);

    uint16_t chip_id_word = ReadIMU16(IMU_REG_CHIP_ID);
    uint8_t chip_id = (uint8_t)(chip_id_word & 0xFFu);
    if (chip_id != BMI323_CHIP_ID_VAL) {
        DB_printf("Incorrect Chip ID: %d (word 0x%04X), Expecting: %d\r\n",
                  chip_id, chip_id_word, BMI323_CHIP_ID_VAL);
        return false;
    }
    DB_printf("IMU Chip Verified: %d\r\n", chip_id);

    uint16_t data = ReadIMU16(IMU_REG_STATUS);
    DB_printf("IMU Status: %d\r\n", data);
                  
    // Setup the accelerometer/gyro settings for the IMU, do a burst write since 
    // addresses are consecutive:
    // Accel
    data2send.DataStruct.LowerByte = 0b00011001; // cutoff = acc_odr/2, acc_range = +/- ACCEL_RANGE_G g, Sample Rate = 200 Hz
    data2send.DataStruct.UpperByte = 0b01000010; // Normal mode, averaging of 4 samples

    // Gyro
    data2send2.DataStruct.LowerByte = 0b00011001; // cutoff = gyr_odr/2, gyr_range = +/- GYRO_RANGE_DPS deg/s, Sample Rate = 200 Hz
    data2send2.DataStruct.UpperByte = 0b01000010; // Normal mode, averaging of 4 samples
    WriteIMU2Transfer(IMU_REG_ACC_CONF, data2send, data2send2);

    return true;
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

    /* Dummy read selects SPI before first command after power-on */
    (void)ReadIMU16(IMU_REG_CHIP_ID);

    data2send.FullData = BMI323_CMD_SOFT_RESET;
    WriteIMU2(IMU_REG_CMD, data2send);
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
    *pSPIBUF = IMU_SPI_READ_FLAG | Address;
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
    *pSPIBUF = IMU_SPI_READ_FLAG | Address;
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

/**
 * ImuSpiFlushRx
 *
 * Discards any bytes left in the PIC32 SPI RX buffer and clears SPIROV.
 * Resets ImuSpiRxCount so the next burst starts on byte 0 of a new frame.
 * Call before starting a new transaction and after overflow / misalignment.
 */
static void ImuSpiFlushRx(void)
{
    while (!pSPISTAT->SPIRBE) {
        (void)*pSPIBUF;
    }
    pSPISTAT->SPIROV = 0;
    ImuSpiRxCount = 0;
}

/**
 * ImuSpiPushTxByte
 *
 * Writes one byte to the SPI TX buffer, blocking until SPITBF is clear.
 * Required with ENHBUF enabled so bytes are not dropped when the TX FIFO fills.
 */
static void ImuSpiPushTxByte(uint8_t byte)
{
    while (pSPISTAT->SPITBF) {
        ;
    }
    *pSPIBUF = byte;
}

/**
 * ImuSpiParseBurstFrame
 *
 * Copies the latest IMU_BURST_RX_BYTES RX buffer into Accel[] and Gyro[].
 * Skips the first two bytes (command echo and SPI dummy); bytes 2..13 are
 * six little-endian int16 values: accel X/Y/Z then gyro X/Y/Z.
 */
static void ImuSpiParseBurstFrame(void)
{
    Accel[0].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 0];
    Accel[0].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 1];
    Accel[1].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 2];
    Accel[1].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 3];
    Accel[2].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 4];
    Accel[2].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 5];
    Gyro[0].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 6];
    Gyro[0].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 7];
    Gyro[1].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 8];
    Gyro[1].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 9];
    Gyro[2].DataStruct.LowerByte = rx_data[IMU_BURST_DATA_OFFSET + 10];
    Gyro[2].DataStruct.UpperByte = rx_data[IMU_BURST_DATA_OFFSET + 11];
}

/**
 * ImuStopSampling
 *
 * Halts periodic IMU SPI traffic during recovery or fault handling: stops T6,
 * disables SPI RX interrupts, waits for the current transfer to finish, flushes
 * the RX FIFO, and clears SpiTransferBusy and the transfer-timeout counter.
 */
static void ImuStopSampling(void)
{
    T6CONbits.ON = 0;
    if (PCB_REV == 1) {
        IEC5CLR = _IEC5_SPI4RXIE_MASK;
    } else if (PCB_REV >= 2) {
        IEC3CLR = _IEC3_SPI1RXIE_MASK;
    }
    while (pSPISTAT->SPIBUSY) {
        ;
    }
    ImuSpiFlushRx();
    SpiTransferBusy = false;
    ImuTransferStaleTicks = 0;
    ImuAllowRecovery = 0;
}

/**
 * ImuBusRecover
 *
 * Resynchronizes the PIC32 SPI peripheral after overflow or framing errors:
 * drains RX, toggles the module off/on, and clears SPIROV. Call with sampling
 * already stopped (ImuStopSampling).
 */
static void ImuBusRecover(void)
{
    while (pSPISTAT->SPIBUSY) {
        ;
    }
    ImuSpiFlushRx();
    pSPICON->ON = 0;
    pSPISTAT->SPIROV = 0;
    pSPICON->ON = 1;
}

/**
 * ImuMahonyReset
 *
 * Reinitializes the Mahony AHRS quaternion and integral error state after
 * recovery so attitude does not integrate through invalid samples.
 */
static void ImuMahonyReset(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

/**
 * ImuRequestRecovery
 *
 * Posts EV_IMU_RECOVERY to this state machine (safe to call from ISR). Only
 * acts while ImuAllowRecovery is set (IMURun) and coalesces duplicate requests
 * via ImuRecoveryPosted so RunImuSM performs blocking re-init, not the ISR.
 */
static void ImuRequestRecovery(void)
{
    ES_Event_t ev;

    if (!ImuAllowRecovery || ImuRecoveryPosted) {
        return;
    }
    ImuRecoveryPosted = true;
    ev.EventType = EV_IMU_RECOVERY;
    ev.EventParam = 0;
    (void)PostImuSM(ev);
}

/**
 * ImuEnterRecovery
 *
 * Full runtime recovery entry: stop sampling, reset the filter, bus recover,
 * soft-reset the BMI323, then enter IMURecovering and wait for InitIMU on
 * ES_TIMEOUT (same post-reset path as boot).
 */
static void ImuEnterRecovery(void)
{
    if (CurrentState != IMURun) {
        return;
    }
    DB_printf("IMU recovery started\r\n");
    ImuStopSampling();
    ImuMahonyReset();
    ImuRecoveryAttempts = 0;
    CurrentState = IMURecovering;
    ImuBusRecover();
    ResetIMU();
    ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
}

/**
 * ImuPostLed
 *
 * Posts EV_LED_ON or EV_LED_OFF to LEDService; EventParam is the LED number.
 */
static void ImuPostLed(ES_EventType_t event_type, uint8_t led)
{
    ES_Event_t ev;

    ev.EventType = event_type;
    ev.EventParam = led;
    (void)PostLEDService(ev);
}

/**
 * ImuEnterInitFailed
 *
 * Terminal fault state after boot init or mid-run recovery is exhausted.
 * Stops sampling, turns off IMU_RUN_LED, and turns on IMU_FAULT_LED.
 */
static void ImuEnterInitFailed(void)
{
    ImuStopSampling();
    CurrentState = IMUInitFailed;
    ImuPostLed(EV_LED_ON, IMU_FAULT_LED);
}

/**
 * ImuSpiStartSampleBurst
 *
 * Starts a 14-byte SPI read from DATA_0 (0x03): read address, dummy byte,
 * then 12 clocks to burst accel and gyro data. RX ISR assembles the response
 * and clears SpiTransferBusy when IMU_BURST_RX_BYTES have been received.
 * Invoked from the T6 timer ISR at ~100 Hz during IMURun.
 */
static void ImuSpiStartSampleBurst(void)
{
    uint8_t i;

    ImuSpiFlushRx();
    SpiTransferBusy = true;
    ImuTransferStaleTicks = 0;
    ImuSpiRxCount = 0;

    ImuSpiPushTxByte(IMU_SPI_READ_FLAG | IMU_REG_DATA_0);
    ImuSpiPushTxByte(0x00u);
    for (i = 0; i < IMU_BURST_DATA_BYTES; i++) {
        ImuSpiPushTxByte(0x00u);
    }
}

static float RawAccelToMps2(int16_t raw)
{
    return (float)raw * GRAVITY_MPS2 / ACCEL_LSB_PER_G;
}

static float RawGyroToDegPerS(int16_t raw)
{
    return (float)raw / GYRO_LSB_PER_DPS;
}

void PrintImuData(void)
{
    DB_printf("Accel x: %d m/s^2\r\n",
              (int16_t)RawAccelToMps2((int16_t)Accel[0].FullData));
    DB_printf("Accel y: %d m/s^2\r\n",
              (int16_t)RawAccelToMps2((int16_t)Accel[1].FullData));
    DB_printf("Accel z: %d m/s^2\r\n",
              (int16_t)RawAccelToMps2((int16_t)Accel[2].FullData));
    DB_printf("Vel x: %d deg/sec\r\n",
              (int16_t)RawGyroToDegPerS((int16_t)Gyro[0].FullData));
    DB_printf("Vel y: %d deg/sec\r\n",
              (int16_t)RawGyroToDegPerS((int16_t)Gyro[1].FullData));
    DB_printf("Vel z: %d deg/sec\r\n\r\n",
              (int16_t)RawGyroToDegPerS((int16_t)Gyro[2].FullData));
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

void __ISR(IMU_SPI_RX_ISR_VECTOR, IPL5SRS) ImuSpiRxIsr(void)
{
    static float imu_data[6];

    if (pSPISTAT->SPIROV) {
        ImuSpiFlushRx();
        SpiTransferBusy = false;
        ImuRequestRecovery();
        ImuSpiClearRxIf();
        return;
    }

    while (pSPISTAT->SPIRBE == 0) {
        rx_data[ImuSpiRxCount] = (uint8_t)*pSPIBUF;
        ImuSpiRxCount += 1;
        if (ImuSpiRxCount >= IMU_BURST_RX_BYTES) {
            ImuSpiRxCount = 0;
            ImuSpiParseBurstFrame();
            GetIMUData(imu_data);
            MahonyUpdate(imu_data[0], imu_data[1], imu_data[2], imu_data[3],
                         imu_data[4], imu_data[5], DT);
            SpiTransferBusy = false;
        }
    }
    ImuSpiClearRxIf();
}

void __ISR(IMU_SPI_TX_ISR_VECTOR, IPL5SRS) ImuSpiTxIsr(void)
{
    ImuSpiClearTxIf();
}

void __ISR(_TIMER_6_VECTOR, IPL5SRS) T6Handler(void)
{
    IFS0CLR = _IFS0_T6IF_MASK;

    if (pSPISTAT->SPIROV) {
        ImuSpiFlushRx();
        SpiTransferBusy = false;
        ImuRequestRecovery();
        return;
    }

    if (SpiTransferBusy || pSPISTAT->SPIBUSY) {
        if (SpiTransferBusy) {
            ImuTransferStaleTicks += 1;
            if (ImuTransferStaleTicks >= IMU_TRANSFER_STALE_TICKS) {
                ImuTransferStaleTicks = 0;
                SpiTransferBusy = false;
                ImuSpiFlushRx();
                ImuRequestRecovery();
            }
        }
        return;
    }

    ImuTransferStaleTicks = 0;
    ImuSpiStartSampleBurst();
}