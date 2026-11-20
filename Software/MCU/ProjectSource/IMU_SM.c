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

/* STATUS register bits (datasheet 6.1, all clear-on-read) */
#define IMU_STATUS_POR_DETECTED 0x0001u /* chip was power-cycled or reset */
#define IMU_STATUS_DRDY_GYR     0x0040u
#define IMU_STATUS_DRDY_ACC     0x0080u

/* Value reported by the BMI323 for an axis whose data is invalid
 * (sensor disabled, no sample yet, or just after reset). */
#define IMU_INVALID_SAMPLE      ((int16_t)0x8000)

/* Burst read starting at STATUS (0x02):
 *   [0] cmd echo, [1] SPI dummy, [2..3] STATUS, [4..15] 6 x int16
 *   (accel X/Y/Z then gyro X/Y/Z, little-endian). */
#define IMU_BURST_START_REG     IMU_REG_STATUS
#define IMU_BURST_RX_BYTES      16u
#define IMU_BURST_STATUS_OFFSET 2u      /* STATUS LSB index in rx_data[] */
#define IMU_BURST_DATA_OFFSET   4u      /* First data byte index in rx_data[] */

/* Fault handling: a single bad frame is dropped; this many consecutive bad
 * frames (~100 ms at 100 Hz) triggers a full re-init. */
#define IMU_BAD_FRAME_LIMIT     10u

/* EventParam of EV_IMU_RECOVERY: why recovery was requested */
#define IMU_FAULT_BAD_FRAMES    1u
#define IMU_FAULT_POR           2u

/* Retry delay after init/recovery gives up; doubles each time up to the cap.
 * The state machine never stops retrying. */
#define IMU_BACKOFF_MIN_MS      1000u
#define IMU_BACKOFF_MAX_MS      8000u

/* IMU_TEST_HOOKS (IMU_SM.h) enables fault injection for bench testing. */
#if IMU_TEST_HOOKS
static volatile uint8_t ImuInjectBadFrames = 0;   /* corrupt the next N frames */
static volatile bool ImuInjectSoftReset = false;  /* send soft reset on next T6 tick */
static void ImuSpiSendSoftResetFromIsr(void);
#endif

/* PIC32MZ enhanced-buffer FIFO is 16 bytes deep in 8-bit mode; the whole
 * burst is loaded in one shot in ImuSpiStartSampleBurst, so it must fit. */
#define IMU_SPI_FIFO_DEPTH      16u
#if (IMU_BURST_RX_BYTES > IMU_SPI_FIFO_DEPTH)
#error "IMU burst no longer fits in the SPI FIFO; ImuSpiStartSampleBurst must be reworked"
#endif

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
#define IMU_YAW_GYRO_SIGN       (1.0f)  /* flip to -1.0f if pivot spin direction is inverted */

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
static void ImuSpiStartSampleBurst(void);
static void ImuStopSampling(void);
static void ImuBusRecover(void);
static void ImuMahonyReset(void);
static void ImuRequestRecovery(uint8_t reason);
static void ImuNoteBadFrame(volatile uint16_t *counter);
static void ImuProcessBurstFrame(void);
static void ImuEnterRecovery(uint8_t reason);
static void ImuStartRetry(void);
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

/* Fault bookkeeping (written from IPL5 ISRs, read by RunImuSM) */
static volatile uint8_t ImuConsecutiveBadFrames = 0;
static volatile uint16_t ImuBadInvalidCount = 0;   /* an axis read 0x8000 */
static volatile uint16_t ImuBadNoDrdyCount = 0;    /* no new sample since last read */
static volatile uint16_t ImuBadOverflowCount = 0;  /* SPI RX overflow */
static volatile uint16_t ImuBadStaleCount = 0;     /* transfer never completed */
static volatile uint16_t ImuPorCount = 0;          /* chip reset detected */
static uint16_t ImuRetryDelayMs = IMU_BACKOFF_MIN_MS;
static bool ImuFaultLedOn = false;

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
  // SPI mode 3 (CPOL=1, CPHA=1). BMI323 drives SDO on the falling edge of SCK
  // and expects capture on the rising edge (datasheet 7.2.3).
  pSPICON->SMP = 0; // Input sampled at middle of data output time (rising edge). SMP=1 would sample at the falling edge, racing the BMI323's SDO change
  pSPICON->CKE = 0; // Serial output data changes on transition from idle clock state to active clock state (falling edge)
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
                    DB_printf("IMU init failed after %u attempts; retrying in %u ms\r\n",
                              (unsigned)IMU_INIT_MAX_ATTEMPTS,
                              (unsigned)ImuRetryDelayMs);
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
    {
      /* Fault/backoff state: wait ImuRetryDelayMs, then start a fresh
       * recovery cycle. Never terminal. */
      if (ThisEvent.EventType == ES_TIMEOUT) {
          ImuStartRetry();
      }
    }
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
            ImuConsecutiveBadFrames = 0;
            ImuAllowRecovery = 1;
            ImuRecoveryPosted = false;
            ImuTransferStaleTicks = 0;
            ImuRetryDelayMs = IMU_BACKOFF_MIN_MS;
            if (ImuFaultLedOn) {
                ImuPostLed(EV_LED_OFF, IMU_FAULT_LED);
                ImuFaultLedOn = false;
            }
        }
        break;

        default:
          ;
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
                    DB_printf("IMU recovery failed after %u attempts; retrying in %u ms\r\n",
                              (unsigned)IMU_RECOVERY_MAX_ATTEMPTS,
                              (unsigned)ImuRetryDelayMs);
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
            ImuEnterRecovery((uint8_t)ThisEvent.EventParam);
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

    float sin_pitch = 2.0f * (qa * qc - qd * qb);
    if (!isfinite(qa) || !isfinite(qb) || !isfinite(qc) || !isfinite(qd) || !isfinite(sin_pitch)) {
        *roll = 0.0f;
        *pitch = 0.0f;
        return;
    }
    sin_pitch = fmaxf(fminf(sin_pitch, 1.0f), -1.0f);

    *roll = atan2f(qa * qb + qc * qd, 0.5f - qb * qb + qc * qc) * 57.29578f;
    *pitch = asinf(sin_pitch) * 57.29578f;
    if (!isfinite(*roll)) {
        *roll = 0.0f;
    }
    if (!isfinite(*pitch)) {
        *pitch = 0.0f;
    }
}

/**
 * GetRobotYawRate
 *
 * Returns robot yaw rate in rad/s from Gyro Z. Only valid while the IMU
 * state machine is in IMURun; otherwise returns false and leaves *rad_per_s
 * unchanged. Gyro sample is copied with interrupts disabled for consistency.
 *
 * @param rad_per_s - robot yaw rate in rad/s (positive = CCW when viewed from above)
 * @return true if IMU is running and rate was written, false otherwise
 */
bool GetRobotYawRate(float *rad_per_s)
{
    int16_t raw;

    if (QueryImuSM() != IMURun) {
        return false;
    }

    __builtin_disable_interrupts();
    raw = (int16_t)Gyro[2].FullData;
    __builtin_enable_interrupts();

    *rad_per_s = IMU_YAW_GYRO_SIGN * RawGyroToDegPerS(raw) * 0.0174533f;
    return true;
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

    /* Reading STATUS is required here, not just informational: it clears the
     * clear-on-read por_detected flag set by the soft reset. Every sample
     * burst reads STATUS and treats por_detected as "chip reset, re-init",
     * so skipping this read would cause an endless recovery loop. */
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
 * ImuProcessBurstFrame
 *
 * Validates a complete IMU_BURST_RX_BYTES frame in rx_data[] and, only if it
 * is good, publishes it to Accel[]/Gyro[] and runs the Mahony update.
 * Called from the SPI RX ISR.
 *
 *  - STATUS.por_detected set: the BMI323 reset itself (e.g. brownout) and is
 *    back to defaults with sensors disabled -> request recovery immediately.
 *  - Neither drdy bit set: no new sample since the last read (sensor stalled);
 *    the data would be a repeat, so drop it.
 *  - Any axis == 0x8000: the chip's "invalid data" marker -> drop the frame.
 *
 * Dropped frames leave the last good sample in place and count toward
 * IMU_BAD_FRAME_LIMIT. Each axis is published with a single 16-bit store so
 * readers can no longer see half-updated (torn) values.
 */
static void ImuProcessBurstFrame(void)
{
    int16_t raw[6];
    uint16_t status;
    uint8_t i;

#if IMU_TEST_HOOKS
    if (ImuInjectBadFrames > 0u) {
        /* Simulate the chip's invalid-data marker on accel X */
        ImuInjectBadFrames -= 1;
        rx_data[IMU_BURST_DATA_OFFSET] = 0x00u;
        rx_data[IMU_BURST_DATA_OFFSET + 1u] = 0x80u;
    }
#endif

    status = (uint16_t)rx_data[IMU_BURST_STATUS_OFFSET] |
             ((uint16_t)rx_data[IMU_BURST_STATUS_OFFSET + 1] << 8);

    if (status & IMU_STATUS_POR_DETECTED) {
        ImuPorCount += 1;
        ImuRequestRecovery(IMU_FAULT_POR);
        return;
    }

    if ((status & (IMU_STATUS_DRDY_ACC | IMU_STATUS_DRDY_GYR)) == 0u) {
        ImuNoteBadFrame(&ImuBadNoDrdyCount);
        return;
    }

    for (i = 0; i < 6u; i++) {
        raw[i] = (int16_t)((uint16_t)rx_data[IMU_BURST_DATA_OFFSET + 2u * i] |
                           ((uint16_t)rx_data[IMU_BURST_DATA_OFFSET + 2u * i + 1u] << 8));
        if (raw[i] == IMU_INVALID_SAMPLE) {
            ImuNoteBadFrame(&ImuBadInvalidCount);
            return;
        }
    }

    /* Good frame */
    ImuConsecutiveBadFrames = 0;
    for (i = 0; i < 3u; i++) {
        Accel[i].FullData = (uint16_t)raw[i];
        Gyro[i].FullData = (uint16_t)raw[i + 3u];
    }

    MahonyUpdate(RawAccelToMps2(raw[0]), RawAccelToMps2(raw[1]),
                 RawAccelToMps2(raw[2]), RawGyroToDegPerS(raw[3]),
                 RawGyroToDegPerS(raw[4]), RawGyroToDegPerS(raw[5]), DT);
}

/**
 * ImuNoteBadFrame
 *
 * Records one dropped frame (bumps the per-cause counter and the consecutive
 * counter). Requests recovery only after IMU_BAD_FRAME_LIMIT bad frames in a
 * row, so a single glitch no longer resets the chip. ISR context.
 */
static void ImuNoteBadFrame(volatile uint16_t *counter)
{
    *counter += 1;
    if (ImuConsecutiveBadFrames < 255u) {
        ImuConsecutiveBadFrames += 1;
    }
    if (ImuConsecutiveBadFrames >= IMU_BAD_FRAME_LIMIT) {
        ImuRequestRecovery(IMU_FAULT_BAD_FRAMES);
    }
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
 * If the post fails (queue full) the flag is cleared so the next bad frame
 * retries, instead of suppressing recovery forever.
 *
 * @param reason - IMU_FAULT_* code, delivered as EventParam
 */
static void ImuRequestRecovery(uint8_t reason)
{
    ES_Event_t ev;

    if (!ImuAllowRecovery || ImuRecoveryPosted) {
        return;
    }
    ImuRecoveryPosted = true;
    ev.EventType = EV_IMU_RECOVERY;
    ev.EventParam = reason;
    if (!PostImuSM(ev)) {
        ImuRecoveryPosted = false;
    }
}

/**
 * ImuEnterRecovery
 *
 * Full runtime recovery entry: stop sampling, reset the filter, bus recover,
 * soft-reset the BMI323, then enter IMURecovering and wait for InitIMU on
 * ES_TIMEOUT (same post-reset path as boot).
 *
 * @param reason - IMU_FAULT_* code from the EV_IMU_RECOVERY event
 */
static void ImuEnterRecovery(uint8_t reason)
{
    if (CurrentState != IMURun) {
        return;
    }
    ImuStopSampling();
#if IMU_TEST_HOOKS
    ImuInjectBadFrames = 0;  /* don't carry test corruption past a recovery */
#endif
    DB_printf("IMU recovery started: %s\r\n",
              (reason == IMU_FAULT_POR) ? "chip reset detected"
                                        : "too many bad frames");
    DB_printf("  bad frames: invalid=%u nodrdy=%u ovf=%u stale=%u por=%u\r\n",
              (unsigned)ImuBadInvalidCount, (unsigned)ImuBadNoDrdyCount,
              (unsigned)ImuBadOverflowCount, (unsigned)ImuBadStaleCount,
              (unsigned)ImuPorCount);
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
 * Fault/backoff state after boot init or mid-run recovery is exhausted.
 * Stops sampling, turns on IMU_FAULT_LED, and arms a retry after
 * ImuRetryDelayMs (doubling each time, capped at IMU_BACKOFF_MAX_MS). The
 * LED is turned off and the delay reset once IMURun is reached again.
 */
static void ImuEnterInitFailed(void)
{
    ImuStopSampling();
    CurrentState = IMUInitFailed;
    if (!ImuFaultLedOn) {
        ImuPostLed(EV_LED_ON, IMU_FAULT_LED);
        ImuFaultLedOn = true;
    }
    ES_Timer_InitTimer(IMU_TIMER, ImuRetryDelayMs);
    if (ImuRetryDelayMs < IMU_BACKOFF_MAX_MS) {
        ImuRetryDelayMs = (uint16_t)(ImuRetryDelayMs * 2u);
        if (ImuRetryDelayMs > IMU_BACKOFF_MAX_MS) {
            ImuRetryDelayMs = IMU_BACKOFF_MAX_MS;
        }
    }
}

/**
 * ImuStartRetry
 *
 * Leaves the IMUInitFailed backoff state: resynchronizes the SPI peripheral,
 * soft-resets the BMI323 and enters IMURecovering with a fresh attempt
 * budget. IMURecovering then runs InitIMU after IMU_RESET_DELAY_MS.
 */
static void ImuStartRetry(void)
{
    DB_printf("IMU retrying init\r\n");
    ImuRecoveryAttempts = 0;
    CurrentState = IMURecovering;
    ImuBusRecover();
    ResetIMU();
    ES_Timer_InitTimer(IMU_TIMER, IMU_RESET_DELAY_MS);
}

/**
 * ImuSpiStartSampleBurst
 *
 * Starts a 16-byte SPI read from STATUS (0x02): read address, dummy byte,
 * then 14 clocks for STATUS + accel + gyro. RX ISR assembles the response
 * and clears SpiTransferBusy when IMU_BURST_RX_BYTES have been received.
 * Invoked from the T6 timer ISR at ~100 Hz during IMURun.
 */
static void ImuSpiStartSampleBurst(void)
{
    uint8_t i;
    unsigned int isr_state;

    ImuSpiFlushRx();
    SpiTransferBusy = true;
    ImuTransferStaleTicks = 0;
    ImuSpiRxCount = 0;

    /* Load the whole frame into the TX FIFO with interrupts masked. With
     * MSSEN=1 the hardware releases SS as soon as the TX FIFO and shift
     * register run dry; if a higher-priority ISR (motor/Jetson, IPL6/7)
     * preempted us between pushes, SS could rise mid-frame and the BMI323
     * would treat the remaining bytes as a new (write) transaction.
     * The caller has verified SPIBUSY == 0, so the FIFO is empty and the
     * full frame fits without waiting on SPITBF (no spin in the ISR). */
    isr_state = __builtin_get_isr_state();
    __builtin_disable_interrupts();
    *pSPIBUF = IMU_SPI_READ_FLAG | IMU_BURST_START_REG;
    for (i = 1; i < IMU_BURST_RX_BYTES; i++) {
        *pSPIBUF = 0x00u; /* SPI dummy byte, then clocks for STATUS + data */
    }
    __builtin_set_isr_state(isr_state);
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
    
    // Normalize accelerometer; skip accel correction when magnitude is invalid
    {
        float accel_sq = ax * ax + ay * ay + az * az;
        if (accel_sq > 1.0e-12f) {
            recipNorm = 1.0f / sqrtf(accel_sq);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimate the direction of gravity
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error (cross product of estimated and measured direction of gravity)
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
        } else {
            halfex = 0.0f;
            halfey = 0.0f;
            halfez = 0.0f;
        }
    }
    
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
    {
        float q_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        if (q_sq > 1.0e-12f) {
            recipNorm = 1.0f / sqrtf(q_sq);
            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;
        } else {
            ImuMahonyReset();
        }
    }
}

void __ISR(IMU_SPI_RX_ISR_VECTOR, IPL5SRS) ImuSpiRxIsr(void)
{
    if (pSPISTAT->SPIROV) {
        ImuSpiFlushRx();
        SpiTransferBusy = false;
        ImuNoteBadFrame(&ImuBadOverflowCount);
        ImuSpiClearRxIf();
        return;
    }

    while (pSPISTAT->SPIRBE == 0) {
        uint8_t byte = (uint8_t)*pSPIBUF;

        if (!SpiTransferBusy) {
            /* Stray byte with no burst in flight: discard it rather than
             * letting it shift the alignment of the next frame. */
            continue;
        }
        rx_data[ImuSpiRxCount] = byte;
        ImuSpiRxCount += 1;
        if (ImuSpiRxCount >= IMU_BURST_RX_BYTES) {
            ImuSpiRxCount = 0;
            SpiTransferBusy = false;
            ImuProcessBurstFrame();
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
        ImuNoteBadFrame(&ImuBadOverflowCount);
        return;
    }

    if (SpiTransferBusy || pSPISTAT->SPIBUSY) {
        if (SpiTransferBusy) {
            ImuTransferStaleTicks += 1;
            if (ImuTransferStaleTicks >= IMU_TRANSFER_STALE_TICKS) {
                ImuTransferStaleTicks = 0;
                SpiTransferBusy = false;
                ImuSpiFlushRx();
                ImuNoteBadFrame(&ImuBadStaleCount);
            }
        }
        return;
    }

    ImuTransferStaleTicks = 0;
#if IMU_TEST_HOOKS
    if (ImuInjectSoftReset) {
        ImuInjectSoftReset = false;
        ImuSpiSendSoftResetFromIsr();
        return;
    }
#endif
    ImuSpiStartSampleBurst();
}

/****************************************************************************
 Diagnostics / test hooks (called from main-line code, e.g. UsbService)
 ****************************************************************************/

/**
 * ImuPrintDiagnostics
 *
 * Prints state, fault counters and the latest published sample. Counters
 * accumulate since boot (they are not cleared by recovery).
 */
void ImuPrintDiagnostics(void)
{
    static const char * const StateNames[] = {
        "InitPState", "Reset", "Wait", "Run", "Recovering", "InitFailed(backoff)"
    };
    float roll;
    float pitch;
    ImuState_t state = CurrentState;

    DB_printf("IMU state: %s\r\n",
              (state <= IMUInitFailed) ? StateNames[state] : "?");
    DB_printf("  consecutive bad: %u, next retry delay: %u ms, fault LED: %u\r\n",
              (unsigned)ImuConsecutiveBadFrames, (unsigned)ImuRetryDelayMs,
              (unsigned)ImuFaultLedOn);
    DB_printf("  bad frames: invalid=%u nodrdy=%u ovf=%u stale=%u por=%u\r\n",
              (unsigned)ImuBadInvalidCount, (unsigned)ImuBadNoDrdyCount,
              (unsigned)ImuBadOverflowCount, (unsigned)ImuBadStaleCount,
              (unsigned)ImuPorCount);
    /* accel in cm/s^2, gyro in 0.1 deg/s (DB_printf has no float support) */
    DB_printf("  accel cm/s^2: %d %d %d\r\n",
              (int)(100.0f * RawAccelToMps2((int16_t)Accel[0].FullData)),
              (int)(100.0f * RawAccelToMps2((int16_t)Accel[1].FullData)),
              (int)(100.0f * RawAccelToMps2((int16_t)Accel[2].FullData)));
    DB_printf("  gyro 0.1dps:  %d %d %d\r\n",
              (int)(10.0f * RawGyroToDegPerS((int16_t)Gyro[0].FullData)),
              (int)(10.0f * RawGyroToDegPerS((int16_t)Gyro[1].FullData)),
              (int)(10.0f * RawGyroToDegPerS((int16_t)Gyro[2].FullData)));
    GetAngles(&roll, &pitch);
    DB_printf("  roll, pitch deg: %d, %d\r\n", (int)roll, (int)pitch);
}

#if IMU_TEST_HOOKS
/**
 * ImuTestInjectBadFrames
 *
 * Marks the next n received frames as invalid (accel X = 0x8000), which
 * exercises the real validation path. n < IMU_BAD_FRAME_LIMIT should be
 * absorbed silently; n >= IMU_BAD_FRAME_LIMIT should trigger one recovery.
 */
void ImuTestInjectBadFrames(uint8_t n)
{
    ImuInjectBadFrames = n;
    DB_printf("IMU test: corrupting next %u frames (limit %u)\r\n",
              (unsigned)n, (unsigned)IMU_BAD_FRAME_LIMIT);
}

/**
 * ImuTestInjectSoftReset
 *
 * On the next T6 tick, sends a soft-reset command to the BMI323 instead of a
 * sample burst. This simulates the chip resetting by itself (e.g. brownout)
 * and should be detected via STATUS.por_detected or as bad frames, then
 * recovered.
 */
void ImuTestInjectSoftReset(void)
{
    if (CurrentState != IMURun) {
        DB_printf("IMU test: not in Run state, ignoring\r\n");
        return;
    }
    ImuInjectSoftReset = true;
    DB_printf("IMU test: soft-resetting BMI323 behind the driver's back\r\n");
}

/**
 * ImuSpiSendSoftResetFromIsr
 *
 * T6 ISR helper: writes CMD (0x7E) = 0xDEAF as a 3-byte frame. SpiTransferBusy
 * stays false, so the RX ISR discards the echoed bytes.
 */
static void ImuSpiSendSoftResetFromIsr(void)
{
    unsigned int isr_state;

    ImuSpiFlushRx();
    isr_state = __builtin_get_isr_state();
    __builtin_disable_interrupts();
    *pSPIBUF = IMU_REG_CMD;                              /* write, address 0x7E */
    *pSPIBUF = (uint8_t)(BMI323_CMD_SOFT_RESET & 0xFFu); /* LSB first */
    *pSPIBUF = (uint8_t)(BMI323_CMD_SOFT_RESET >> 8);
    __builtin_set_isr_state(isr_state);
}
#endif