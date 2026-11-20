/****************************************************************************
 Module
   IMU_SM.c

 Description
   Event-driven, non-blocking driver for the Bosch BMI323 6-axis IMU.

 Notes
   Architecture
   ------------
   * One SPI transaction engine (ImuXferStart / RX ISR) is used for every
     transfer. Each transaction is at most 16 bytes, so the whole frame is
     loaded into the PIC32 enhanced-buffer FIFO in one masked burst; with
     MSSEN=1 this keeps SS asserted for the entire frame. No code ever
     busy-waits on the SPI peripheral.
   * Initialization is a chain of steps driven by events: each step starts
     one transfer, the RX ISR posts EV_IMU_XFER_DONE (tagged with a transfer
     id) when it completes, and an ES timer acts as the per-step timeout.
   * In IMURun, T6 (~100 Hz) starts a 16-byte burst from STATUS. The RX ISR
     validates the frame (por_detected, drdy, 0x8000 invalid marker),
     publishes good samples to a double buffer with a sequence number and a
     core-timer timestamp, and posts EV_IMU_DATA_UPDATE (coalesced). The
     Mahony filter runs in RunImuSM, not in the ISR.
   * Readers (any context / priority) get a consistent snapshot of the latest
     sample or attitude, plus validity and freshness.
   * Faults are tiered: a bad frame is dropped; IMU_BAD_FRAME_LIMIT bad frames
     in a row or a detected chip reset re-runs init; repeated init failure
     enters a backoff state that retries forever.

 History
 When           Who     What/Why
 -------------- ---     --------

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "IMU_SM.h"
#include "LEDService.h"
#include <sys/attribs.h>
#include "dbprintf.h"
#include <math.h>

/*----------------------------- Module Defines ----------------------------*/
/* BMI323 registers / commands (BST-BMI323 datasheet) */
#define IMU_SPI_READ_FLAG       0x80u
#define IMU_REG_CHIP_ID         0x00u
#define IMU_REG_ERR             0x01u
#define IMU_REG_STATUS          0x02u
#define IMU_REG_ACC_CONF        0x20u
#define IMU_REG_CMD             0x7Eu
#define BMI323_CHIP_ID_VAL      0x43u
#define BMI323_CMD_SOFT_RESET   0xDEAFu

/* ERR_REG bits */
#define IMU_ERR_FATAL           0x0001u
#define IMU_ERR_ACC_CONF        0x0020u
#define IMU_ERR_GYR_CONF        0x0040u

/* STATUS register bits (all clear-on-read) */
#define IMU_STATUS_POR_DETECTED 0x0001u /* chip was power-cycled or reset */
#define IMU_STATUS_DRDY_GYR     0x0040u
#define IMU_STATUS_DRDY_ACC     0x0080u

/* ACC_CONF / GYR_CONF value: ODR 200 Hz (0x9), range +/-4 g resp. +/-250 dps
 * (0b001), bandwidth ODR/2, average 4 samples (0b010), normal mode (0b100).
 * Must match ACCEL_LSB_PER_G / GYRO_LSB_PER_DPS below. */
#define IMU_ACC_CONF_VAL        0x4219u
#define IMU_GYR_CONF_VAL        0x4219u
#define IMU_CONF_MASK           0x77FFu /* defined fields only (bits 11, 15 reserved) */

/* Value reported by the BMI323 for an axis whose data is invalid
 * (sensor disabled, no sample yet, or just after reset). */
#define IMU_INVALID_SAMPLE      ((int16_t)0x8000)

/* SPI framing: every read returns [cmd echo][dummy][data...] */
#define IMU_SPI_FIFO_DEPTH      16u     /* PIC32MZ ENHBUF depth in 8-bit mode */
#define IMU_RX_DATA_OFFSET      2u      /* first payload byte of a read */

/* Sample burst from STATUS (0x02):
 *   [0] echo, [1] dummy, [2..3] STATUS, [4..15] acc X/Y/Z, gyr X/Y/Z (LE) */
#define IMU_SAMPLE_BYTES        16u
#define IMU_SAMPLE_STATUS_OFS   2u
#define IMU_SAMPLE_DATA_OFS     4u
#if (IMU_SAMPLE_BYTES > IMU_SPI_FIFO_DEPTH)
#error "IMU sample burst must fit in the SPI FIFO"
#endif

/* Fault handling */
#define IMU_BAD_FRAME_LIMIT     10u     /* consecutive bad frames -> re-init */
#define IMU_FAULT_BAD_FRAMES    1u      /* EV_IMU_RECOVERY params */
#define IMU_FAULT_POR           2u
#define IMU_TRANSFER_STALE_TICKS 3u     /* T6 ticks (~30 ms) for a stuck sample */

/* Init timing (ms). The reset / power-up waits are deliberately generous;
 * a too-short wait just fails a step and is retried. */
#define IMU_POWERUP_DELAY_MS    100u    /* before first access after boot */
#define IMU_RESET_DELAY_MS      100u    /* after soft reset */
#define IMU_SETTLE_MS           100u    /* sensors on -> data ready (gyro ~30 ms) */
#define IMU_XFER_TIMEOUT_MS     20u     /* a <=16 byte transfer takes ~100 us */
#define IMU_RESTART_DELAY_MS    10u     /* before a re-init from Run/backoff */
#define IMU_ATTEMPT_DELAY_MS    50u     /* between attempts within one cycle */
#define IMU_INIT_MAX_ATTEMPTS   3u      /* attempts before backoff */
#define IMU_BACKOFF_MIN_MS      1000u   /* backoff doubles up to the cap... */
#define IMU_BACKOFF_MAX_MS      8000u   /* ...and never gives up */

/* LEDService EventParam for EV_LED_ON / EV_LED_OFF (see LEDService.c) */
#define IMU_FAULT_LED           2u      /* LATH4: IMU not running */

/* Scaling (must match IMU_ACC_CONF_VAL / IMU_GYR_CONF_VAL) */
#define ACCEL_LSB_PER_G         8192.0f /* LSB/g at +/-4 g */
#define GYRO_LSB_PER_DPS        131.2f  /* LSB/(deg/s) at +/-250 dps */
#define GRAVITY_MPS2            9.80665f
#define IMU_YAW_GYRO_SIGN       (1.0f)  /* flip to -1.0f if pivot spin direction is inverted */
#define IMU_DEG_TO_RAD          0.0174533f
#define IMU_RAD_TO_DEG          57.29578f

/* Timestamps come from the CP0 core timer (SYSCLK/2 = 100 MHz; see
 * ES_Timer_RATE_1mS = 100000 counts), independent of the T6 period. */
#define IMU_CORE_TIMER_HZ       100000000.0f
#define IMU_COUNTS_PER_MS       100000u
#define IMU_MAX_AGE_COUNTS      10000000u /* 100 ms: older data is "stale" */

/* BMI323 needs >= 2 us of bus idle after every access (datasheet 7.2.3 /
 * Fig. 35). Enforced in ImuXferStart with margin. */
#define IMU_SPI_IDLE_COUNTS     500u    /* 5 us */

/* Mahony filter */
#define TWO_KP                  (2.0f * 5.0f)
#define TWO_KI                  0.0f
#define DT                      0.00999936f /* nominal sample period (T6) */
#define IMU_MAX_FILTER_DT_S     0.05f   /* larger measured dt -> use nominal DT */

/*------------------------------ Module Types -----------------------------*/
typedef enum
{
    IMU_XFER_IDLE = 0,
    IMU_XFER_SAMPLE,    /* periodic burst: validated + published in the ISR */
    IMU_XFER_INIT,      /* init step: completion posts EV_IMU_XFER_DONE */
    IMU_XFER_DISCARD    /* fire-and-forget (test hook) */
} ImuXferKind_t;

typedef enum
{
    IMU_STEP_START_WAIT = 0, /* timer: let the chip power up / bus go idle */
    IMU_STEP_DUMMY_READ,     /* xfer: one read switches the interface to SPI */
    IMU_STEP_SOFT_RESET,     /* xfer: CMD = softreset */
    IMU_STEP_RESET_WAIT,     /* timer */
    IMU_STEP_WAKE_READ,      /* xfer: dummy read again (reset reverts to I2C) */
    IMU_STEP_READ_ID,        /* xfer: CHIP_ID, ERR_REG, STATUS (clears por) */
    IMU_STEP_WRITE_CONF,     /* xfer: ACC_CONF + GYR_CONF */
    IMU_STEP_VERIFY_CONF,    /* xfer: read back ACC_CONF + GYR_CONF */
    IMU_STEP_SETTLE_WAIT,    /* timer: sensors start up */
    IMU_STEP_CHECK_READY,    /* xfer: ERR_REG + STATUS, expect drdy, no errors */
    IMU_STEP_COUNT
} ImuInitStep_t;

/* Published attitude (internal; readers use GetAngles) */
typedef struct
{
    uint32_t seq;   /* 0 = invalid / being written */
    uint32_t t;     /* core timer count of the sample it was computed from */
    float roll;     /* deg */
    float pitch;    /* deg */
} ImuAttitude_t;

/*---------------------------- Module Functions ---------------------------*/
/* SPI transaction engine */
static void ImuSpiFlushRx(void);
static bool ImuXferStart(ImuXferKind_t kind, const uint8_t *tx, uint8_t len);
static void ImuXferAbort(void);

/* Init sequence / state handling */
static void ImuStartInit(uint16_t delay_ms);
static void ImuInitBeginAttempt(uint16_t delay_ms);
static void ImuInitEnterStep(ImuInitStep_t step);
static bool ImuInitXfer(const uint8_t *tx, uint8_t len);
static void ImuInitHandleEvent(ES_Event_t ev);
static void ImuInitFail(const char *why);
static void ImuEnterRun(void);
static void ImuEnterBackoff(void);
static void ImuStopSampling(void);
static void ImuSetFaultLed(bool on);
static void ImuArmTimer(uint16_t ms);
static bool ImuIsOwnTimeout(ES_Event_t ev);

/* Sample path (ISR) */
static void ImuProcessSampleFrame(void);
static void ImuNoteBadFrame(volatile uint16_t *counter);
static void ImuRequestRecovery(uint8_t reason);
static bool ImuPostFromIsr(ES_EventType_t type, uint16_t param);
static void ImuPublishSample(const int16_t *raw, uint32_t t);

/* Readers / filter (task + any context) */
static bool ImuReadSampleRaw(ImuSample_t *out);
static bool ImuReadAttitude(ImuAttitude_t *out);
static bool ImuIsFresh(uint32_t t);
static void ImuInvalidatePublished(void);
static void ImuFilterLatest(void);
static void ImuPublishAttitude(uint32_t t);
static void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt);
static void ImuMahonyReset(void);
static float RawAccelToMps2(int16_t raw);
static float RawGyroToDegPerS(int16_t raw);
static uint16_t ImuRxWord(uint8_t ofs);

#if IMU_TEST_HOOKS
static volatile uint8_t ImuInjectBadFrames = 0;   /* corrupt the next N frames */
static volatile bool ImuInjectSoftReset = false;  /* send soft reset on next T6 tick */
#endif

/*---------------------------- Module Variables ---------------------------*/
static ImuState_t CurrentState;
static uint8_t MyPriority;

/* --- SPI peripheral --- */
#if PCB_REV == 1
static volatile __SPI4CONbits_t * pSPICON;
static volatile __SPI4CON2bits_t * pSPICON2;
static volatile __SPI4STATbits_t * pSPISTAT;
#define IMU_SPI_RX_ISR_VECTOR  _SPI4_RX_VECTOR
#define IMU_SPI_TX_ISR_VECTOR  _SPI4_TX_VECTOR
#define ImuSpiClearRxIf()      (IFS5CLR = _IFS5_SPI4RXIF_MASK)
#define ImuSpiClearTxIf()      do { IEC5CLR = _IEC5_SPI4TXIE_MASK; IFS5CLR = _IFS5_SPI4TXIF_MASK; } while (0)
#define ImuSpiRxIntEnable()    (IEC5SET = _IEC5_SPI4RXIE_MASK)
#elif PCB_REV >= 2
static volatile __SPI1CONbits_t * pSPICON;
static volatile __SPI1CON2bits_t * pSPICON2;
static volatile __SPI1STATbits_t * pSPISTAT;
#define IMU_SPI_RX_ISR_VECTOR  _SPI1_RX_VECTOR
#define IMU_SPI_TX_ISR_VECTOR  _SPI1_TX_VECTOR
#define ImuSpiClearRxIf()      (IFS3CLR = _IFS3_SPI1RXIF_MASK)
#define ImuSpiClearTxIf()      do { IEC3CLR = _IEC3_SPI1TXIE_MASK; IFS3CLR = _IFS3_SPI1TXIF_MASK; } while (0)
#define ImuSpiRxIntEnable()    (IEC3SET = _IEC3_SPI1RXIE_MASK)
#endif
static volatile uint32_t * pSPIBRG;
static volatile uint32_t * pSPIBUF;

/* --- Transaction engine (shared by task-level init and IPL5 ISRs) --- */
static volatile ImuXferKind_t XferKind = IMU_XFER_IDLE;
static volatile uint8_t XferLen = 0;
static volatile uint8_t XferCount = 0;
static volatile uint8_t XferIdInFlight = 0;      /* id posted with XFER_DONE */
static volatile uint8_t RxBuf[IMU_SPI_FIFO_DEPTH];
static uint8_t XferId = 0;                       /* id of the current init xfer */
static volatile uint32_t ImuXferEndT = 0;        /* core timer when SS was last released */

/* IMU_TIMER bookkeeping, used to reject a stale ES_TIMEOUT that was already
 * queued when the timer was re-armed (ES timers cannot be tagged). */
static uint32_t ImuTimerStartT = 0;
static uint32_t ImuTimerHalfCounts = 0;
static const uint8_t ImuSampleTx[IMU_SAMPLE_BYTES] = {
    IMU_SPI_READ_FLAG | IMU_REG_STATUS           /* remaining bytes are 0x00 */
};

/* --- Init sequence --- */
static ImuInitStep_t InitStep = IMU_STEP_START_WAIT;
static uint8_t ImuInitAttempts = 0;
static uint16_t ImuRetryDelayMs = IMU_BACKOFF_MIN_MS;
static bool ImuFaultLedOn = false;
static const char * const ImuStepNames[IMU_STEP_COUNT] = {
    "start-wait", "dummy-read", "soft-reset", "reset-wait", "wake-read",
    "read-id", "write-conf", "verify-conf", "settle-wait", "check-ready"
};

/* --- Run-time fault bookkeeping (written by IPL5 ISRs) --- */
static volatile uint8_t ImuAllowRecovery = 0;
static volatile bool ImuRecoveryPosted = false;
static volatile bool ImuDataPending = false;
static uint8_t ImuTransferStaleTicks = 0;
static volatile uint8_t ImuConsecutiveBadFrames = 0;
static volatile uint16_t ImuBadInvalidCount = 0;   /* an axis read 0x8000 */
static volatile uint16_t ImuBadNoDrdyCount = 0;    /* no new sample since last read */
static volatile uint16_t ImuBadOverflowCount = 0;  /* SPI RX overflow */
static volatile uint16_t ImuBadStaleCount = 0;     /* transfer never completed */
static volatile uint16_t ImuPorCount = 0;          /* chip reset detected */
static uint16_t ImuInitFailCount = 0;              /* failed init attempts */

/* --- Published sample: double buffer. The writer (RX ISR) fills the back
 * buffer (seq = 0 while writing) and then flips SampleFront with a single
 * store. Readers copy the front buffer and accept the copy only if its seq
 * is non-zero and unchanged afterwards. --- */
static volatile ImuSample_t SampleBuf[2];
static volatile uint8_t SampleFront = 0;
static uint32_t SampleSeq = 0;                     /* ISR-only */

/* Diagnostics: measured period between good samples (ISR-only writer) */
static uint32_t ImuLastGoodT = 0;
static bool ImuHaveLastGood = false;
static volatile int32_t ImuSamplePeriodAvg = 0;    /* core-timer counts */

/* --- Published attitude: same scheme, written by RunImuSM --- */
static volatile ImuAttitude_t AttBuf[2];
static volatile uint8_t AttFront = 0;
static uint32_t AttSeq = 0;

/* --- Mahony filter state (task context only) --- */
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;
static uint32_t ImuLastFilteredSeq = 0;
static uint32_t ImuLastFilteredT = 0;
static bool ImuHaveFilterTime = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitImuSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Configures the SPI peripheral, T6 and interrupts, then posts ES_INIT.
     No IMU traffic happens here; the init sequence runs from RunImuSM.
****************************************************************************/
bool InitImuSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

#if PCB_REV == 1
    // While the SPI module is off (e.g. the off/on in ImuXferAbort) the pins
    // fall back to their LAT value: idle SS and SCK high so they don't glitch
    LATDSET = _LATD_LATD9_MASK | _LATD_LATD10_MASK;
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
    // Set interrupt pins to inputs
    TRISDSET = _TRISD_TRISD12_MASK | _TRISD_TRISD13_MASK;

    INT2R = 0b1010; // Map RD12 -> External interrupt 2

    // While the SPI module is off (e.g. the off/on in ImuXferAbort) the pins
    // fall back to their LAT value: idle SS (RD4) and SCK (RD1) high
    LATDSET = _LATD_LATD1_MASK | _LATD_LATD4_MASK;

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
  pSPICON->MSSEN = 1; // SS is automatically driven (held low while the FIFO has data)
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
  pSPICON->STXISEL = 0b00; // TX interrupt (unused)
  pSPICON->SRXISEL = 0b01; // RX interrupt while the receive buffer is not empty

  pSPICON2->AUDEN = 0; // Audio protocol is disabled

  ImuSpiFlushRx();

  *pSPIBRG = 15; // 1.56 MHz clock frequency, IMU Has max frequency of 10 MHz

  // Setup Timer 6: sample trigger, only running in IMURun
  T6CON = 0;
  T6CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T6CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR6 = 1953; // ~100 Hz
  TMR6 = 0; // Set TMR6 to 0

  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  PRISSbits.PRI5SS = 0b0101; // IPL5SRS handlers use shadow set 5 (below motor IPL6/7)

  // IMU SPI + sample timer at IPL5 - below motor control (7) and odometry T7 (6)
  if (PCB_REV == 1) {
    IPC41bits.SPI4TXIP = 5;
    IPC41bits.SPI4RXIP = 5;
  } else if (PCB_REV >= 2) {
    IPC27bits.SPI1TXIP = 5;
    IPC27bits.SPI1RXIP = 5;
  }
  IPC7bits.T6IP = 5;

  // Disable the RX/TX interrupt while configuring
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

  // T6 interrupt is always enabled; T6 itself only runs in IMURun
  IEC0SET = _IEC0_T6IE_MASK;
  // RX interrupt is always enabled: it completes every transaction
  ImuSpiRxIntEnable();

  __builtin_enable_interrupts(); // Global enable interrupts

  pSPICON->ON = 1; // Finally turn the SPI module on

  MyPriority = Priority;
  CurrentState = InitPState_IMU;
  ThisEvent.EventType = ES_INIT;
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     PostImuSM
****************************************************************************/
bool PostImuSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunImuSM

 Description
   InitPState_IMU -> IMUInit (step chain) -> IMURun
   IMURun --EV_IMU_RECOVERY--> IMUInit
   IMUInit --too many failed attempts--> IMUBackoff --timeout--> IMUInit
****************************************************************************/
ES_Event_t RunImuSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  /* Re-arm the ISR-side coalescing flags whatever state we are in, so a
   * late event can never leave them stuck. */
  if (ThisEvent.EventType == EV_IMU_DATA_UPDATE) {
      ImuDataPending = false;
  } else if (ThisEvent.EventType == EV_IMU_RECOVERY) {
      ImuRecoveryPosted = false;
  }

  switch (CurrentState)
  {
    case InitPState_IMU:
    {
      if (ThisEvent.EventType == ES_INIT) {
          ImuStartInit(IMU_POWERUP_DELAY_MS);
      }
    }
    break;

    case IMUInit:
    {
      ImuInitHandleEvent(ThisEvent);
    }
    break;

    case IMURun:
    {
      switch (ThisEvent.EventType)
      {
        case EV_IMU_DATA_UPDATE:
        {
            ImuFilterLatest();
        }
        break;

        case EV_IMU_RECOVERY:
        {
            DB_printf("IMU recovery started: %s\r\n",
                      (ThisEvent.EventParam == IMU_FAULT_POR) ? "chip reset detected"
                                                              : "too many bad frames");
            DB_printf("  bad frames: invalid=%u nodrdy=%u ovf=%u stale=%u por=%u\r\n",
                      (unsigned)ImuBadInvalidCount, (unsigned)ImuBadNoDrdyCount,
                      (unsigned)ImuBadOverflowCount, (unsigned)ImuBadStaleCount,
                      (unsigned)ImuPorCount);
            ImuStartInit(IMU_RESTART_DELAY_MS);
        }
        break;

        default:
          ;
      }
    }
    break;

    case IMUBackoff:
    {
      if (ImuIsOwnTimeout(ThisEvent)) {
          DB_printf("IMU retrying init\r\n");
          ImuStartInit(IMU_RESTART_DELAY_MS);
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
****************************************************************************/
ImuState_t QueryImuSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Public data access. All of these are safe to call from any context,
 including ISRs at a higher priority than the IMU's IPL5.
 ****************************************************************************/

/**
 * ImuGetSample
 *
 * Copies the latest validated raw sample. Returns true only while the IMU is
 * in IMURun and the sample is fresh (< 100 ms old); otherwise the contents
 * of *out are undefined.
 */
bool ImuGetSample(ImuSample_t *out)
{
    if (CurrentState != IMURun) {
        return false;
    }
    if (!ImuReadSampleRaw(out)) {
        return false;
    }
    return ImuIsFresh(out->t);
}

/**
 * GetIMUData
 *
 * Latest sample in engineering units: accel X/Y/Z (m/s^2), gyro X/Y/Z
 * (deg/s). Returns false (and writes zeros) if no fresh valid sample.
 */
bool GetIMUData(float *ImuResults)
{
    ImuSample_t s;
    uint8_t i;

    if (!ImuGetSample(&s)) {
        for (i = 0; i < 6u; i++) {
            ImuResults[i] = 0.0f;
        }
        return false;
    }
    for (i = 0; i < 3u; i++) {
        ImuResults[i] = RawAccelToMps2(s.acc[i]);
        ImuResults[i + 3u] = RawGyroToDegPerS(s.gyr[i]);
    }
    return true;
}

void WriteImuToSPI(uint8_t *Message2Send)
{
  float roll;
  float pitch;
  (void)GetAngles(&roll, &pitch);

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
 * Roll and pitch in degrees from the Mahony filter. Returns true if the IMU
 * is running and the attitude is fresh; otherwise writes 0 to both and
 * returns false.
 */
bool GetAngles(float* roll, float* pitch)
{
    ImuAttitude_t a;

    if ((CurrentState == IMURun) && ImuReadAttitude(&a) && ImuIsFresh(a.t)) {
        *roll = a.roll;
        *pitch = a.pitch;
        return true;
    }
    *roll = 0.0f;
    *pitch = 0.0f;
    return false;
}

/**
 * GetRobotYawRate
 *
 * Robot yaw rate in rad/s from gyro Z (positive = CCW viewed from above).
 * Returns false and leaves *rad_per_s unchanged if no fresh valid sample.
 */
bool GetRobotYawRate(float *rad_per_s)
{
    ImuSample_t s;

    if (!ImuGetSample(&s)) {
        return false;
    }
    *rad_per_s = IMU_YAW_GYRO_SIGN * RawGyroToDegPerS(s.gyr[2]) * IMU_DEG_TO_RAD;
    return true;
}

/***************************************************************************
 private functions: SPI transaction engine
 ***************************************************************************/

/**
 * ImuSpiFlushRx
 *
 * Discards bytes in the RX FIFO (bounded) and clears SPIROV.
 */
static void ImuSpiFlushRx(void)
{
    uint8_t guard;

    for (guard = 0; (guard < 2u * IMU_SPI_FIFO_DEPTH) && !pSPISTAT->SPIRBE; guard++) {
        (void)*pSPIBUF;
    }
    pSPISTAT->SPIROV = 0;
}

/**
 * ImuXferStart
 *
 * Starts one SPI transaction of len bytes (<= FIFO depth). The whole frame is
 * loaded into the TX FIFO with interrupts masked, so a higher-priority ISR
 * cannot stall the push and let SS rise mid-frame. Never waits: returns false
 * if a transaction is already in flight or the peripheral is not idle.
 * Completion is handled by the RX ISR according to kind. Any context.
 */
static bool ImuXferStart(ImuXferKind_t kind, const uint8_t *tx, uint8_t len)
{
    unsigned int isr_state;
    uint8_t i;

    if ((len == 0u) || (len > IMU_SPI_FIFO_DEPTH)) {
        return false;
    }

    /* Bus idle gap required by the BMI323. Sampling transfers are ~10 ms
     * apart, so in practice this only engages for back-to-back init
     * transfers; the wait is hard-bounded to IMU_SPI_IDLE_COUNTS (5 us). */
    {
        uint32_t wait_start = _CP0_GET_COUNT();
        while (((uint32_t)(_CP0_GET_COUNT() - ImuXferEndT) < IMU_SPI_IDLE_COUNTS) &&
               ((uint32_t)(_CP0_GET_COUNT() - wait_start) < IMU_SPI_IDLE_COUNTS)) {
            ;
        }
    }

    isr_state = __builtin_get_isr_state();
    __builtin_disable_interrupts();
    if ((XferKind != IMU_XFER_IDLE) || pSPISTAT->SPIBUSY || !pSPISTAT->SPITBE) {
        __builtin_set_isr_state(isr_state);
        return false;
    }
    ImuSpiFlushRx();
    XferLen = len;
    XferCount = 0;
    XferKind = kind;
    for (i = 0; i < len; i++) {
        *pSPIBUF = tx[i];
    }
    __builtin_set_isr_state(isr_state);
    return true;
}

/**
 * ImuXferAbort
 *
 * Cancels any transaction in flight without waiting: the SPI module is
 * switched off and on (which also releases SS and resets the FIFOs), the RX
 * FIFO is flushed and the engine returns to idle. Any context.
 */
static void ImuXferAbort(void)
{
    unsigned int isr_state = __builtin_get_isr_state();
    __builtin_disable_interrupts();
    pSPICON->ON = 0;
    pSPICON->ON = 1;
    ImuSpiFlushRx();
    XferKind = IMU_XFER_IDLE;
    XferCount = 0;
    ImuXferEndT = _CP0_GET_COUNT();
    ImuSpiClearRxIf();
    __builtin_set_isr_state(isr_state);
}

/**
 * ImuArmTimer
 *
 * Starts IMU_TIMER and records when, so ImuIsOwnTimeout can tell this
 * timer's expiry from a stale ES_TIMEOUT queued before the re-arm.
 */
static void ImuArmTimer(uint16_t ms)
{
    if (ms == 0u) {
        ms = 1u;
    }
    ImuTimerStartT = _CP0_GET_COUNT();
    ImuTimerHalfCounts = (uint32_t)ms * (IMU_COUNTS_PER_MS / 2u);
    ES_Timer_InitTimer(IMU_TIMER, ms);
}

/**
 * ImuIsOwnTimeout
 *
 * True if ev is an IMU_TIMER timeout for the currently armed period. A
 * timeout arriving before half the armed period has elapsed belongs to a
 * previous arming (it was already queued) and is ignored.
 */
static bool ImuIsOwnTimeout(ES_Event_t ev)
{
    if ((ev.EventType != ES_TIMEOUT) || (ev.EventParam != IMU_TIMER)) {
        return false;
    }
    return (uint32_t)(_CP0_GET_COUNT() - ImuTimerStartT) >= ImuTimerHalfCounts;
}

/***************************************************************************
 private functions: init sequence (task context)
 ***************************************************************************/

/**
 * ImuStartInit
 *
 * Stops sampling and starts a fresh init cycle (attempt budget reset) after
 * delay_ms. Used at boot, for run-time recovery and from backoff.
 */
static void ImuStartInit(uint16_t delay_ms)
{
    ImuStopSampling();
    CurrentState = IMUInit;
    ImuInitAttempts = 0;
    ImuInitBeginAttempt(delay_ms);
}

/**
 * ImuInitBeginAttempt
 *
 * Starts one pass through the init steps after delay_ms.
 */
static void ImuInitBeginAttempt(uint16_t delay_ms)
{
    InitStep = IMU_STEP_START_WAIT;
    ImuArmTimer(delay_ms);
}

/**
 * ImuInitXfer
 *
 * Starts an init transfer with a fresh id and arms the step timeout.
 */
static bool ImuInitXfer(const uint8_t *tx, uint8_t len)
{
    XferId += 1u;
    XferIdInFlight = XferId;
    if (!ImuXferStart(IMU_XFER_INIT, tx, len)) {
        return false;
    }
    ImuArmTimer(IMU_XFER_TIMEOUT_MS);
    return true;
}

/**
 * ImuInitEnterStep
 *
 * Performs the action for entering an init step: either start its transfer
 * (completion -> EV_IMU_XFER_DONE) or arm its wait timer.
 */
static void ImuInitEnterStep(ImuInitStep_t step)
{
    bool ok = true;

    InitStep = step;
    switch (step)
    {
        case IMU_STEP_DUMMY_READ:
        case IMU_STEP_WAKE_READ:
        {
            /* Any read switches the BMI323 from I2C/I3C to SPI; result ignored */
            const uint8_t tx[4] = { IMU_SPI_READ_FLAG | IMU_REG_CHIP_ID, 0u, 0u, 0u };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        case IMU_STEP_SOFT_RESET:
        {
            const uint8_t tx[3] = { IMU_REG_CMD,
                                    (uint8_t)(BMI323_CMD_SOFT_RESET & 0xFFu),
                                    (uint8_t)(BMI323_CMD_SOFT_RESET >> 8) };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        case IMU_STEP_RESET_WAIT:
            ImuArmTimer(IMU_RESET_DELAY_MS);
            break;

        case IMU_STEP_READ_ID:
        {
            /* echo, dummy, CHIP_ID(2), ERR_REG(2), STATUS(2). Reading STATUS
             * also clears the por_detected flag set by the soft reset; the
             * sample path treats por_detected as "chip reset, re-init". */
            const uint8_t tx[8] = { IMU_SPI_READ_FLAG | IMU_REG_CHIP_ID };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        case IMU_STEP_WRITE_CONF:
        {
            const uint8_t tx[5] = { IMU_REG_ACC_CONF,
                                    (uint8_t)(IMU_ACC_CONF_VAL & 0xFFu),
                                    (uint8_t)(IMU_ACC_CONF_VAL >> 8),
                                    (uint8_t)(IMU_GYR_CONF_VAL & 0xFFu),
                                    (uint8_t)(IMU_GYR_CONF_VAL >> 8) };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        case IMU_STEP_VERIFY_CONF:
        {
            /* echo, dummy, ACC_CONF(2), GYR_CONF(2) */
            const uint8_t tx[6] = { IMU_SPI_READ_FLAG | IMU_REG_ACC_CONF };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        case IMU_STEP_SETTLE_WAIT:
            ImuArmTimer(IMU_SETTLE_MS);
            break;

        case IMU_STEP_CHECK_READY:
        {
            /* echo, dummy, ERR_REG(2), STATUS(2) */
            const uint8_t tx[6] = { IMU_SPI_READ_FLAG | IMU_REG_ERR };
            ok = ImuInitXfer(tx, sizeof(tx));
        }
        break;

        default:
            ok = false;
            break;
    }

    if (!ok) {
        ImuInitFail("SPI busy, transfer not started");
    }
}

/**
 * ImuInitHandleEvent
 *
 * IMUInit event handler. ES_TIMEOUT ends a wait step or, during a transfer
 * step, means the transfer timed out. EV_IMU_XFER_DONE carries the transfer
 * id; completions from abandoned attempts are ignored.
 */
static void ImuInitHandleEvent(ES_Event_t ev)
{
    if ((ev.EventType == ES_TIMEOUT) && !ImuIsOwnTimeout(ev)) {
        return;     /* stale timeout from a previous arming */
    }
    if (ev.EventType == ES_TIMEOUT) {
        switch (InitStep)
        {
            case IMU_STEP_START_WAIT:  ImuInitEnterStep(IMU_STEP_DUMMY_READ); break;
            case IMU_STEP_RESET_WAIT:  ImuInitEnterStep(IMU_STEP_WAKE_READ); break;
            case IMU_STEP_SETTLE_WAIT: ImuInitEnterStep(IMU_STEP_CHECK_READY); break;
            default:                   ImuInitFail("transfer timeout"); break;
        }
        return;
    }

    if ((ev.EventType != EV_IMU_XFER_DONE) || ((uint8_t)ev.EventParam != XferId)) {
        return;
    }

    switch (InitStep)
    {
        case IMU_STEP_DUMMY_READ:
            ImuInitEnterStep(IMU_STEP_SOFT_RESET);
            break;

        case IMU_STEP_SOFT_RESET:
            ImuInitEnterStep(IMU_STEP_RESET_WAIT);
            break;

        case IMU_STEP_WAKE_READ:
            ImuInitEnterStep(IMU_STEP_READ_ID);
            break;

        case IMU_STEP_READ_ID:
        {
            uint8_t chip_id = (uint8_t)(ImuRxWord(IMU_RX_DATA_OFFSET) & 0xFFu);
            uint16_t err = ImuRxWord(IMU_RX_DATA_OFFSET + 2u);

            if (chip_id != BMI323_CHIP_ID_VAL) {
                DB_printf("IMU chip id 0x%x, expected 0x%x\r\n",
                          (unsigned)chip_id, (unsigned)BMI323_CHIP_ID_VAL);
                ImuInitFail("wrong chip id");
            } else if (err & IMU_ERR_FATAL) {
                ImuInitFail("ERR_REG.fatal_err set");
            } else {
                ImuInitEnterStep(IMU_STEP_WRITE_CONF);
            }
        }
        break;

        case IMU_STEP_WRITE_CONF:
            ImuInitEnterStep(IMU_STEP_VERIFY_CONF);
            break;

        case IMU_STEP_VERIFY_CONF:
        {
            uint16_t acc = ImuRxWord(IMU_RX_DATA_OFFSET);
            uint16_t gyr = ImuRxWord(IMU_RX_DATA_OFFSET + 2u);

            if (((acc & IMU_CONF_MASK) != (IMU_ACC_CONF_VAL & IMU_CONF_MASK)) ||
                ((gyr & IMU_CONF_MASK) != (IMU_GYR_CONF_VAL & IMU_CONF_MASK))) {
                DB_printf("IMU conf readback acc=0x%x gyr=0x%x\r\n",
                          (unsigned)acc, (unsigned)gyr);
                ImuInitFail("config readback mismatch");
            } else {
                ImuInitEnterStep(IMU_STEP_SETTLE_WAIT);
            }
        }
        break;

        case IMU_STEP_CHECK_READY:
        {
            uint16_t err = ImuRxWord(IMU_RX_DATA_OFFSET);
            uint16_t status = ImuRxWord(IMU_RX_DATA_OFFSET + 2u);

            if (err & (IMU_ERR_FATAL | IMU_ERR_ACC_CONF | IMU_ERR_GYR_CONF)) {
                DB_printf("IMU ERR_REG 0x%x\r\n", (unsigned)err);
                ImuInitFail("ERR_REG error after config");
            } else if (status & IMU_STATUS_POR_DETECTED) {
                ImuInitFail("chip reset during init");
            } else if ((status & (IMU_STATUS_DRDY_ACC | IMU_STATUS_DRDY_GYR)) !=
                       (IMU_STATUS_DRDY_ACC | IMU_STATUS_DRDY_GYR)) {
                DB_printf("IMU STATUS 0x%x\r\n", (unsigned)status);
                ImuInitFail("sensors not producing data");
            } else {
                ImuEnterRun();
            }
        }
        break;

        default:
            break;
    }
}

/**
 * ImuInitFail
 *
 * Aborts the current attempt. Retries up to IMU_INIT_MAX_ATTEMPTS, then
 * enters backoff.
 */
static void ImuInitFail(const char *why)
{
    ImuXferAbort();
    ImuInitAttempts += 1u;
    ImuInitFailCount += 1u;
    DB_printf("IMU init: %s at step %s (attempt %u/%u)\r\n", why,
              (InitStep < IMU_STEP_COUNT) ? ImuStepNames[InitStep] : "?",
              (unsigned)ImuInitAttempts, (unsigned)IMU_INIT_MAX_ATTEMPTS);

    if (ImuInitAttempts < IMU_INIT_MAX_ATTEMPTS) {
        ImuInitBeginAttempt(IMU_ATTEMPT_DELAY_MS);
    } else {
        ImuEnterBackoff();
    }
}

/**
 * ImuEnterRun
 *
 * Init succeeded: reset the filter and fault counters and start sampling.
 */
static void ImuEnterRun(void)
{
    ImuMahonyReset();
    ImuLastFilteredSeq = 0;
    ImuHaveFilterTime = false;
    ImuConsecutiveBadFrames = 0;
    ImuRecoveryPosted = false;
    ImuDataPending = false;
    ImuTransferStaleTicks = 0;
    ImuRetryDelayMs = IMU_BACKOFF_MIN_MS;
    ImuSetFaultLed(false);

    CurrentState = IMURun;
    ImuAllowRecovery = 1;
    TMR6 = 0;
    IFS0CLR = _IFS0_T6IF_MASK;
    T6CONbits.ON = 1;
    DB_printf("IMU running\r\n");
}

/**
 * ImuEnterBackoff
 *
 * All attempts in this cycle failed: fault LED on, retry after
 * ImuRetryDelayMs (doubling up to IMU_BACKOFF_MAX_MS). Never terminal.
 */
static void ImuEnterBackoff(void)
{
    ImuStopSampling();
    CurrentState = IMUBackoff;
    ImuSetFaultLed(true);
    DB_printf("IMU init failed %u times; retrying in %u ms\r\n",
              (unsigned)IMU_INIT_MAX_ATTEMPTS, (unsigned)ImuRetryDelayMs);
    ImuArmTimer(ImuRetryDelayMs);
    if (ImuRetryDelayMs < IMU_BACKOFF_MAX_MS) {
        ImuRetryDelayMs = (uint16_t)(ImuRetryDelayMs * 2u);
        if (ImuRetryDelayMs > IMU_BACKOFF_MAX_MS) {
            ImuRetryDelayMs = IMU_BACKOFF_MAX_MS;
        }
    }
}

/**
 * ImuStopSampling
 *
 * Stops T6, cancels any transfer in flight (no waiting) and invalidates the
 * published sample and attitude.
 */
static void ImuStopSampling(void)
{
    T6CONbits.ON = 0;
    IFS0CLR = _IFS0_T6IF_MASK;
    ImuAllowRecovery = 0;
    ImuXferAbort();
    ImuTransferStaleTicks = 0;
    ImuHaveLastGood = false;
#if IMU_TEST_HOOKS
    ImuInjectBadFrames = 0;  /* don't carry test corruption past a re-init */
    ImuInjectSoftReset = false;
#endif
    ImuInvalidatePublished();
}

/**
 * ImuSetFaultLed
 *
 * Posts EV_LED_ON / EV_LED_OFF to LEDService only on a change.
 */
static void ImuSetFaultLed(bool on)
{
    ES_Event_t ev;

    if (on == ImuFaultLedOn) {
        return;
    }
    ImuFaultLedOn = on;
    ev.EventType = on ? EV_LED_ON : EV_LED_OFF;
    ev.EventParam = IMU_FAULT_LED;
    (void)PostLEDService(ev);
}

/***************************************************************************
 private functions: sample path (IPL5 ISR context)
 ***************************************************************************/

/**
 * ImuProcessSampleFrame
 *
 * Validates a complete sample frame in RxBuf[] and, only if it is good,
 * publishes it and notifies RunImuSM.
 *  - STATUS.por_detected: the chip reset itself -> request re-init now.
 *  - Neither drdy bit set: no new sample since the last read -> drop.
 *  - Any axis == 0x8000 (invalid marker) -> drop.
 */
static void ImuProcessSampleFrame(void)
{
    uint32_t now = _CP0_GET_COUNT();
    int16_t raw[6];
    uint16_t status;
    uint8_t i;

#if IMU_TEST_HOOKS
    if (ImuInjectBadFrames > 0u) {
        /* Simulate the chip's invalid-data marker on accel X */
        ImuInjectBadFrames -= 1u;
        RxBuf[IMU_SAMPLE_DATA_OFS] = 0x00u;
        RxBuf[IMU_SAMPLE_DATA_OFS + 1u] = 0x80u;
    }
#endif

    status = ImuRxWord(IMU_SAMPLE_STATUS_OFS);

    if (status & IMU_STATUS_POR_DETECTED) {
        ImuPorCount += 1u;
        ImuRequestRecovery(IMU_FAULT_POR);
        return;
    }

    if ((status & (IMU_STATUS_DRDY_ACC | IMU_STATUS_DRDY_GYR)) == 0u) {
        ImuNoteBadFrame(&ImuBadNoDrdyCount);
        return;
    }

    for (i = 0; i < 6u; i++) {
        raw[i] = (int16_t)ImuRxWord(IMU_SAMPLE_DATA_OFS + 2u * i);
        if (raw[i] == IMU_INVALID_SAMPLE) {
            ImuNoteBadFrame(&ImuBadInvalidCount);
            return;
        }
    }

    /* Good frame */
    ImuConsecutiveBadFrames = 0;
    ImuPublishSample(raw, now);

    if (ImuHaveLastGood) {
        /* Integer EMA (1/64) in core-timer counts: keeps the ISR FPU-free */
        int32_t dt_counts = (int32_t)(now - ImuLastGoodT);
        if ((dt_counts > 0) && (dt_counts <= (int32_t)(IMU_MAX_AGE_COUNTS / 2u))) {
            ImuSamplePeriodAvg += (dt_counts - ImuSamplePeriodAvg) / 64;
        }
    }
    ImuLastGoodT = now;
    ImuHaveLastGood = true;

    if (!ImuDataPending) {
        ImuDataPending = true;
        if (!ImuPostFromIsr(EV_IMU_DATA_UPDATE, 0u)) {
            ImuDataPending = false;  /* retry with the next sample */
        }
    }
}

/**
 * ImuNoteBadFrame
 *
 * Records one dropped frame. Requests recovery only after
 * IMU_BAD_FRAME_LIMIT bad frames in a row.
 */
static void ImuNoteBadFrame(volatile uint16_t *counter)
{
    *counter += 1u;
    if (ImuConsecutiveBadFrames < 255u) {
        ImuConsecutiveBadFrames += 1u;
    }
    if (ImuConsecutiveBadFrames >= IMU_BAD_FRAME_LIMIT) {
        ImuRequestRecovery(IMU_FAULT_BAD_FRAMES);
    }
}

/**
 * ImuRequestRecovery
 *
 * Posts EV_IMU_RECOVERY once (coalesced) while in IMURun. If the post fails
 * the flag is cleared so the next bad frame retries.
 */
static void ImuRequestRecovery(uint8_t reason)
{
    if (!ImuAllowRecovery || ImuRecoveryPosted) {
        return;
    }
    ImuRecoveryPosted = true;
    if (!ImuPostFromIsr(EV_IMU_RECOVERY, reason)) {
        ImuRecoveryPosted = false;
    }
}

static bool ImuPostFromIsr(ES_EventType_t type, uint16_t param)
{
    ES_Event_t ev;

    ev.EventType = type;
    ev.EventParam = param;
    return PostImuSM(ev);
}

/**
 * ImuPublishSample
 *
 * Writes a validated sample into the back buffer and flips it to the front.
 */
static void ImuPublishSample(const int16_t *raw, uint32_t t)
{
    uint8_t back = (uint8_t)(SampleFront ^ 1u);
    volatile ImuSample_t *p = &SampleBuf[back];
    uint8_t i;

    p->seq = 0u;    /* mark "being written" for any reader still on it */
    p->t = t;
    for (i = 0; i < 3u; i++) {
        p->acc[i] = raw[i];
        p->gyr[i] = raw[i + 3u];
    }
    SampleSeq += 1u;
    if (SampleSeq == 0u) {
        SampleSeq = 1u;
    }
    p->seq = SampleSeq;
    SampleFront = back;
}

/***************************************************************************
 private functions: readers and filter
 ***************************************************************************/

/**
 * ImuReadSampleRaw
 *
 * Consistent copy of the front sample buffer (no state/freshness check).
 * A reader at higher priority than the writer always reads the front buffer
 * the writer is not touching; a lower-priority reader that gets lapped sees
 * seq change and retries (bounded).
 */
static bool ImuReadSampleRaw(ImuSample_t *out)
{
    uint8_t tries;
    uint8_t i;

    for (tries = 0; tries < 4u; tries++) {
        volatile ImuSample_t *p = &SampleBuf[SampleFront];
        uint32_t seq = p->seq;

        if (seq == 0u) {
            continue;
        }
        out->t = p->t;
        for (i = 0; i < 3u; i++) {
            out->acc[i] = p->acc[i];
            out->gyr[i] = p->gyr[i];
        }
        if (p->seq == seq) {
            out->seq = seq;
            return true;
        }
    }
    return false;
}

static bool ImuReadAttitude(ImuAttitude_t *out)
{
    uint8_t tries;

    for (tries = 0; tries < 4u; tries++) {
        volatile ImuAttitude_t *p = &AttBuf[AttFront];
        uint32_t seq = p->seq;

        if (seq == 0u) {
            continue;
        }
        out->t = p->t;
        out->roll = p->roll;
        out->pitch = p->pitch;
        if (p->seq == seq) {
            out->seq = seq;
            return true;
        }
    }
    return false;
}

static bool ImuIsFresh(uint32_t t)
{
    return (uint32_t)(_CP0_GET_COUNT() - t) <= IMU_MAX_AGE_COUNTS;
}

/**
 * ImuInvalidatePublished
 *
 * Marks the published sample and attitude invalid (sampling is stopped when
 * this is called, so the ISR is not writing).
 */
static void ImuInvalidatePublished(void)
{
    SampleBuf[0].seq = 0u;
    SampleBuf[1].seq = 0u;
    AttBuf[0].seq = 0u;
    AttBuf[1].seq = 0u;
}

/**
 * ImuFilterLatest
 *
 * EV_IMU_DATA_UPDATE handler: runs the Mahony filter on the newest sample
 * (if not already processed) with dt measured from sample timestamps, then
 * publishes roll/pitch. If the main loop falls behind, intermediate samples
 * are skipped and the larger measured dt covers the interval.
 */
static void ImuFilterLatest(void)
{
    ImuSample_t s;
    float dt = DT;

    if (!ImuReadSampleRaw(&s) || (s.seq == ImuLastFilteredSeq)) {
        return;
    }
    if (ImuHaveFilterTime) {
        dt = (float)(s.t - ImuLastFilteredT) * (1.0f / IMU_CORE_TIMER_HZ);
        if (!(dt > 0.0f) || (dt > IMU_MAX_FILTER_DT_S)) {
            dt = DT;
        }
    }
    ImuLastFilteredSeq = s.seq;
    ImuLastFilteredT = s.t;
    ImuHaveFilterTime = true;

    MahonyUpdate(RawAccelToMps2(s.acc[0]), RawAccelToMps2(s.acc[1]),
                 RawAccelToMps2(s.acc[2]), RawGyroToDegPerS(s.gyr[0]),
                 RawGyroToDegPerS(s.gyr[1]), RawGyroToDegPerS(s.gyr[2]), dt);
    ImuPublishAttitude(s.t);
}

/**
 * ImuPublishAttitude
 *
 * Converts the quaternion to roll/pitch (deg) and publishes it. A non-finite
 * filter state resets the filter instead of publishing garbage.
 */
static void ImuPublishAttitude(uint32_t t)
{
    uint8_t back;
    volatile ImuAttitude_t *p;
    float sin_pitch = 2.0f * (q0 * q2 - q1 * q3);
    float roll;
    float pitch;

    if (!isfinite(q0) || !isfinite(q1) || !isfinite(q2) || !isfinite(q3) ||
        !isfinite(sin_pitch)) {
        ImuMahonyReset();
        return;
    }
    sin_pitch = fmaxf(fminf(sin_pitch, 1.0f), -1.0f);
    /* Standard ZYX roll: atan2(q0q1 + q2q3, 1/2 - q1^2 - q2^2). (The previous
     * code had "+ q2^2", which skewed roll as pitch grew.) */
    roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * IMU_RAD_TO_DEG;
    pitch = asinf(sin_pitch) * IMU_RAD_TO_DEG;
    if (!isfinite(roll) || !isfinite(pitch)) {
        return;
    }

    back = (uint8_t)(AttFront ^ 1u);
    p = &AttBuf[back];
    p->seq = 0u;
    p->t = t;
    p->roll = roll;
    p->pitch = pitch;
    AttSeq += 1u;
    if (AttSeq == 0u) {
        AttSeq = 1u;
    }
    p->seq = AttSeq;
    AttFront = back;
}

/**
 * ImuMahonyReset
 *
 * Reinitializes the Mahony quaternion and integral error state.
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

static float RawAccelToMps2(int16_t raw)
{
    return (float)raw * GRAVITY_MPS2 / ACCEL_LSB_PER_G;
}

static float RawGyroToDegPerS(int16_t raw)
{
    return (float)raw / GYRO_LSB_PER_DPS;
}

/* Little-endian 16-bit word from RxBuf */
static uint16_t ImuRxWord(uint8_t ofs)
{
    return (uint16_t)((uint16_t)RxBuf[ofs] | ((uint16_t)RxBuf[ofs + 1u] << 8));
}

/**
 * MahonyUpdate
 * Mahony filter update from accelerometer and gyroscope measurements. Code
 * modified from: https://github.com/PaulStoffregen/MahonyAHRS/blob/master/src/MahonyAHRS.cpp#L170
 * Runs in task context (RunImuSM).
 *
 * @param ax, ay, az - acceleration (any consistent units)
 * @param gx, gy, gz - angular velocity (deg/sec)
 * @param dt - time step (seconds)
 */
static void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    float recipNorm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= IMU_DEG_TO_RAD;
    gy *= IMU_DEG_TO_RAD;
    gz *= IMU_DEG_TO_RAD;

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
        integralFBx += TWO_KI * halfex * dt;
        integralFBy += TWO_KI * halfey * dt;
        integralFBz += TWO_KI * halfez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += TWO_KP * halfex;
    gy += TWO_KP * halfey;
    gz += TWO_KP * halfez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
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

/***************************************************************************
 Interrupt handlers (IPL5)
 ***************************************************************************/

/**
 * ImuSpiRxIsr
 *
 * Collects the bytes of the transaction in flight and dispatches on
 * completion: sample frames are validated/published here, init transfers
 * post EV_IMU_XFER_DONE. Bytes with no transaction in flight are discarded.
 * On RX overflow the transaction is aborted (a sample counts as a bad frame;
 * an init step fails through its ES timer timeout).
 */
void __ISR(IMU_SPI_RX_ISR_VECTOR, IPL5SRS) ImuSpiRxIsr(void)
{
    uint8_t guard;

    /* Clear first: the "not empty" source re-asserts if bytes remain */
    ImuSpiClearRxIf();

    if (pSPISTAT->SPIROV) {
        ImuXferKind_t kind = XferKind;
        ImuXferAbort();
        if (kind == IMU_XFER_SAMPLE) {
            ImuNoteBadFrame(&ImuBadOverflowCount);
        }
        return;
    }

    for (guard = 0; (guard < IMU_SPI_FIFO_DEPTH) && !pSPISTAT->SPIRBE; guard++) {
        uint8_t byte = (uint8_t)*pSPIBUF;

        if (XferKind == IMU_XFER_IDLE) {
            continue;   /* stray byte: discard */
        }
        RxBuf[XferCount] = byte;
        XferCount += 1u;
        if (XferCount >= XferLen) {
            ImuXferKind_t kind = XferKind;
            XferKind = IMU_XFER_IDLE;
            ImuXferEndT = _CP0_GET_COUNT();   /* start of the required bus idle */
            if (kind == IMU_XFER_SAMPLE) {
                ImuProcessSampleFrame();
            } else if (kind == IMU_XFER_INIT) {
                /* If the post fails, the step's timeout fails the attempt */
                (void)ImuPostFromIsr(EV_IMU_XFER_DONE, XferIdInFlight);
            }
        }
    }
}

void __ISR(IMU_SPI_TX_ISR_VECTOR, IPL5SRS) ImuSpiTxIsr(void)
{
    ImuSpiClearTxIf();  /* TX interrupt is never enabled; defensive */
}

/**
 * T6Handler
 *
 * ~100 Hz sample trigger (runs only in IMURun). Starts a sample burst, or
 * counts a stuck transfer as a bad frame and aborts it.
 */
void __ISR(_TIMER_6_VECTOR, IPL5SRS) T6Handler(void)
{
    IFS0CLR = _IFS0_T6IF_MASK;

    if (XferKind != IMU_XFER_IDLE) {
        ImuTransferStaleTicks += 1u;
        if (ImuTransferStaleTicks >= IMU_TRANSFER_STALE_TICKS) {
            ImuTransferStaleTicks = 0;
            ImuXferAbort();
            ImuNoteBadFrame(&ImuBadStaleCount);
        }
        return;
    }
    ImuTransferStaleTicks = 0;

#if IMU_TEST_HOOKS
    if (ImuInjectSoftReset) {
        static const uint8_t tx[3] = { IMU_REG_CMD,
                                       (uint8_t)(BMI323_CMD_SOFT_RESET & 0xFFu),
                                       (uint8_t)(BMI323_CMD_SOFT_RESET >> 8) };
        ImuInjectSoftReset = false;
        (void)ImuXferStart(IMU_XFER_DISCARD, tx, sizeof(tx));
        return;
    }
#endif

    if (!ImuXferStart(IMU_XFER_SAMPLE, ImuSampleTx, IMU_SAMPLE_BYTES)) {
        /* Peripheral not idle although no transfer is tracked: resync */
        ImuXferAbort();
        ImuNoteBadFrame(&ImuBadStaleCount);
    }
}

/****************************************************************************
 Diagnostics / test hooks (called from main-line code, e.g. UsbService)
 ****************************************************************************/

/**
 * ImuPrintDiagnostics
 *
 * Prints state, fault counters, the latest sample and attitude. Counters
 * accumulate since boot.
 */
void ImuPrintDiagnostics(void)
{
    static const char * const StateNames[] = {
        "InitPState", "Init", "Run", "Backoff"
    };
    ImuState_t state = CurrentState;
    ImuSample_t s;
    float roll;
    float pitch;
    bool fresh_angles;

    DB_printf("IMU state: %s", (state <= IMUBackoff) ? StateNames[state] : "?");
    if (state == IMUInit) {
        DB_printf(" (step %s, attempt %u)",
                  (InitStep < IMU_STEP_COUNT) ? ImuStepNames[InitStep] : "?",
                  (unsigned)(ImuInitAttempts + 1u));
    }
    DB_printf("\r\n");
    DB_printf("  consecutive bad: %u, init failures: %u, next backoff: %u ms, fault LED: %u\r\n",
              (unsigned)ImuConsecutiveBadFrames, (unsigned)ImuInitFailCount,
              (unsigned)ImuRetryDelayMs, (unsigned)ImuFaultLedOn);
    DB_printf("  bad frames: invalid=%u nodrdy=%u ovf=%u stale=%u por=%u\r\n",
              (unsigned)ImuBadInvalidCount, (unsigned)ImuBadNoDrdyCount,
              (unsigned)ImuBadOverflowCount, (unsigned)ImuBadStaleCount,
              (unsigned)ImuPorCount);

    if (ImuReadSampleRaw(&s)) {
        /* accel in cm/s^2, gyro in 0.1 deg/s (DB_printf has no float support) */
        DB_printf("  sample #%u, age %u ms%s\r\n", (unsigned)s.seq,
                  (unsigned)((uint32_t)(_CP0_GET_COUNT() - s.t) / 100000u),
                  ImuIsFresh(s.t) ? "" : " (STALE)");
        DB_printf("  accel cm/s^2: %d %d %d\r\n",
                  (int)(100.0f * RawAccelToMps2(s.acc[0])),
                  (int)(100.0f * RawAccelToMps2(s.acc[1])),
                  (int)(100.0f * RawAccelToMps2(s.acc[2])));
        DB_printf("  gyro 0.1dps:  %d %d %d\r\n",
                  (int)(10.0f * RawGyroToDegPerS(s.gyr[0])),
                  (int)(10.0f * RawGyroToDegPerS(s.gyr[1])),
                  (int)(10.0f * RawGyroToDegPerS(s.gyr[2])));
    } else {
        DB_printf("  no valid sample\r\n");
    }

    fresh_angles = GetAngles(&roll, &pitch);
    DB_printf("  roll, pitch deg: %d, %d%s\r\n", (int)roll, (int)pitch,
              fresh_angles ? "" : " (invalid)");
    DB_printf("  sample period: %u us (nominal %u us)\r\n",
              (unsigned)(ImuSamplePeriodAvg / 100), (unsigned)(DT * 1.0e6f));
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
 * sample burst, simulating a brownout. Should be detected (por_detected or
 * bad frames) and recovered.
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
#endif
