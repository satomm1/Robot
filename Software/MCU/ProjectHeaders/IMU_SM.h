/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef ImuFSM_H
#define ImuFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_IMU,
  IMUInit,      /* non-blocking init sequence (reset, verify, configure) */
  IMURun,       /* sampling; data valid */
  IMUBackoff    /* init failed repeatedly; waiting to retry (never terminal) */
}ImuState_t;

/* One validated raw sample (see ImuGetSample) */
typedef struct
{
    uint32_t seq;    /* increments per published sample (never 0 when valid) */
    uint32_t t;      /* CP0 core timer count when the sample was received */
    int16_t acc[3];  /* raw accel X/Y/Z, 8192 LSB/g (+/-4 g) */
    int16_t gyr[3];  /* raw gyro X/Y/Z, 131.2 LSB/(deg/s) (+/-250 dps) */
} ImuSample_t;

// Public Function Prototypes

bool InitImuSM(uint8_t Priority);
bool PostImuSM(ES_Event_t ThisEvent);
ES_Event_t RunImuSM(ES_Event_t ThisEvent);
ImuState_t QueryImuSM(void);

/* Data access: safe from any context, including higher-priority ISRs.
 * Each returns false when the IMU is not running or the data is stale
 * (older than 100 ms). */
bool ImuGetSample(ImuSample_t *out);
bool GetIMUData(float *ImuResults);          /* m/s^2 x3, deg/s x3; zeros if false */
bool GetAngles(float* roll, float* pitch);   /* deg; zeros if false */
bool GetRobotYawRate(float *rad_per_s);      /* unchanged if false */
void WriteImuToSPI(uint8_t *Message2Send);
void ImuPrintDiagnostics(void);

/* Set to 0 to compile out IMU fault-injection test hooks */
#define IMU_TEST_HOOKS 1
#if IMU_TEST_HOOKS
void ImuTestInjectBadFrames(uint8_t n);
void ImuTestInjectSoftReset(void);
#endif
#endif /* ImuFSM_H */

