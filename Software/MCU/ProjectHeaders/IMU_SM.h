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
  InitPState_IMU, IMUReset, IMUWait, IMURun
}ImuState_t;

typedef struct
{
    uint8_t LowerByte;
    uint8_t UpperByte;
} DataByBytes_t;

typedef union
{
    DataByBytes_t DataStruct;
    uint16_t FullData;
} AccelGyroData_t;

typedef union
{
    DataByBytes_t DataStruct;
    uint16_t FullData;
} ReceivedData_t;

// Public Function Prototypes

bool InitImuSM(uint8_t Priority);
bool PostImuSM(ES_Event_t ThisEvent);
ES_Event_t RunImuSM(ES_Event_t ThisEvent);
ImuState_t QueryImuSM(void);
void GetIMUData(float *ImuResults);
void WriteImuToSPI(uint8_t *Message2Send);
uint8_t ReadIMU8(uint8_t Address);
uint16_t ReadIMU16(uint8_t Address);
#endif /* ImuFSM_H */

