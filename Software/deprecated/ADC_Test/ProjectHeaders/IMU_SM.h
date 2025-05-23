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
  InitPState_IMU, IMUWait
}ImuState_t;

typedef struct
{
    uint8_t LowerByte;
    uint8_t UpperByte;
} DataByBytes_t;

typedef union
{
    DataByBytes_t DataStruct;
    int16_t FullData;
} AccelGyroData_t;

// Public Function Prototypes

bool InitImuSM(uint8_t Priority);
bool PostImuSM(ES_Event_t ThisEvent);
ES_Event_t RunImuSM(ES_Event_t ThisEvent);
ImuState_t QueryImuSM(void);
void WriteImuToSPI(uint32_t Buffer);
void WriteAccelToSPI(uint32_t Buffer);
void WriteGyroToSPI(uint32_t Buffer);
#endif /* ImuFSM_H */

