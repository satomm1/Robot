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
  InitPState_IMU, IMU1, IMU2
}ImuState_t;

// Public Function Prototypes

bool InitImuSM(uint8_t Priority);
bool PostImuSM(ES_Event_t ThisEvent);
ES_Event_t RunImuSM(ES_Event_t ThisEvent);
ImuState_t QueryImuSM(void);

#endif /* ImuFSM_H */

