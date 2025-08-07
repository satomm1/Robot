/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef EnvironmentSensor_FSM_H
#define EnvironmentSensor_FSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Env, EnvReset, EnvWait, EnvRun
}EnvironmentSensorState_t;

// Public Function Prototypes

bool InitEnvironmentSensorSM(uint8_t Priority);
bool PostEnvironmentSensorSM(ES_Event_t ThisEvent);
ES_Event_t RunEnvironmentSensorSM(ES_Event_t ThisEvent);
EnvironmentSensorState_t QueryEnvironmentSensorSM(void);
#endif /* EnvironmentSensor_FSM_H */

