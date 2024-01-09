/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef JetsonFSM_H
#define JetsonFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Jetson, RobotInactive, RobotPending, RobotActive
}JetsonState_t;

// Public Function Prototypes

bool InitJetsonSM(uint8_t Priority);
bool PostJetsonSM(ES_Event_t ThisEvent);
ES_Event_t RunJetsonSM(ES_Event_t ThisEvent);
JetsonState_t QueryJetsonSM(void);

#endif /* JetsonFSM_H */

