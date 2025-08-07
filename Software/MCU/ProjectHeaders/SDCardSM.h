/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef SDCard_FSM_H
#define SDCard_FSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_SD, SDReset, SDWait, SDRun
}SDCardState_t;

// Public Function Prototypes

bool InitSDCardSM(uint8_t Priority);
bool PostSDCardSM(ES_Event_t ThisEvent);
ES_Event_t RunSDCardSM(ES_Event_t ThisEvent);
SDCardState_t QuerySDCardSM(void);
#endif /* SDCard_FSM_H */

