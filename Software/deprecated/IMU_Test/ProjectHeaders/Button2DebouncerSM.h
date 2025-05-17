/****************************************************************************

  Header file Button2Debouncer SM

 ****************************************************************************/

#ifndef Button2DebouncerSM_H
#define Button2DebouncerSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Button2Debouncer, Button2DebouncingWait, Button2DebouncingFall, Button2DebouncingRise
}Button2DebouncerState_t;

// Public Function Prototypes

bool InitButton2DebouncerSM(uint8_t Priority);
bool PostButton2DebouncerSM(ES_Event_t ThisEvent);
ES_Event_t RunButton2DebouncerSM(ES_Event_t ThisEvent);
Button2DebouncerState_t QueryButton2DebouncerSM(void);

#endif /* Button2DebouncerSM_H */

