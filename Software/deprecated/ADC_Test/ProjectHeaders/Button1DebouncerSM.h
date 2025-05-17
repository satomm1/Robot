/****************************************************************************

  Header file Button1Debouncer SM

 ****************************************************************************/

#ifndef Button1DebouncerSM_H
#define Button1DebouncerSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Button1Debouncer, Button1DebouncingWait, Button1DebouncingFall, Button1DebouncingRise
}Button1DebouncerState_t;

// Public Function Prototypes

bool InitButton1DebouncerSM(uint8_t Priority);
bool PostButton1DebouncerSM(ES_Event_t ThisEvent);
ES_Event_t RunButton1DebouncerSM(ES_Event_t ThisEvent);
Button1DebouncerState_t QueryButton1DebouncerSM(void);

#endif /* Button1DebouncerSM_H */

