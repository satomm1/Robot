/****************************************************************************

  Header file Button3Debouncer SM

 ****************************************************************************/

#ifndef Button3DebouncerSM_H
#define Button3DebouncerSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Button3Debouncer, Button3DebouncingWait, Button3DebouncingFall, Button3DebouncingRise
}Button3DebouncerState_t;

// Public Function Prototypes

bool InitButton3DebouncerSM(uint8_t Priority);
bool PostButton3DebouncerSM(ES_Event_t ThisEvent);
ES_Event_t RunButton3DebouncerSM(ES_Event_t ThisEvent);
Button3DebouncerState_t QueryButton3DebouncerSM(void);

#endif /* Button3DebouncerSM_H */

