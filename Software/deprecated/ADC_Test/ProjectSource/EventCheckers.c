/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"

static uint8_t Button1Status;
static uint8_t Button2Status;
static uint8_t Button3Status;

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType   = ES_NEW_KEY;
    ThisEvent.EventParam  = GetNewKey();
    ES_PostAll(ThisEvent);
    return true;
  }
  return false;
}

void InitButton1(void) 
{
    // Set Button1 Pin to digital input
    TRISHSET = _TRISH_TRISH9_MASK;
    Button1Status = PORTHbits.RH9;
}

void InitButton2(void) 
{
    // Set Button1 Pin to digital input
    TRISHSET = _TRISH_TRISH10_MASK;
    Button2Status = PORTHbits.RH10;
}

void InitButton3(void) 
{
    // Set Button1 Pin to digital input
    TRISHSET = _TRISH_TRISH11_MASK;
    Button3Status = PORTHbits.RH11;
}

bool CheckButton1(void)
{
    bool ReturnVal = false;
    
    // Get current state of button
    uint8_t CurrentButton1Status = PORTHbits.RH9;
    
    if (Button1Status != CurrentButton1Status) {
        // button state has changed
        ReturnVal = true;
        
        // Post the event of a button up or down
        ES_Event_t ThisEvent;
        if (CurrentButton1Status == 1) { // Button is down
            ThisEvent.EventType = EV_BUTTON1_DOWN;
//            DB_printf("Button1Down\r\n");
            PostButton1DebouncerSM(ThisEvent);
        } else { // button is now up
            ThisEvent.EventType = EV_BUTTON1_UP;
//            DB_printf("Button1Up\r\n");
            PostButton1DebouncerSM(ThisEvent);
        }
        
        // update the last button state
        Button1Status = CurrentButton1Status;
    }
    return ReturnVal;
}

bool CheckButton2(void)
{
    bool ReturnVal = false;
    
    // Get current state of button
    uint8_t CurrentButton2Status = PORTHbits.RH10;
    
    if (Button2Status != CurrentButton2Status) {
        // button state has changed
        ReturnVal = true;
        
        // Post the event of a button up or down
        ES_Event_t ThisEvent;
        if (CurrentButton2Status == 1) { // Button is down
            ThisEvent.EventType = EV_BUTTON2_DOWN;
            PostButton2DebouncerSM(ThisEvent);
        } else { // button is now up
            ThisEvent.EventType = EV_BUTTON2_UP;
            PostButton2DebouncerSM(ThisEvent);
        }
        
        // update the last button state
        Button2Status = CurrentButton2Status;
    }
    return ReturnVal;
}

bool CheckButton3(void)
{
    bool ReturnVal = false;
    
    // Get current state of button
    uint8_t CurrentButton3Status = PORTHbits.RH11;
    
    if (Button3Status != CurrentButton3Status) {
        // button state has changed
        ReturnVal = true;
        
        // Post the event of a button up or down
        ES_Event_t ThisEvent;
        if (CurrentButton3Status == 1) { // Button is down
            ThisEvent.EventType = EV_BUTTON3_DOWN;
            PostButton3DebouncerSM(ThisEvent);
        } else { // button is now up
            ThisEvent.EventType = EV_BUTTON3_UP;
            PostButton3DebouncerSM(ThisEvent);
        }
        
        // update the last button state
        Button3Status = CurrentButton3Status;
    }
    return ReturnVal;
}