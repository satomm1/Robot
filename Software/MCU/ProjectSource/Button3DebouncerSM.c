/****************************************************************************
 Module
   Button3DebouncerSM.c

 Revision
   1.0.1

 Description
   Implements the button flat state machine. Provides software debounce for the
   button

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "Button3DebouncerSM.h"
#include "EventCheckers.h"
#include "dbprintf.h"
/*----------------------------- Module Defines ----------------------------*/
#define DEBOUNCE_TIME 50
//#define DEBUG
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static Button3DebouncerState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitButton3DebouncerSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

****************************************************************************/
bool InitButton3DebouncerSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_Button3Debouncer;
    
  InitButton3(); // Init the button event checker
  
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostButton3DebouncerSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

****************************************************************************/
bool PostButton3DebouncerSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunButton3DebouncerSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
****************************************************************************/
ES_Event_t RunButton3DebouncerSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_Button3Debouncer:
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        CurrentState = Button3DebouncingWait;
      }
    }
    break;

    case Button3DebouncingWait: 
    {
        switch (ThisEvent.EventType)
        {
            case EV_BUTTON3_DOWN: 
            {  
                ES_Timer_InitTimer(BUTTON3_TIMER, DEBOUNCE_TIME);           
                CurrentState = Button3DebouncingFall; 
            }
            break;
        
            case EV_BUTTON3_UP:
            {
                ES_Timer_InitTimer(BUTTON3_TIMER, DEBOUNCE_TIME);           
                CurrentState = Button3DebouncingRise; 
            }
            break;
        
            default:
            ;
        }
    }
    break;
    
    case Button3DebouncingFall:
    {
        switch (ThisEvent.EventType)
        {
            case EV_BUTTON3_UP:
            {       
                CurrentState = Button3DebouncingWait; 
            }
            break;
            
            case ES_TIMEOUT:
            {
                ES_Event_t NewEvent = {EV_BUTTON3_PRESSED, 0};           
                // TODO: Post to proper SM                 
                CurrentState = Button3DebouncingWait; 
                
                #ifdef DEBUG
                DB_printf("Button 3 Pressed\r\n");
                #endif
            }
            break;
        }
    }
    break;
    
    case Button3DebouncingRise:
    {
        switch (ThisEvent.EventType)
        {
            case EV_BUTTON3_DOWN:
            {       
                CurrentState = Button3DebouncingWait; 
            }
            break;
            
            case ES_TIMEOUT:
            {
                ES_Event_t NewEvent = {EV_BUTTON3_RELEASED, 0};           
                // TODO: Post to proper SM         
                CurrentState = Button3DebouncingWait; 
                
                #ifdef DEBUG
                DB_printf("Button 3 Released\r\n");
                #endif
            }
            break;
        }
    }
    break;
    
    default:
      ;
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryButton3DebouncerSM

 Parameters
     None

 Returns
     Button3DebouncerState_t The current state of the Button3Debouncer state machine

 Description
     returns the current state of the Button3Debouncer state machine
 Notes

****************************************************************************/
Button3DebouncerState_t QueryButton3DebouncerSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

