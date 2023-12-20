/****************************************************************************
 Module
   MotorSM.c

 Description
   This is a file for implementing the Motors

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorSM.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static MotorState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitMotorSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set Motor Driving/Direction pins to outputs
  TRISFCLR = _TRISF_TRISF2_MASK | _TRISF_TRISF8_MASK;
  TRISDCLR = _TRISD_TRISD5_MASK;
  TRISJCLR = _TRISJ_TRISJ3_MASK;
  
  // Set encoder pins and fault pins to digital inputs
  ANSELCCLR = _ANSELC_ANSC1_MASK | _ANSELC_ANSC4_MASK;
  TRISCSET = _TRISC_TRISC1_MASK | _TRISC_TRISC4_MASK;
  TRISDSET = _TRISD_TRISD0_MASK;
  TRISHSET = _TRISH_TRISH8_MASK;
  TRISASET = _TRISA_TRISA4_MASK;
  TRISJSET = _TRISJ_TRISJ12_MASK;
  
  // Set motor current pins to be analog inputs
  ANSELJSET = _ANSELJ_ANSJ9_MASK;
  ANSELASET = _ANSELA_ANSA1_MASK;
  TRISJSET = _TRISJ_TRISJ9_MASK;
  TRISASET = _TRISA_TRISA1_MASK;
  
  // Setup Output compare...
  
  // Setup Input capture...
  
  // Setup Interrupts
  
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_Motor;
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
     PostMotorSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostMotorSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMotorSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
****************************************************************************/
ES_Event_t RunMotorSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_Motor:       
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = Motor1;
      }
    }
    break;

    case Motor1:      
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  
        { 
          CurrentState = Motor1;  
        }
        break;

        
        default:
          ;
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
     QueryMotorSM

 Parameters
     None

 Returns
     MotorState_t The current state of the Motor state machine

 Description
     returns the current state of the Motor state machine
****************************************************************************/
MotorState_t QueryMotorSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

