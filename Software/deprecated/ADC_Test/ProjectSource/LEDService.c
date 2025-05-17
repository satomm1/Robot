/****************************************************************************
 Module
   LEDService.c

 Revision
   1.0.1

 Description
   This is a file for implementing the LED service under the
   Gen2 Events and Services Framework.

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
#include "LEDService.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitLEDService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
****************************************************************************/
bool InitLEDService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Set LED pins to digital output
  TRISCCLR = _TRISC_TRISC12_MASK | _TRISC_TRISC15_MASK;
  TRISHCLR = _TRISH_TRISH4_MASK | _TRISH_TRISH5_MASK | _TRISH_TRISH6_MASK | _TRISH_TRISH7_MASK;
  
  ANSELHCLR = _ANSELH_ANSH4_MASK | _ANSELH_ANSH5_MASK | _ANSELH_ANSH6_MASK;
  
  // Start with all LEDs turned off
  LATHbits.LATH4 = 0; // LED2
  LATHbits.LATH5 = 0; // LED1
  LATHbits.LATH6 = 0; // LED4
  LATHbits.LATH7 = 0; // LED3
  LATCbits.LATC12 = 0; // LED6
  LATCbits.LATC15 = 0; // LED5
  
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
     PostLEDService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostLEDService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunLEDService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Turns LEDs on or off
****************************************************************************/
ES_Event_t RunLEDService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType) {
      
      // Turn a LED on
      case EV_LED_ON:
      {
          // Event Parameter is the LED to turn on
          switch (ThisEvent.EventParam) {
              case 1:
              {
                  LATHbits.LATH5 = 1;
              }
              break;
              
              case 2:
              {
                  LATHbits.LATH4 = 1;
              }
              break;
              
              case 3:
              {
                  LATHbits.LATH7 = 1;
              }
              break;
              
              case 4:
              {
                  LATHbits.LATH6 = 1;
              }
              break;
              
              case 5:
              {
                  LATCbits.LATC15 = 1;
              }
              break;
              
              case 6:
              {
                  LATCbits.LATC12 = 1;
              }
              break;
              
              default:
                break;
          }
      }
      break;
      
      // Turn a LED off
      case EV_LED_OFF:
      {
          // Event Parameter is the LED to turn off
          switch (ThisEvent.EventParam) {
              case 1:
              {
                  LATHbits.LATH5 = 0;
              }
              break;
              
              case 2:
              {
                  LATHbits.LATH4 = 0;
              }
              break;
              
              case 3:
              {
                  LATHbits.LATH7 = 0;
              }
              break;
              
              case 4:
              {
                  LATHbits.LATH6 = 0;
              }
              break;
              
              case 5:
              {
                  LATCbits.LATC15 = 0;
              }
              break;
              
              case 6:
              {
                  LATCbits.LATC12 = 0;
              }
              break;
              
              default:
                break;
          }
      }
      break;
      
      default:
        break;
  }
    
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

