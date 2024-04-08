/****************************************************************************
 Module
   ReflectService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ReflectService.h"
#include "ADC_HAL.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint16_t ReflectiveResults[3];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitReflectService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitReflectService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Set sensors to be analog inputs
  TRISJSET = _TRISJ_TRISJ11_MASK;
  TRISBSET = _TRISB_TRISB4_MASK | _TRISB_TRISB11_MASK;
  
  ANSELJSET = _ANSELJ_ANSJ11_MASK;
  ANSELBSET = _ANSELB_ANSB4_MASK | _ANSELB_ANSB11_MASK;
  
  // Setup the ADC
  InitADC();
  ES_Timer_InitTimer(REFLECT_TIMER, 500); // Init timer to tell when to read
  
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
     PostReflectService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostReflectService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunReflectService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunReflectService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  if (ThisEvent.EventType == ES_TIMEOUT) {
      ReadADC(ReflectiveResults);
      
      DB_printf("Reflect 1: %d\r\n", ReflectiveResults[0]);
      DB_printf("Reflect 2: %d\r\n", ReflectiveResults[1]);
      DB_printf("Reflect 3: %d\r\n", ReflectiveResults[2]);
      
      ES_Timer_InitTimer(REFLECT_TIMER, 1000); // Init Timer to read again
  }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
void __ISR(_ADC_VECTOR, IPL4SRS) ADCHandler(void) {
    uint32_t status = ADCCON2;
    if ((status>>29) & 1) { 
        IFS1CLR = _IFS1_ADCIF_MASK; // Clear interrupt flag
        IEC1CLR = _IEC1_ADCIE_MASK; // Disable the interrupt
        
        ReflectiveResults[0] = ADCDATA6; // fetch the result
        ReflectiveResults[1] = ADCDATA37; // fetch the result
        ReflectiveResults[2] = ADCDATA4; // fetch the result
    } else {
        DB_printf("Some other ADC interrupt is active!\r\n");
    }
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

