/****************************************************************************
 Module
   UsbService.c

 Revision
   1.0.1

 Description
   This is the USB service used for serial output to a terminal

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// This module
#include "UsbService.h"


// Hardware
#include <xc.h>
//#include <proc/p32mx170f256b.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include "MotorSM.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 10.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

#define ENTER_POST     ((MyPriority<<3)|0)
#define ENTER_RUN      ((MyPriority<<3)|1)
#define ENTER_TIMEOUT  ((MyPriority<<3)|2)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event_t DeferralQueue[3 + 1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitUsbService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
****************************************************************************/
bool InitUsbService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Set the USB_RST as an output and set high
  TRISKCLR = _TRISK_TRISK4_MASK;
  LATKbits.LATK4 = 1;

  // When doing testing, it is useful to announce just which program
  // is running.
  clrScrn();
  puts("\rSerial Output for MattBot Control Board \r");
  DB_printf( "compiled at %s on %s\n", __TIME__, __DATE__);
  DB_printf( "\n\r\n");
  DB_printf( "Press any key to post key-stroke events\n\r");
  DB_printf( "Press 'xxx' to test xxx \n\r");

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
     PostUsbService

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostUsbService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunUsbService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunUsbService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (ThisEvent.EventType)
  {
    case ES_NEW_KEY:   // announce
    {
      DB_printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);
      if ('a' == ThisEvent.EventParam)
      {
          SPI1BUF = 0b10000000 | 0x4F;
          while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during testing
            }
            uint8_t temp = SPI1BUF;
            DB_printf("Received: %d\r\n", temp);
      }
      
      if ('b' == ThisEvent.EventParam) {
          OC1RS = (312 + 1)/100 * 85;
          OC2RS = (312 + 1)/100 * 85;
          LATFbits.LATF8 = 1; 
          LATJbits.LATJ3 = 1; 
      }
      
      if ('f' == ThisEvent.EventParam) {
          LATFbits.LATF8 = 0; 
          LATJbits.LATJ3 = 0; 
          OC1RS = (312 + 1)/100 * 15;
          OC2RS = (312 + 1)/100 * 15;
      }
      
      if ('s' == ThisEvent.EventParam) {
          SetDesiredSpeed(0,0);
          
          OC1RS = 0;
          OC2RS = 0;
          LATFbits.LATF8 = 0; 
          LATJbits.LATJ3 = 0; 
      }
      
      if ('1' == ThisEvent.EventParam) {
          SetDesiredRPM(45, 45);
      }
      
      if ('2' == ThisEvent.EventParam) {
          SetDesiredRPM(55, 55);
      }
      
      if ('3' == ThisEvent.EventParam) {
          SetDesiredRPM(65, 65);
      }
      
      if ('4' == ThisEvent.EventParam) {
          SetDesiredRPM(150, 150);
      }
      
      if ('5' == ThisEvent.EventParam) {
          SetDesiredSpeed(0.5, 0);
      }
      
      if ('6' == ThisEvent.EventParam) {
          SetDesiredSpeed(-0.1, 0);
      }
      
      if ('c' == ThisEvent.EventParam) {
          SetDesiredSpeed(0, 1);
      }
      
      if ('w' == ThisEvent.EventParam) {
          SetDesiredSpeed(0, -1);
      }
      
    }
    break;
    
    case ES_TIMEOUT:
    {
        OC1RS = 0;
        OC2RS = 0;
        LATFbits.LATF8 = 0; 
        LATJbits.LATJ3 = 0; 
    }
    break;
    
    default:
    {}
     break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
