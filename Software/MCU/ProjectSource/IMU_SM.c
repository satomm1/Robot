/****************************************************************************
 Module
   IMU_SM.c

 Description
   This is a file for implementing the IMU

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
#include "IMU_SM.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static ImuState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitImuSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitImuSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set SPI1 Pins to correct input or output setting
  TRISDCLR = _TRISD_TRISD1_MASK | _TRISD_TRISD3_MASK | _TRISD_TRISD4_MASK; // Set SCK1, SS1, SDO1 to output
  TRISDSET = _TRISD_TRISD2_MASK; // Set SDI1 to Input
  
  // Map SPI1 Pins to correct function
  // RD1 is mapped to CLK1 by default
  RPD4R = 0b0101; // Map RD4 -> SS1
  RPD3R = 0b0101; // Map RD3 -> SDO1
  SDI1R = 0b0000; // Map SDI1 -> RD2
  
  // Initialize SPI1
  SPI1CON = 0; // Reset SPI1CON settings
  SPI1CONbits.FRMEN = 0; // Disable framed SPI support
  SPI1CONbits.FRMPOL = 0; // SS1 is active low
  SPI1CONbits.MSSEN = 1; // SS is automatically driven
  SPI1CONbits.MCLKSEL = 0; // Use PBCLK2 for the Baud Rate Generator
  PB2DIVbits.PBDIV = 4; // Reduce PBCLK2 to 50 MHz
  SPI1CONbits.ENHBUF = 0; // Enhance buffer disabled
  SPI1CONbits.DISSDO = 0; // SDO1 is used by the module
  SPI1CONbits.MODE32 = 0; // 8 bit mode
  SPI1CONbits.MODE16 = 0; // 8 bit mode
  SPI1CONbits.SMP = 0; // Data sampled at middle of data output time
  SPI1CONbits.CKE = 0; // Output data changes on transition from idle to active clock state
  SPI1CONbits.CKP = 1; // Idle state for the clock is high level
  SPI1CONbits.MSTEN = 1; // Host mode
  SPI1CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI1CONbits.STXISEL = 0b00; // Interrupt generated when last transfer shifted out of SPISR and transmit operations are complete
  SPI1CONbits.SRXISEL = 0b00; // Interrupt generated when the last word in the receive buffer is read (buffer is empty))

  SPI1CON2 = 0; // Reset SPI1CON2 register settings
  SPI1CON2bits.AUDEN = 0; // Audio protocol is disabled
  
  SPI1BRG = 15; // 1.56 MHz clock frequency
  
  SPI1CONbits.ON = 1; // Finally turn SPI1 on
  
  // Now need to write to IMU to set up...
  
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_IMU;
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
     PostImuSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostImuSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunImuSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
****************************************************************************/
ES_Event_t RunImuSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_IMU:       
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = IMU1;
      }
    }
    break;

    case IMU1:      
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  
        { 
          CurrentState = IMU1;  
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
     QueryImuSM

 Parameters
     None

 Returns
     ImuState_t The current state of the Imu state machine

 Description
     returns the current state of the Imu state machine
****************************************************************************/
ImuState_t QueryImuSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

