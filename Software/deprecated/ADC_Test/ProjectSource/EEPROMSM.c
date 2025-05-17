/****************************************************************************
 Module
   EEPROMFSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "EEPROMSM.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitEEPROM(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static EEPROMState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitEEPROMFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
    Matthew M Sato
****************************************************************************/
bool InitEEPROMSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Set up SPI5
  
  // Set SCK5 and SS5 as Digital outputs
  TRISFCLR = _TRISF_TRISF12_MASK | _TRISF_TRISF13_MASK;
  ANSELFCLR = _ANSELF_ANSF12_MASK | _ANSELF_ANSF13_MASK;
  
  // Make proper mappings
  TRISGCLR = _TRISG_TRISG0_MASK; // Set SDO5 as digital output
  TRISGSET = _TRISG_TRISG1_MASK; // Set SDI5 as digital input
  
  SDI5R = 0b1100; // SDI5 -> RG1
  RPG0R = 0b1001; // RG0 -> SDO5
  RPF12R = 0b11001; // RF12 -> SS5
  
  SPI5CON = 0;
  SPI5CONbits.MSSEN = 1; // SS automatically driven
  SPI5CONbits.MCLKSEL = 0; //  PBCLK2 used by BRG
  SPI5CONbits.ENHBUF = 1; // Enhanced buffer on
  SPI5CONbits.DISSDO = 0; // SDo5 controlled by the module
  SPI5CONbits.MODE32 = 0;
  SPI5CONbits.MODE16 = 1; // 16 bit mode
  SPI5CONbits.SMP = 0; // Data sampled at middle
  SPI5CONbits.CKE = 0; // Output data changes on transition from idle clock to active clock
  SPI5CONbits.CKP = 1; // Idle clock state is high
  SPI5CONbits.MSTEN = 1; // Host mode
  SPI5CONbits.DISSDI = 0; // SDI pin controlled by the module
  SPI5CONbits.STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
  SPI5CONbits.SRXISEL = 0b01; // Interrupt is generated when the buffer is not empty
  
  SPI5STATbits.SPIROV = 0; // Clear overflow bit
  
  SPI5CONbits.ON = 1; // Turn SPI module on
  
  // put us into the Initial PseudoState
  CurrentState = InitPState_EEPROM;
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
     PostEEPROMFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostEEPROMSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEEPROMFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunEEPROMSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_EEPROM: 
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
          InitEEPROM();

        // now put the machine into the actual initial state
        CurrentState = EEPROMWaiting;
      }
    }
    break;

    case EEPROMWaiting: 
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK: 
        { 
          CurrentState = EEPROMWriting;  
        }
        break;

        
        default:
          ;
      }  
    }
    break;
    
    case EEPROMWriting: 
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK: 
        { 
          CurrentState = EEPROMWriting;  
        }
        break;

        
        default:
          ;
      }  
    }
    break;
    
    case EEPROMReading: 
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK: 
        { 
          CurrentState = EEPROMWriting;  
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
     QueryEEPROMSM

 Parameters
     None

 Returns
     EEPROMState_t The current state of the EEPROM state machine

 Description
     returns the current state of the EEPROM state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
EEPROMState_t QueryEEPROMFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

// Performs SPI actions needed to set up EEPROM for writing/reading
void InitEEPROM(void) {
    
}