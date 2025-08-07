/****************************************************************************
 Module
   SDCardSM.c

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
#include "SDCardSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define SD_CS_PIN LATDbits.LATD14
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
bool InitSD(void);
uint8_t SPI_Transfer_Blocking(uint8_t data);
uint8_t SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static SDCardState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSDCardSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitSDCardSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set SPI6 pins to input/output and digital
  TRISDCLR = _TRISD_TRISD14_MASK | _TRISD_TRISD15_MASK; // SCK and SS to outputs
  TRISECLR = _TRISE_TRISE9_MASK; // SDO6 to output
  TRISESET = _TRISE_TRISE8_MASK; // SDI6 to input
  
  ANSELDCLR = _ANSELD_ANSD14_MASK | _ANSELD_ANSD15_MASK; // Digital
  ANSELECLR = _ANSELE_ANSE8_MASK | _ANSELE_ANSE9_MASK; // Digital
  
  // Map SPI6 functions
  SDI6R = 0b1101;  // Map to RE8
  RPE9R = 0b1010; // Map to SDO6
  RPD14R = 0b1010; // Map to SS6
  
  // Setup SPI6 for writing/reading to SD Card
  SPI6CONbits.ON = 0; // Turn off
  SPI6CONbits.FRMPOL = 0; // SS Active low
  SPI6CONbits.MSSEN = 0; // SS not driven automatically
  SPI6CONbits.MCLKSEL = 0; // Use PBCLK2
  SPI6CONbits.ENHBUF = 1; // Use enhanced buffer
  SPI6CONbits.MODE32 = 0;
  SPI6CONbits.MODE16 = 0; // 8 bit mode
  SPI6CONbits.SMP = 0; // Input data sampled at middle of data output time
  SPI6CONbits.CKE = 1; // output data changes on transition from active clock state to Idle clock state
  SPI6CONbits.CKP = 0; // Clock idle low
  SPI6CONbits.MSTEN = 1; // Host mode
  
  SPI6STATbits.SPIROV = 0;
  
  SPI6CON2 = 0;
  SPI6BRG = 124; // 200 kHz
  
  SPI6CONbits.ON = 1; // Turn SPI6 on
  
  // Initialize SD Card
  InitSD();
  
//  SPI6CONbits.MSSEN = 1; // Now set SS to be driven automatically
  
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_SD;
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
     PostSDCardSM

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
bool PostSDCardSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSDCardSM

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
ES_Event_t RunSDCardSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_SD:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = SDReset;
      }
    }
    break;

    case SDReset:        // If current state is state one
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  //If event is event one

        {   // Execute action function for state one : event one
          CurrentState = SDReset;  //Decide what the next state will be
        }
        break;

        // repeat cases as required for relevant events
        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QuerySDCardSM

 Parameters
     None

 Returns
     SDCardState_t The current state of the SDCard state machine

 Description
     returns the current state of the SDCard state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
SDCardState_t QuerySDCardSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

bool InitSD(void) {
  uint16_t timeout;
  uint8_t response;
    
  // Apply at least 74 clock pulses with CS high
  SD_CS_PIN = 1; // Deselect card
  for(int i = 0; i < 10; i++) {
      SPI_Transfer_Blocking(0xFF);
  }
  
  // 2. Send CMD0 (GO_IDLE_STATE) to reset and enter SPI mode
  timeout = 100;
  do {
    response = SendCommand(0, 0, 0x95);  // CMD0 with valid CRC
    timeout--;
  } while(response != 0x01 && timeout > 0);
  
  if (timeout == 0) {
    return false;  // Timeout error   
  }
  
  // 3. Send CMD8 to check card version (needed for SDv2)
  response = SendCommand(8, 0x000001AA, 0x87);  // CMD8 with test pattern

  bool isSDv2 = false;
  if(response == 0x01 || response == 0x05) {
    // Card is SDv2, check echo-back of test pattern
    isSDv2 = true;
    uint8_t r7[4];
    for(int i = 0; i < 4; i++) {
      r7[i] = SPI_Transfer_Blocking(0xFF);
    }
    if (r7[3] != 0xAA) {
      return false;  // Pattern mismatch
    }
  }
  
  // 4. Send ACMD41 until card is initialized
  timeout = 1000;
  do {
    // Send CMD55 before ACMD41
    SendCommand(55, 0, 0x01);
        
    // For SDv2, use HCS bit in argument
    uint32_t arg = isSDv2 ? 0x40000000 : 0;
    response = SendCommand(41, arg, 0x01);
        
    timeout--;
    if(timeout == 0) { 
      return false;  // Timeout error  
    }
  } while(response != 0x00);
  
  // 5. For SDv2, check if card is high capacity
  bool isHighCapacity = false;
  if(isSDv2) {
    response = SendCommand(58, 0, 0x01);  // Read OCR
    if(response == 0x00) {
      uint8_t ocr[4];
      for(int i = 0; i < 4; i++) {
        ocr[i] = SPI_Transfer_Blocking(0xFF);
      }
      isHighCapacity = (ocr[0] & 0x40) ? true : false;
    }
    SD_CS_PIN = 1;  // Deselect card
  }
  
  // 6. Set block size to 512 bytes (not needed for SDHC/SDXC)
  if(!isHighCapacity) {
    response = SendCommand(16, 512, 0x01);
    if(response != 0x00) { 
      return false;
    }
  }
  
  // 7. Optional: Switch to higher SPI speed now that init is complete
  SPI6CONbits.ON = 0;      // Disable SPI
  SPI6BRG = 1;             // 12.5 MHz
  SPI6CONbits.ON = 1;      // Re-enable SPI
  
}

// Basic SPI transfer function
uint8_t SPI_Transfer_Blocking(uint8_t data) {
  SPI6BUF = data;               // Write data to buffer
  while(!SPI6STATbits.SPIRBF);  // Wait for transfer to complete
  return SPI6BUF;               // Return received data
}

uint8_t SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
  SD_CS_PIN = 0;           // Select card
    
  SPI_Transfer_Blocking(0xFF);      // Dummy byte
  SPI_Transfer_Blocking(cmd | 0x40);// Command byte (always OR with 0x40)
  SPI_Transfer_Blocking(arg >> 24); // Argument: MSB first
  SPI_Transfer_Blocking(arg >> 16);
  SPI_Transfer_Blocking(arg >> 8);
  SPI_Transfer_Blocking(arg);
  SPI_Transfer_Blocking(crc);       // CRC byte
    
  uint8_t response;
  // Wait for response (skip stuff byte for some commands)
  int n = 10;
  do {
    response = SPI_Transfer_Blocking(0xFF);
    n--;
  } while(response == 0xFF && n > 0);
    
  return response;
}