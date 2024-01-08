/****************************************************************************
 Module
   Jetson_SM.c

 Description
   This is a file for implementing the Jetson

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
#include <sys/attribs.h>
#include "JetsonSM.h"
#include "MotorSM.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define JETSON_TIMEOUT 10000 // Timeout where no SPI response disconnects us
#define YELLOW_LATCH LATJbits.LATJ4
#define GREEN_LATCH LATJbits.LATJ5
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static JetsonState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static uint8_t ReceiveBuffer[2][8];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitJetsonSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitJetsonSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set the indicator LED pins to correct setting and start with yellow
  TRISJCLR = _TRISJ_TRISJ4_MASK | _TRISJ_TRISJ5_MASK;
  YELLOW_LATCH = 1; // Turn Yellow Light On
  GREEN_LATCH = 0; // Turn Green Light Off
  
  // Set SPI2 Pins to correct input or output setting  
  ANSELGCLR = _ANSELG_ANSG6_MASK | _ANSELG_ANSG7_MASK | _ANSELG_ANSG8_MASK | _ANSELG_ANSG9_MASK; 
  TRISGCLR = _TRISG_TRISG8_MASK;
  TRISGSET = _TRISG_TRISG6_MASK | _TRISG_TRISG7_MASK | _TRISG_TRISG9_MASK;
  
  // Map SPI2 Pins to correct function
  // RD1 is mapped to CLK1 by default
  RPG8R = 0b0110; // Map RG8 -> SDO2
  SS2R = 0b0001; // Map SS2 -> RG9
  SDI2R = 0b0001; // Map SDI2 -> RG7
  
  // Initialize SPI2
  SPI2CON = 0; // Reset SPI1CON settings
  SPI2CONbits.FRMEN = 0; // Disable framed SPI support
  SPI2CONbits.FRMPOL = 0; // SS1 is active low
  SPI2CONbits.MCLKSEL = 0; // Use PBCLK2 for the Baud Rate Generator (50 MHz)
  SPI2CONbits.ENHBUF = 1; // Enhance buffer enabled
  SPI2CONbits.DISSDO = 0; // SDO1 is used by the module
  SPI2CONbits.MODE32 = 0; // 8 bit mode
  SPI2CONbits.MODE16 = 0; // 8 bit mode
  SPI2CONbits.SMP = 0; // Data sampled at middle of data output time
  SPI2CONbits.CKE = 0; // Output data changes on transition from idle to active clock state
  SPI2CONbits.CKP = 1; // Idle state for the clock is high level
  SPI2CONbits.MSTEN = 0; // Client mode
  SPI2CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI2CONbits.STXISEL = 0b01; // Interrupt is generated when the buffer is completely empty
  SPI2CONbits.SRXISEL = 0b10; // Interrupt is generated when the receive buffer is full by one-half or more (8)

  SPI2CON2 = 0; // Reset SPI2CON2 register settings
  SPI2CON2bits.AUDEN = 0; // Audio protocol is disabled
  
  while (!SPI2STATbits.SPIRBE){
      uint8_t ClearData = SPI2BUF;
  }
  SPI2STATbits.SPIROV = 0; // Clear the Receive overflow bit
  
  // Don't need to set BRG since acting in client mode
  
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC36bits.SPI2TXIP = 7; // SPI2TX
  IPC35bits.SPI2RXIP = 7; // SPI2RX
  
  // Clear interrupt flags
  IFS4CLR = _IFS4_SPI2TXIF_MASK | _IFS4_SPI2RXIF_MASK;
  
  // Local enable interrupts
  IEC4SET = _IEC4_SPI2TXIE_MASK | _IEC4_SPI2RXIE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
  SPI2CONbits.ON = 1; // Finally turn SPI2 on
  
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_Jetson;
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
     PostJetsonSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostJetsonSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunJetsonSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
****************************************************************************/
ES_Event_t RunJetsonSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_Jetson:       
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        // now put the machine into the actual initial state
        CurrentState = RobotInactive;
      }
    }
    break;

    case RobotInactive:      
    {
      switch (ThisEvent.EventType)
      {
        case EV_JETSON_MESSAGE_RECEIVED:  
        { 
          if (ReceiveBuffer[ThisEvent.EventParam][0] == 0b11111111) {
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b11111111;
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b00000000;
            SPI2BUF = 0b00000000;
            
            YELLOW_LATCH = 0; // Turn yellow LED off
            GREEN_LATCH = 1; // Turn green LED on
            ES_Timer_InitTimer(JETSON_TIMER, JETSON_TIMEOUT); // Start timeout timer
            
            CurrentState = RobotActive;  
          }
        }
        break;

        default:
          ;
      } 
    }
    break;
    
    case RobotActive:      
    {
      switch (ThisEvent.EventType)
      {
        case EV_JETSON_MESSAGE_RECEIVED:  
        { 
          // Determine what message type we have
          switch (ReceiveBuffer[ThisEvent.EventParam][0])
          {
            case 0b00000001: // Received Odom Request message
            {
              // Nothing to do, just reset timeout timer
              ES_Timer_InitTimer(JETSON_TIMER, JETSON_TIMEOUT);
            }
            break;
            
            case 0b00000010: // Received Velocity Update message
            {
              // TODO: Translate message to linear/angular velocity values
              float lin_vel = 0.;
              float ang_vel = 0.;
              SetDesiredSpeed(lin_vel,ang_vel);
            }
            break;
              
            case 0b10101010: // Received message to shutdown
            {
              SetDesiredRPM(0, 0); // Stop all movement of the robot
              ES_Timer_StopTimer(JETSON_TIMER);
            
              YELLOW_LATCH = 1; // Turn yellow LED on
              GREEN_LATCH = 0;  // Turn green LED off
              CurrentState = RobotInactive;      
            }
            break;
            
            default:
              ;  
          }       
        }
        break;

        case EV_JETSON_TRANSFER_COMPLETE:  
        { 
          // TODO: Get Odometry Update and put in TX Buffer
        }
        break;
        
        case ES_TIMEOUT:
        {
          // Timed out: Shutdown the robot
          SetDesiredRPM(0, 0); // Stop all movement of the robot
            
          YELLOW_LATCH = 1; // Turn yellow LED on
          GREEN_LATCH = 0;  // Turn green LED off
          CurrentState = RobotInactive;
        }
            
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
     QueryJetsonSM

 Parameters
     None

 Returns
     JetsonState_t The current state of the Jetson state machine

 Description
     returns the current state of the Jetson state machine
****************************************************************************/
JetsonState_t QueryJetsonSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
    SPI2TX

 Description
    Creates an event that indicates we need to fill TX buffer again
****************************************************************************/
void __ISR(_SPI2_TX_VECTOR, IPL7SRS) SPI2TXHandler(void)
{
    // Static for speed
    static ES_Event_t TransferEvent = {EV_JETSON_TRANSFER_COMPLETE, 0};
    
    // Clear the interrupt
    IFS4CLR = _IFS4_SPI2TXIF_MASK;
    
    // Tell state machine the transfer buffer is empty
    PostJetsonSM(TransferEvent);
}


/****************************************************************************
 Function
    SPI2RX

 Description
   Creates an event when a SPI message is received
****************************************************************************/
void __ISR(_SPI2_RX_VECTOR, IPL7SRS) SPI2RXHandler(void)
{
    // Static for speed
    static ES_Event_t ReceiveEvent = {EV_JETSON_MESSAGE_RECEIVED, 0};
    static uint8_t buffer_num = 0;
    
    // Clear the interrupt
    IFS4CLR = _IFS4_SPI2RXIF_MASK; 
    
    for (uint8_t i=0; i < 8; i++) {
        ReceiveBuffer[buffer_num][i] = SPI2BUF;
    }
   
    // Tell which buffer we just stored the data in
    ReceiveEvent.EventParam = buffer_num;
    if (buffer_num) {
        buffer_num = 0;
    } else {
        buffer_num = 1;
    }
    
    // Tell state machine the data is ready
    PostJetsonSM(ReceiveEvent);
}
