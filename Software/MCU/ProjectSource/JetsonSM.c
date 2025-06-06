/****************************************************************************
 Module
   Jetson_SM.c

 Description
   This is a file for implementing the Jetson state machine. This code 
   facilitates communications between the MCU and the Jetson via SPI.
   This code provides a state machine. When receiving the correct 
   initialization message, the state machine moves to a pending state.
   When receiving a confirmation message, the state machine moves to the 
   active state. In the active state the MCU receives desired velocities,
   while the MCU sends dead reckoning information and other sensor info.
   If the MCU does not receive a valid message in a set period of time, or
   the Jetson sends a shutdown message, the state machine moves to the inactive
   state.

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
#include "IMU_SM.h"
#include "ReflectService.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define JETSON_TIMEOUT 1000 // Timeout where no SPI response disconnects us
#define PENDING_TIMEOUT 1000 // Timeout to receive confirmation that Jetson recieved our confirmation message
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

static uint8_t ReceiveBuffer[2][16];
static uint8_t MessageToSend[16];

// Indicates what message we are currently sending from MCU to Jetson
static uint8_t CurrentMessage; 

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitJetsonSM

 Parameters
     uint8_t : the priority of this service

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
  SPI2CONbits.SSEN = 1; // SSx pin is used for Client mode
  SPI2CONbits.MSTEN = 0; // Client mode
  SPI2CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI2CONbits.STXISEL = 0b11; // Interrupt is generated when the buffer is not full
  SPI2CONbits.SRXISEL = 0b01; // Interrupt is generated when the receive buffer is not empty 

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
  IPC36bits.SPI2TXIS = 2;
  IPC35bits.SPI2RXIS = 2;
  
  // Clear interrupt flags
  IFS4CLR = _IFS4_SPI2TXIF_MASK | _IFS4_SPI2RXIF_MASK;
  
  // Local enable interrupts
  IEC4SET = _IEC4_SPI2RXIE_MASK; // | _IEC4_SPI2TXIE_MASK; 
  
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
        
        CurrentMessage = 0; // Start with message 0
        
        // Preload buffer with 0
        SPI2BUF = 0;
      }
    }
    break;

    case RobotInactive:      
    {
      switch (ThisEvent.EventType)
      {
        case EV_JETSON_MESSAGE_RECEIVED:  
        { 
          if (ReceiveBuffer[ThisEvent.EventParam][1] == 0b11111111 && ReceiveBuffer[ThisEvent.EventParam][0] == 90) {

            // Send message received message to Jetson
            MessageToSend[0] = 0;
            MessageToSend[1] = 0b11111111;
            MessageToSend[2] = 0; 
            MessageToSend[3] = ROBOT_ID; // Send Robot ID
            for (uint8_t ii = 4; ii < 16; ii++) {
                MessageToSend[ii] = 0; // Fill rest of buffer with 0's
            }
            
            // Start pending timeout timer
            ES_Timer_InitTimer(JETSON_TIMER, PENDING_TIMEOUT);
            
            CurrentState = RobotPending;  
            DB_printf("Moving to RobotPending\r\n");
          } else {
              for (uint8_t ii = 0; ii < 16; ii++) {
                MessageToSend[ii] = 0; // Fill rest of buffer with 0's
            }
          }
        }
        break;

        default:
          ;
      } 
    }
    break;
    
    case RobotPending:
    {
      switch (ThisEvent.EventType)
      {
        case EV_JETSON_MESSAGE_RECEIVED:  
        { 
          if (ReceiveBuffer[ThisEvent.EventParam][0] == 90 && ReceiveBuffer[ThisEvent.EventParam][1] == 0b10101010) {
            // We received confirmation that the message was received
            
            // Get starting position(s)
            float x_pos;
            uint32_t combined_bytes = ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][2] << 24) | 
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][3] << 16) |
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][4] << 8) |
                        ReceiveBuffer[ThisEvent.EventParam][5];
            *((uint32_t*)&x_pos) = combined_bytes;
            
            float y_pos;
            combined_bytes = ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][6] << 24) | 
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][7] << 16) |
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][8] << 8) |
                        ReceiveBuffer[ThisEvent.EventParam][9];
            *((uint32_t*)&y_pos) = combined_bytes;
            
            float theta_pos;
            combined_bytes = ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][10] << 24) | 
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][11] << 16) |
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][12] << 8) |
                        ReceiveBuffer[ThisEvent.EventParam][13];
            *((uint32_t*)&theta_pos) = combined_bytes;
            
            DB_printf("x: %d\n", (uint32_t)(x_pos*100));
            DB_printf("y: %d\n", (uint32_t)(y_pos*100));
            DB_printf("th: %d\n", (uint32_t)(theta_pos*100));
            
            // Use provided position to set the initial dead reckoning positions
            SetPosition(x_pos, y_pos, theta_pos);
            
            YELLOW_LATCH = 0; // Turn yellow LED off
            GREEN_LATCH = 1; // Turn green LED on
            ES_Timer_InitTimer(JETSON_TIMER, JETSON_TIMEOUT); // Start timeout timer
            
            CurrentState = RobotActive;  
            DB_printf("Moving to RobotActive\r\n");
          } else {
              for (uint8_t ii = 0; ii < 16; ii++) {
                  MessageToSend[ii] = 0;
              }
          }
        }
        break;
        
        case ES_TIMEOUT:
        {
            // Didn't receive confirmation in time
            CurrentState = RobotInactive;
            DB_printf("Moving to RobotInactive");
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
            case 90: // Operations Message
            {
                if (ReceiveBuffer[ThisEvent.EventParam][1] == 0b11110000) {
                    // Received Shutdown message
                    SetDesiredRPM(0, 0); // Stop all movement of the robot
                    ES_Timer_StopTimer(JETSON_TIMER); // Stop timer

                    YELLOW_LATCH = 1; // Turn yellow LED on
                    GREEN_LATCH = 0;  // Turn green LED off
                    CurrentState = RobotInactive;  

                    CurrentMessage = 0;
                    ResetPosition();
                    DB_printf("Received End Message: going to RobotInactive\r\n");
                }
            }
            break;
            
            case 45: // Velocity Message
            {
                float desired_lin_v;
                float desired_ang_v;

                ES_Timer_InitTimer(JETSON_TIMER, JETSON_TIMEOUT); // Restart timeout timer
                switch (CurrentMessage)
                {
                    case 0:
                    {
                        // Write the cliff sensor/button data
                        WriteCliffToSPI(MessageToSend);
                        CurrentMessage = 1;
                    }
                    break;

                    case 1:
                    {
                        // Write the IMU data
                        WriteImuToSPI(MessageToSend);
                        CurrentMessage = 2;
                    }
                    break;

                    case 2:
                    {
                        // Write the position as determined by dead reckoning
                        WritePositionToSPI(MessageToSend);
                        CurrentMessage = 3;
                    }
                    break;

                    case 3:
                    {
                        // Write the velocity as determined by dead reckoning
                        WriteDeadReckoningVelocityToSPI(MessageToSend);
                        CurrentMessage = 0;
                    }
                    break;
                }


                // Convert Received Data to float
                uint32_t combined_bytes = ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][1] << 24) | 
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][2] << 16) |
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][3] << 8) |
                        ReceiveBuffer[ThisEvent.EventParam][4];
                *((uint32_t*)&desired_lin_v) = combined_bytes;
                // DB_printf("%d, ", (int)(desired_lin_v));

                combined_bytes = ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][5] << 24) | 
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][6] << 16) |
                        ((uint32_t)ReceiveBuffer[ThisEvent.EventParam][7] << 8) |
                        ReceiveBuffer[ThisEvent.EventParam][8];
                *((uint32_t*)&desired_ang_v) = combined_bytes;

                // DB_printf("%d \r\n", (int)combined_bytes);
                SetDesiredSpeed(desired_lin_v, desired_ang_v);
            }
            break;
            
            default:
              ;  
          }       
        }
        break;
        
        case ES_TIMEOUT:
        {
          // Timed out: Shutdown the robot
          SetDesiredRPM(0, 0); // Stop all movement of the robot
            
          YELLOW_LATCH = 1; // Turn yellow LED on
          GREEN_LATCH = 0;  // Turn green LED off
          CurrentState = RobotInactive;
                    
          DB_printf("Timed out, moving to Robot Inactive\r\n");
          
          CurrentMessage = 0;
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
    
    // Turn the interrupt off
    IEC4CLR = _IEC4_SPI2TXIE_MASK;
    // Clear the interrupt 
    IFS4CLR = _IFS4_SPI2TXIF_MASK;
}


/****************************************************************************
 Function
    SPI2RX

 Description
   Creates an event when a SPI message is received
 * 
 * We are expecting incoming messages to come in the following form. Each 
 * message will be 16 8-bit words:
 *  - Message 1: Header message indicating start of sequence and containing the desired velocities
 *  - Message 2: Message so that IMU data can be passed
 *  - Message 3: Message so that position can be passed
****************************************************************************/
void __ISR(_SPI2_RX_VECTOR, IPL7SRS) SPI2RXHandler(void)
{
    static bool InMessage = false;
    
    // Static for speed
    static ES_Event_t ReceiveEvent = {EV_JETSON_MESSAGE_RECEIVED, 0};
    static uint8_t buffer_num = 0;
    static uint8_t MessageIndex = 0;
    static uint8_t TempData;
    
    if (InMessage) {
        while (!SPI2STATbits.SPIRBE && MessageIndex <= 15) {
            ReceiveBuffer[buffer_num][MessageIndex] = SPI2BUF;
            MessageIndex += 1;
        }
    } else {
        TempData = SPI2BUF;
        if (TempData == 55) {
            while (!SPI2STATbits.SPIRBE) {
                TempData = SPI2BUF;
            }

            for (uint8_t ii = 0; ii < 16; ii++) {
                SPI2BUF = MessageToSend[ii];
            }
            InMessage = true; // Now we are accepting message bytes
        }
    }
    
    IFS4CLR = _IFS4_SPI2RXIF_MASK; // Clear the interrupt
    
    if (MessageIndex == 16) {
        MessageIndex = 0; // Reset Message index once full
        InMessage = false; // No longer accepting message bytes
        
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
    
    // For debugging:
//    DB_printf("Received 16 Messages!\r\n");
    // Read the data from the buffer
//    for (uint8_t i=0; i < 16; i++) {
//        ReceiveBuffer[buffer_num][i] = SPI2BUF;
//        DB_printf("%d, ", ReceiveBuffer[buffer_num][i]);
//    }
//    DB_printf("\r\n");    
}
