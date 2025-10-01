/****************************************************************************
 Module
   EnvironmentSensorSM.c

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
#include <sys/attribs.h>
#include "EnvironmentSensorSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define SHT4x_ADD 0x44
#define SGP41_ADD 0x59
#define I2C_WRITE 0
#define I2C_READ 1

#define CRC8_POLY  0x31  // x^8 + x^5 + x^4 + 1
#define CRC8_INIT  0xFF
#define CRC8_XOROUT 0x00
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
bool I2C2_Busy(void);
bool Get_T_RH(bool heat, uint8_t *buf);
bool Get_AirQuality(uint8_t *buf);
uint8_t crc8_poly31_ff(const uint8_t *data, uint16_t len);
bool crc8_valid_residue(const uint8_t data[3]);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static EnvironmentSensorState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// Keep track of the I2C module
static volatile I2C2_Trans i2c2_t = { .stage = I2C_ST_IDLE };
static volatile uint8_t commands[16];

static uint8_t T_RH_Buffer[16];
static uint8_t AirQuality_Buffer[16];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitEnvironmentSensorSM

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
bool InitEnvironmentSensorSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  // Set SDA2/SCL2 Pins to Input
  TRISASET = _TRISA_TRISA2_MASK | _TRISA_TRISA3_MASK;
  
  I2C2CON = 0; // Reset the I2C register
  
  // Configure Baud: BRG = PBCLK * (1/(2*FSCK) - TPGD) - 2
  I2C2BRG = 243; // Set for 100 kHz
  
  I2C2STAT = 0; // Reset the status register
  
  ///////////// Setup the necessary I2C2 Interrupts ///////////////////////////
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  PRISSbits.PRI6SS = 0b0110; // Interrupt with a priority level of 6 uses Shadow Set 6
  
  // Set interrupt priorities
  IPC37bits.I2C2MIP = 7;
  IPC37bits.I2C2BIP = 7;
  IPC37bits.I2C2SIP = 7;
  
  // Clear interrupt flags
  IFS4CLR = _IFS4_I2C2MIF_MASK | _IFS4_I2C2BIF_MASK | _IFS4_I2C2SIF_MASK;
  
  // Local enable interrupts
  IEC4SET = _IEC4_I2C2MIE_MASK | _IEC4_I2C2BIE_MASK; // | _IEC4_I2C2SIE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  /////////////////////////////////////////////////////////////////////////////
    
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_Env;
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
     PostEnvironmentSensorSM

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
bool PostEnvironmentSensorSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEnvironmentSensorSM

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
ES_Event_t RunEnvironmentSensorSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_Env:
    {
      if (ThisEvent.EventType == ES_INIT)    
      {
        // now put the machine into the actual initial state
        CurrentState = Idle_Env_T_RH;
        
        // Turn I2C2 On to allow I2C operations
        I2C2CONbits.ON = 1;
        
        ES_Timer_InitTimer(ENV_TIMER, 1000);
      }
    }
    break;

    case Idle_Env_T_RH:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_TIMER) {
                CurrentState = T_RH_Meas; 
                Get_T_RH(true, T_RH_Buffer);
            }          
        }
        break;

        default:
          ;
      }  
    }
    break;
     
    case T_RH_Meas:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }
        }
        break;
        
        case EV_I2C_COMPLETE:
        {
            // TODO: Process the data
            
            ES_Timer_InitTimer(ENV_TIMER, 1000);
            CurrentState = Idle_Env_Air;
        }
        break;
        
        case EV_I2C_ERROR:
        {
            // We had an error: try getting the data again
            Get_T_RH(true, T_RH_Buffer);
        }
        break;

        default:
          ;
      }  
    }
    break;
    
    case Idle_Env_Air:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_TIMER) {
                CurrentState = Air_Meas; 
                Get_AirQuality(AirQuality_Buffer);
            }          
        }
        break;

        default:
          ;
      }  
    }
    break;
     
    case Air_Meas:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }
          
        }
        break;
        
        case EV_I2C_COMPLETE:
        {
            // TODO: Process the data
            
            ES_Timer_InitTimer(ENV_TIMER, 1000);
            CurrentState = Idle_Env_T_RH;
        }
        break;
        
        case EV_I2C_ERROR:
        {
            // We had an error: try getting the data again
            Get_AirQuality(AirQuality_Buffer);
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
     QueryEnvironmentSensorSM

 Parameters
     None

 Returns
     EnvironmentSensorState_t The current state of the EnvironmentSensor state machine

 Description
     returns the current state of the EnvironmentSensor state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
EnvironmentSensorState_t QueryEnvironmentSensorSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/
bool I2C2_Busy(void) {
    return i2c2_t.busy;
}

bool Get_T_RH(bool heat, uint8_t *buf) {
    // Check if I2C2 is busy or not ready yet
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    } else if (!I2C2STATbits.P) {
        return false;
    }
    
    i2c2_t.dev7 = SHT4x_ADD;
    
    // Command length = 1 byte, change command depending on heat or no heat
    i2c2_t.command_wait = true;
    i2c2_t.command_len = 1;
    if (heat) {
        commands[0] = 0x15;
        i2c2_t.command_wait_time = 105;
    } else {
        commands[0] = 0xFD;
        i2c2_t.command_wait_time = 50;
    }
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0;
    
    i2c2_t.buf = buf;
    i2c2_t.len = 6;
    i2c2_t.idx = 0;
    i2c2_t.busy = true;
    i2c2_t.stage = I2C_ST_START;
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool Get_AirQuality(uint8_t *buf) {
    // Check if I2C2 is busy or not ready yet
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    } else if (!I2C2STATbits.P) {
        return false;
    }
    
    i2c2_t.dev7 = SGP41_ADD;
    
    // Command length = 1 byte, change command depending on heat or no heat
    i2c2_t.command_wait = true;
    i2c2_t.command_len = 2;
    commands[0] = 0x26;
    commands[1] = 0x19;
    i2c2_t.command_wait_time = 55;
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0;
    
    i2c2_t.buf = buf;
    i2c2_t.len = 6;
    i2c2_t.idx = 0;
    i2c2_t.busy = true;
    i2c2_t.stage = I2C_ST_START;
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

uint8_t crc8_poly31_ff(const uint8_t *data, uint16_t len) {
    uint8_t crc = CRC8_INIT;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];              // combine next byte
        for (uint8_t j = 8; j > 0; --j) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            } else {
                crc = (crc << 1);
            }
        }
    }
    return (uint8_t)(crc ^ CRC8_XOROUT);
}

bool crc8_valid_residue(const uint8_t data[3]) {
    return (crc8_poly31_ff(data, 3) == 0x00);
}

/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/

void __ISR(_I2C2_BUS_VECTOR, IPL7SRS) BusHandler(void) {
    // Clear bus collision flag
    IFS4CLR = _IFS4_I2C2BIF_MASK;
     
    // Attempt recovery: Stop, mark error
    if (!I2C2CONbits.PEN) {
        I2C2CONbits.PEN = 1;
    }
    i2c2_t.busy = false;
    
    ES_Event_t new_event = {EV_I2C_ERROR, 0};
    PostEnvironmentSensorSM(new_event);
    
    i2c2_t.stage = I2C_ST_IDLE;
}

void __ISR(_I2C2_SLAVE_VECTOR, IPL7SRS) ClientHandler(void) {
    
}

void __ISR(_I2C2_MASTER_VECTOR, IPL7SRS) HostHandler(void) {
    // Clear the interrupt
    IFS4CLR = _IFS4_I2C2MIF_MASK;
    
    switch (i2c2_t.stage) {
        case I2C_ST_START:
            // Start complete --> send device addr (write)
            I2C2TRN = (i2c2_t.dev7 << 1) | I2C_WRITE;
            i2c2_t.stage = I2C_ST_ADDR_W;
            break;

        case I2C_ST_ADDR_W:
            // Address(W) completed; check ACK
            if (I2C2STATbits.ACKSTAT) { 
                i2c2_t.stage = I2C_ST_ERROR; 
                break; 
            }
            
            // No Error --- OK to continue
            // Send register address
            I2C2TRN = i2c2_t.reg[i2c2_t.reg_idx++];
                    
            // Only advance to next state if sent all the commands
            if (i2c2_t.reg_idx >= i2c2_t.command_len) {
                i2c2_t.stage = I2C_ST_REG;
            }
            break;

        case I2C_ST_REG:
            if (I2C2STATbits.ACKSTAT) { 
                i2c2_t.stage = I2C_ST_ERROR; 
                break; 
            }
            
            if (i2c2_t.command_wait) {
                // Start a timer for waiting
                ES_Timer_InitTimer(ENV_WAIT_TIMER, i2c2_t.command_wait_time);
            } else {
                // Immediately continue with Repeated start
                I2C2CONbits.RSEN = 1;
            }
            i2c2_t.stage = I2C_ST_RESTART;
            break;

        case I2C_ST_RESTART:
            // Restart complete --> send device addr (read)
            I2C2TRN = (i2c2_t.dev7 << 1) | I2C_READ;
            i2c2_t.stage = I2C_ST_ADDR_R;
            break;

        case I2C_ST_ADDR_R:
            if (I2C2STATbits.ACKSTAT) { 
                i2c2_t.stage = I2C_ST_ERROR; 
                break; 
            }
            
            // Begin first receive
            I2C2CONbits.RCEN = 1;
            i2c2_t.stage = I2C_ST_RECV;
            break;

        case I2C_ST_RECV:
            // A byte has been received, read it
            if (!I2C2STATbits.RBF) {
                // No byte yet, wait for next interrupt
                break;
            }
            i2c2_t.buf[i2c2_t.idx++] = I2C2RCV;

            // Prepare ACK/NACK
            if (i2c2_t.idx < i2c2_t.len) {
                I2C2CONbits.ACKDT = 0; // ACK
            } else {
                I2C2CONbits.ACKDT = 1; // NACK on last
            }
            I2C2CONbits.ACKEN = 1;     // send ACK/NACK
            i2c2_t.stage = I2C_ST_ACK;
            break;

        case I2C_ST_ACK:
            if (i2c2_t.idx < i2c2_t.len) {
                // More bytes to receive
                I2C2CONbits.RCEN = 1;
                i2c2_t.stage = I2C_ST_RECV;
            } else {
                // Done reading ? Stop
                I2C2CONbits.PEN = 1;
                i2c2_t.stage = I2C_ST_STOP;
            }
            break;

        case I2C_ST_STOP:
            // Stop complete
            i2c2_t.stage = I2C_ST_DONE;
            // fallthrough
        case I2C_ST_DONE:
        {
            i2c2_t.busy = false;
            
            ES_Event_t new_event = {EV_I2C_COMPLETE, 0};
            if (i2c2_t.dev7 == SHT4x_ADD) {
                new_event.EventParam = 0; // indicate humidity/temperature sensor
            } else if (i2c2_t.dev7 == SGP41_ADD) {
                new_event.EventParam = 1; // indicate air quality sensor
            }
            PostEnvironmentSensorSM(new_event);
            
            i2c2_t.stage = I2C_ST_IDLE;
            break;
        }

        case I2C_ST_ERROR:
            // Issue Stop to release bus if possible
            if (!I2C2CONbits.PEN) {
                I2C2CONbits.PEN = 1;
            }
            i2c2_t.busy = false;
            
            ES_Event_t new_event = {EV_I2C_ERROR, 0};
            PostEnvironmentSensorSM(new_event);
            
            i2c2_t.stage = I2C_ST_IDLE;
            break;

        default:
            break;

    }
}