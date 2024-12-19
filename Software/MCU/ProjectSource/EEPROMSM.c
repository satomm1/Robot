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
#include "dbprintf.h"
#include "sys/attribs.h"
/*----------------------------- Module Defines ----------------------------*/
#define WREN 0b00000110
#define WRDI 0b00000100
#define READ 0b00000011
#define WRITE 0b00000010
#define RDSR 0b00000101

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static EEPROMState_t CurrentState;

// Variables for keeping track of address/page we are writing to
static uint32_t CurrentAddress = 0;
static uint32_t SamplesOnCurrentPage = 0;
static uint32_t CurrentPage = 0;

// Variable to assist in TX of SPI data
static bool transferring = false;  // Transferring data
static bool transfer_wait = false; // Waiting for transfer to finish
static bool sent_wren = false;  // waiting for write enable to finish
static bool sent_wrdi = false;  // Waiting for write disable to finish
static bool sent_instr_address = false;
static uint8_t tx_index = 0;
static uint8_t bytes_to_write[32];
static uint16_t num_bytes_to_write = 0;

// Variables to assist in RX of SPI data
static bool receiving = false;
static uint8_t bytes_read[32];
static uint16_t num_bytes_to_read = 0;

// Variable to assist in reading status
static bool status_reading = false;

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
  
  
  // Setup Hold* and WriteProtect* Pins and set high
  TRISBCLR = _TRISB_TRISB12_MASK | _TRISB_TRISB13_MASK; // Output
  ANSELBCLR = _ANSELB_ANSB12_MASK | _ANSELB_ANSB13_MASK;  // Digital
  LATBbits.LATB12 = 1; // Set high
  LATBbits.LATB13 = 1; // Set high
  
  // Set up SPI5
  
  // Set SCK5 and SS5 as Digital outputs
  TRISFCLR = _TRISF_TRISF12_MASK | _TRISF_TRISF13_MASK;
  ANSELFCLR = _ANSELF_ANSF12_MASK | _ANSELF_ANSF13_MASK;
  
  // Make proper mappings
  TRISGCLR = _TRISG_TRISG0_MASK; // Set SDO5 as digital output
  TRISGSET = _TRISG_TRISG1_MASK; // Set SDI5 as digital input
  
  SDI5R = 0b1100; // SDI5 -> RG1
  RPG0R = 0b1001; // RG0 -> SDO5
  RPF12R = 0b1001; // RF12 -> SS5
  
  SPI5CON = 0;
  SPI5CON2 = 0;
  SPI5CONbits.MSSEN = 1; // SS automatically driven
  SPI5CONbits.MCLKSEL = 0; // PBCLK2 used by BRG
  SPI5CONbits.ENHBUF = 1; // Enhanced buffer on
  SPI5CONbits.DISSDO = 0; // SDO5 controlled by the module
  SPI5CONbits.MODE32 = 0;
  SPI5CONbits.MODE16 = 0; // 8 bit mode
  SPI5CONbits.SMP = 0; // Data sampled at middle
  SPI5CONbits.CKE = 0; // Output data changes on transition from idle clock to active clock
  SPI5CONbits.CKP = 1; // Idle clock state is high
  SPI5CONbits.MSTEN = 1; // Host mode
  SPI5CONbits.DISSDI = 0; // SDI pin controlled by the module
  SPI5CONbits.STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
  SPI5CONbits.SRXISEL = 0b01; // Interrupt is generated when the buffer is not empty
  
  SPI5BRG = 9; // F_pb = 50 MHz --> F_sck = 2.5 MHz,  (EEPROM max 10 MHz)
  
  SPI5STATbits.SPIROV = 0; // Clear overflow bit
  
  //************  Set up interrupts ************//
  __builtin_disable_interrupts(); // Global disable interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  
  // Clear interrupt flags
  IFS5CLR = _IFS5_SPI5RXIF_MASK | _IFS5_SPI5TXIF_MASK;
  
  // Set interrupt priorities
  IPC44bits.SPI5RXIP = 7;
  IPC44bits.SPI5RXIS = 3;
  IPC44bits.SPI5TXIP = 7;
  IPC44bits.SPI5TXIS = 3;
  
  // Make sure TX interrupt is disabled to start
  IEC5CLR = _IEC5_SPI5TXIE_MASK;
  
  // Enable RX interrupt 
  IEC5SET = _IEC5_SPI5RXIE_MASK;
   
  __builtin_enable_interrupts(); // Global enable interrupts
  
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
        // now put the machine into the actual initial state
        CurrentState = EEPROMWaiting;
      }
    }
    break;

    case EEPROMWaiting: 
    {
      switch (ThisEvent.EventType)
      {
        
        case EV_EEPROM_RX_COMPLETE:
        {
            DB_printf("Received Data is: \r\n");
            for (uint8_t i = 0; i<num_bytes_to_read; i++) {
                DB_printf("%d\r\n", bytes_read[i]);
            }
        }
        break;
        
        case EV_WRITE_ENABLED:
        {    
            DB_printf("Sent WREN, SS Status = %d\r\n", PORTFbits.RF12);
            
            if (ThisEvent.EventParam) {
                CurrentState = EEPROMWriting;
                SPI5CONbits.STXISEL = 0b11; // Interrupt is generated when the buffer is not full (has one or more empty elements)
                IEC5SET = _IEC5_SPI5TXIE_MASK; // Enable interrupt
                
                DB_printf("Entered EEPROMWriting\r\n");
            } else {
                CurrentState = EEPROMWriteEnabled;
                DB_printf("Entered EEPROMWriteEnabled\r\n");
            }
        }
        break;
            
        default:
          ;
      }  
    }
    break;
    
    case EEPROMWriteEnabled: 
    {
      switch (ThisEvent.EventType)
      {
        case EV_BEGIN_WRITE: 
        { 
          CurrentState = EEPROMWriting;
          SPI5CONbits.STXISEL = 0b11; // Interrupt is generated when the buffer is not full (has one or more empty elements)
          IEC5SET = _IEC5_SPI5TXIE_MASK; // Enable interrupt
          
          DB_printf("Entered EEPROMWriting\r\n");
        }
        break;
        
        case EV_WRITE_DISABLED:
        {
          DB_printf("Sent WRDI, SS Status = %d\r\n", PORTFbits.RF12);
            
          CurrentState = EEPROMWaiting;
          
          DB_printf("Entered EEPROMWaiting\r\n");
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
        case ES_TIMEOUT: 
        { 
          // 5 ms wait timer elapsed, write complete
          CurrentState = EEPROMWaiting;  
          
          DB_printf("Entered EEPROMWaiting\r\n");
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

// Note: To facilitate clean pages, we only accept data in sequences of 32 x 8 bytes
//       Each page is 256 bytes, so this allows 8 samples per page, even if each
//       sample is less than 32 bytes.

void WriteEnable(void) {
    
    if (CurrentState != EEPROMWaiting) {
        // Not in valid state to perform write enable
        transferring = false;
        DB_printf("Not in valid state to write enable!\r\n");
        return;
    }
    
    // Change interrupt trigger and clear interrupt flag
    SPI5CONbits.STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
    IFS5CLR = _IFS5_SPI5TXIF_MASK;
    
    sent_wren = true;  // Waiting for SS go inactive after sending WREN
    sent_instr_address = false; // Have not yet send address/instruction bytes
    SPI5BUF = WREN;
    
    // Enable interrupt
    IEC5SET = _IEC5_SPI5TXIE_MASK;
}

void WriteDisable(void) {
    
    if (CurrentState != EEPROMWriteEnabled) {
        // Not in valid state to perform write disable
        DB_printf("Not write enabled! No need to write disable...\r\n");
        return;
    }
    
    // Change interrupt trigger and clear interrupt flag
    SPI5CONbits.STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
    IFS5CLR = _IFS5_SPI5TXIF_MASK;
    
    sent_wrdi = true;
    SPI5BUF = WRDI;
    
    // Enable the transmit interrupt
    IEC5SET = _IEC5_SPI5TXIE_MASK;
}

void WriteByteEEPROM(uint8_t data) {
    
    if (CurrentState == EEPROMWriting) {
        return; // Write in progress, don't do anything
    }
    
    tx_index = 0;
    bytes_to_write[0] = data;
    num_bytes_to_write = 1;
        
    transferring = true;  // In writing mode
    
    if (CurrentState == EEPROMWaiting) {
        WriteEnable();  // Write Enable First
    } else if (CurrentState == EEPROMWriteEnabled) {
        // Already WriteEnabled, can begin write of data
        ES_Event_t new_event = {EV_BEGIN_WRITE, 0};
        PostEEPROMSM(new_event);
    }
}

void WriteMultiBytesEEPROM(uint8_t *data, uint16_t N) {
    
    if (N > 32) {
        // We don't allow writes of more than 32 bytes at a time
        return;
    }
    
    if (CurrentState == EEPROMWriting) {
        return; // Write in progress, don't do anything
    }
    
    tx_index = 0;
    num_bytes_to_write = N;
    for (uint8_t i = 0; i < N; i++){
        bytes_to_write[i] = data[i];
    }
    
    transferring = true; // In writing mode

    if (CurrentState == EEPROMWaiting) {
        WriteEnable();  // Write Enable First
    } else if (CurrentState == EEPROMWriteEnabled) {
        // Already WriteEnabled, can begin write of data
        ES_Event_t new_event = {EV_BEGIN_WRITE, 0};
        PostEEPROMSM(new_event);
    }
}

void ReadByteEEPROM(uint32_t address) {
    
    receiving = true; // We are now interested in data we receive
    num_bytes_to_read = 1;
    
    // Get relevant address bytes
    uint8_t address_byte1 = (address >> 16) & 0xFF;
    uint8_t address_byte2 = (address >> 8) & 0xFF;
    uint8_t address_byte3 = (address >> 0) & 0xFF;
    
    DB_printf("Reading Address: %d\r\n", address);
    
    SPI5BUF = READ;
    SPI5BUF = address_byte1;
    SPI5BUF = address_byte2;
    SPI5BUF = address_byte3;
    SPI5BUF = 0x0F; // For retrieving data
}

void ReadMultiBytesEEPROM(uint32_t address, uint16_t N) {
    if (N>32) {
        // Only allowed to read up to 32 bytes at a time
        return;
    }
    
    receiving = true; // We are now interested in data we receive
    num_bytes_to_read = N;
    
    // Get relevant address bytes
    uint8_t address_byte1 = (address >> 16) & 0xFF;
    uint8_t address_byte2 = (address >> 8) & 0xFF;
    uint8_t address_byte3 = (address >> 0) & 0xFF;
    
    SPI5BUF = READ;
    SPI5BUF = address_byte1;
    SPI5BUF = address_byte2;
    SPI5BUF = address_byte3;
    
    // TODO: Send correct number of 0b00000000 bytes, will need TX interrupt as well
}

void ReadStatusEEPROM(void) {
    status_reading = true;
    SPI5BUF = RDSR;
    SPI5BUF = 0xFF;
}

/***************************************************************************
 private functions
 ***************************************************************************/

void __ISR(_SPI5_TX_VECTOR, IPL7SRS) SPI5TXHandler(void)
{
    IEC5CLR = _IEC5_SPI5TXIE_MASK; // Disable the interrupt
    IFS5CLR = _IFS5_SPI5TXIF_MASK; // clear the interrupt flag 
    
    if (sent_wren) {
        sent_wren = false;
                
        // Post Event to say we have write enabled complete
        ES_Event_t new_event = {EV_WRITE_ENABLED, 0};
        
        if (transferring) {
            new_event.EventParam = 1; // Indicate we have data to transfer
        }
        
        PostEEPROMSM(new_event);
    } else if (sent_wrdi) {
        sent_wrdi = false;
        
        // Post Event to say we have write disabled complete
        ES_Event_t new_event = {EV_WRITE_DISABLED, 0};        
        PostEEPROMSM(new_event);
    } else if (transferring) {  // Here we finish sending outstanding bytes
        
        if (!sent_instr_address) {
           // Get relevant address bytes
           uint8_t address_byte3 = (CurrentAddress >> 0) & 0xFF;
           uint8_t address_byte2 = (CurrentAddress >> 8) & 0xFF;
           uint8_t address_byte1 = (CurrentAddress >> 16) & 0xFF;

           DB_printf("Writing to address: %x\r\n", CurrentAddress);

           // Send Write Sequence
           SPI5BUF = WRITE; // Write Instruction
           SPI5BUF = address_byte1; // Write Address
           SPI5BUF = address_byte2;
           SPI5BUF = address_byte3;
           
           sent_instr_address = true; // We have now sent address and instruction bits
        }
        
        // Send bytes as long as we still have bytes to send and TX buffer not full
        while ((tx_index < num_bytes_to_write) && SPI5STATbits.SPITBF) {
                SPI5BUF = bytes_to_write[tx_index];
                tx_index += 1;
        }
        
        // Check if still have bytes to send
        if (tx_index == num_bytes_to_write) {
            transferring = false;  // Done with TX
            transfer_wait = true; // Now just wait for TX to finish
            SPI5CONbits.STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
        }
        IEC5SET = _IEC5_SPI5TXIE_MASK; // Enable interrupt
    } else if (transfer_wait) {  // Here we know we have finished the TX
        transfer_wait = false; // transfer has finished
        
        // Update the current address/page
        CurrentAddress += 32;
        SamplesOnCurrentPage += 1;
        if (SamplesOnCurrentPage == 8) {
            CurrentPage += 1;
            SamplesOnCurrentPage = 0;
        }
        
        ES_Timer_InitTimer(EEPROM_TIMER, 5); // Set 5 ms wait timer
    }
}

void __ISR(_SPI5_RX_VECTOR, IPL7SRS) SPI5RXHandler(void) {
    
    static uint8_t rx_data;
    static uint8_t rx_indx = 0;
    
    if (receiving && SPI5STATbits.RXBUFELM >= 5) {
        while (!SPI5STATbits.SPIRBE) {
            rx_data = SPI5BUF;
            
            // Only care about actual data (prev data is only setup bits)
            if (rx_indx >= 4) {
                DB_printf("rx_data: %d\r\n", rx_data);
                bytes_read[rx_indx-4] = rx_data;
            } 
            
            rx_indx += 1;
            
            if (rx_indx-4 >= num_bytes_to_read) {
                // We read all the bytes we expected to
                rx_indx = 0;
                receiving = false;
                
                // TODO Publish an event to print this data
                ES_Event_t new_event = {EV_EEPROM_RX_COMPLETE, 0};
                PostEEPROMSM(new_event);
            }
        }
    } else if (receiving) {
        
    } else if (status_reading) {
        while (!SPI5STATbits.SPIRBE && status_reading) {
            rx_data = SPI5BUF;
            
            rx_indx += 1;
            if (rx_indx == 2) {
                DB_printf("Status is: %d\r\n", rx_data);
                rx_indx = 0;
                
                status_reading = false;
            }
        }   
    } else {
        // For now, we just discard all values since we aren't reading any values
        while (!SPI5STATbits.SPIRBE) {
            rx_data = SPI5BUF;
//            DB_printf("Received rx\r\n");
        }
    }
    
    IFS5CLR = _IFS5_SPI5RXIF_MASK; // clear the interrupt flag 
}