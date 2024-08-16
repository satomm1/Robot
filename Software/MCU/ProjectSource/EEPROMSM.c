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
#include <sys/attribs.h>
#include "EEPROMSM.h"
#include "MotorSM.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define WREN  0b00000110 // Write enable
#define WRDI  0b00000100 // Write disable
#define RDSR  0b00000101 // Read status register
#define WRSR  0b00000001 // Write status register
#define READ  0b00000011 // Read from memory array
#define WRITE 0b00000010 // Write to memory array
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void ReadStatus(void);
void WriteArray(EEPROM_Address_t Address, int8_t * Data, uint8_t Bytes2Send);
bool ReadArray(EEPROM_Address_t Address, uint8_t Bytes2Get);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static EEPROMState_t CurrentState;
static volatile __SPI5CONbits_t * pSPICON;
static volatile __SPI5CON2bits_t * pSPICON2;
static volatile __SPI5STATbits_t * pSPISTAT;
static volatile uint32_t * pSPIBRG;
static volatile uint32_t * pSPIBUF;

static uint8_t bytes_to_read = 0;
static volatile uint8_t rx_data[8];
static volatile uint8_t rx_indx = 0;

static EEPROM_Address_t CurrentAddress;

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
  
  CurrentAddress.FullAddress = 0;
  
  // Set write protect/hold as outputs (HOLD* = RB13, WRITE_PROTECT* = RB12)
  TRISBCLR = _TRISB_TRISB12_MASK | _TRISB_TRISB13_MASK;
  ANSELBCLR = _ANSELB_ANSB12_MASK | _ANSELB_ANSB13_MASK;
  LATBbits.LATB12 = 1; // Disable Write protect
  LATBbits.LATB13 = 1; // Disable Hold
  
  // Set up SPI5
  pSPICON = (__SPI5CONbits_t *)&SPI5CON;
  pSPICON2 = (__SPI5CON2bits_t *)&SPI5CON2;
  pSPIBRG = &SPI5BRG;
  pSPIBUF = &SPI5BUF;
  pSPISTAT = (__SPI5STATbits_t *)&SPI5STAT;
  SPI5CON = 0;
  SPI5CON2 = 0;
  
  // Set SCK5 and SS5 as Digital outputs
  TRISFCLR = _TRISF_TRISF12_MASK | _TRISF_TRISF13_MASK;
  ANSELFCLR = _ANSELF_ANSF12_MASK | _ANSELF_ANSF13_MASK;
  
  TRISGCLR = _TRISG_TRISG0_MASK; // Set SDO5 as digital output
  TRISGSET = _TRISG_TRISG1_MASK; // Set SDI5 as digital input
  
  // Make proper mappings
  SDI5R = 0b1100; // SDI5 -> RG1
  RPG0R = 0b1001; // RG0 -> SDO5
  RPF12R = 0b11001; // RF12 -> SS5
  
  pSPICON->MSSEN = 1; // SS automatically driven
  pSPICON->MCLKSEL = 0; // PBCLK2 used by BRG (50 MHz))
  pSPICON->ENHBUF = 1; // Enhanced buffer on
  pSPICON->DISSDO = 0; // SDO5 controlled by the module
  pSPICON->MODE32 = 0;
  pSPICON->MODE16 = 0; // 8 bit mode
  pSPICON->SMP = 0; // Data sampled at middle
  pSPICON->CKE = 0; // Output data changes on transition from idle clock to active clock
  pSPICON->CKP = 1; // Idle clock state is high
  pSPICON->MSTEN = 1; // Host mode
  pSPICON->DISSDI = 0; // SDI pin controlled by the module
  pSPICON->STXISEL = 0b00; // Interrupt is generated when the last transfer is shifted out of SPISR and transmit operations are complete
  pSPICON->SRXISEL = 0b01; // Interrupt is generated when the buffer is not empty
  
  pSPISTAT->SPIROV = 0; // Clear overflow bit
  *pSPIBRG = 2; // (8.33 MHz, Max allowed for EEPROM is 10 MHz)
  
  // Set up interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC44bits.SPI5TXIP = 7; // SPI5TX
  IPC44bits.SPI5RXIP = 7; // SPI5RX
  
  // Disable the RX/TX interrupt
  IEC5CLR = _IEC5_SPI5RXIE_MASK | _IEC5_SPI5TXIE_MASK; // SPI5
  
  // Clear Interrupt Flags
  IFS5CLR = _IFS5_SPI5RXIF_MASK | _IFS5_SPI5TXIF_MASK; // SPI5
  
  __builtin_enable_interrupts(); // Global enable interrupts
  pSPICON->ON = 1; // Turn SPI module on
  
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
          
          ReadStatus();
//          EEPROM_Address_t New_Address;
//          New_Address.FullAddress = 0;
//          ReadArray(New_Address, 1);
//          int8_t New_Data[8];
//          New_Data[0] = 12;
//          WriteArray(New_Address, New_Data, 1);
      }
    }
    break;

    case EEPROMWaiting: 
    {
      switch (ThisEvent.EventType)
      {
        case EV_EEPROM_WRITE_HISTORY: 
        { 
          CurrentState = EEPROMWriting;  
          int8_t* Data = GetRLData();
          WriteArray(CurrentAddress, Data, 11);
          
          CurrentAddress.FullAddress += 11;
          
          // Make sure there is room to fit on each page
          if (((CurrentAddress.FullAddress + 11) % 256) < 11) {
              CurrentAddress.FullAddress += 11 - (CurrentAddress.FullAddress + 11) % 256;
          }
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
          CurrentState = EEPROMWaiting;  
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
        case EV_EEPROM_READ_FINISHED: 
        { 
          DB_printf("Read Data:\r\n");
          for (uint8_t i=0; i<bytes_to_read; i++){
              DB_printf("%d: %d\r\n", i, rx_data[i]);
          }
          CurrentState = EEPROMWaiting;  
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

// Read from status register to make sure everything is working
void ReadStatus(void) {
    CurrentState = EEPROMReading; 
    bytes_to_read = 2;
    *pSPIBUF = RDSR;
    *pSPIBUF = 0b00000000;
    
    rx_indx = 0;
    IEC5SET = _IEC5_SPI5RXIE_MASK; // Enable SPI5 RX Interrupt
}

void WriteArray(EEPROM_Address_t Address, int8_t * Data, uint8_t Bytes2Send){
    // Write the correct bytes to the EEPROM
    bytes_to_read = 5 + Bytes2Send;
    *pSPIBUF = WREN;
    *pSPIBUF = WRITE;
    *pSPIBUF = Address.AddressStruct.Byte3;
    *pSPIBUF = Address.AddressStruct.Byte2;
    *pSPIBUF = Address.AddressStruct.Byte1;
    for (uint8_t ii = 0; ii < Bytes2Send; ii++) {
        *pSPIBUF = Data[ii];
    }

    IEC5SET = _IEC5_SPI5TXIE_MASK; // Enable SPI5 TX Interrupt
}

bool ReadArray(EEPROM_Address_t Address, uint8_t Bytes2Get){
    if (CurrentState == EEPROMWaiting) {
        CurrentState = EEPROMReading;
        
        bytes_to_read = 4 + Bytes2Get;
        *pSPIBUF = READ;
        *pSPIBUF = Address.AddressStruct.Byte3;
        *pSPIBUF = Address.AddressStruct.Byte2;
        *pSPIBUF = Address.AddressStruct.Byte1;
        for (uint8_t ii = 0; ii < Bytes2Get; ii++) {
            *pSPIBUF = 0x00;
        } 
        rx_indx = 0;
        IEC5SET = _IEC5_SPI5RXIE_MASK; // Enable SPI5 RX Interrupt
        
        return true;
    } else {
        return false;
    }
}

void __ISR(_SPI5_RX_VECTOR, IPL7SRS) SPI5RXHandler(void)
{    
    ES_Event_t DoneEvent = {EV_EEPROM_READ_FINISHED,0};
    
    while (pSPISTAT->SPIRBE == 0){
        rx_data[rx_indx] = *pSPIBUF;
        rx_indx += 1;
        DB_printf("Here\r\n");
    }
    
    if (rx_indx == bytes_to_read) {
        IEC5CLR = _IEC5_SPI5RXIE_MASK; // Disable SPI5 RX Interrupt
        IFS5CLR = _IFS5_SPI5RXIF_MASK; // clear the interrupt flag 
        
        DoneEvent.EventParam = rx_indx;
        PostEEPROMSM(DoneEvent);
        
        DB_printf("Here\r\n");
    }
}

void __ISR(_SPI5_TX_VECTOR, IPL7SRS) SPI5TXHandler(void)
{    
    static uint8_t dumped_data;
    
    IEC5CLR = _IEC5_SPI5TXIE_MASK; // Disable SPI5 TX Interrupt
    IFS5CLR = _IFS5_SPI5TXIF_MASK; // clear the interrupt flag 
    
    while (pSPISTAT->SPIRBE == 0) {
        dumped_data = *pSPIBUF;
    }
   
    // Set 5 ms timer (accounts for write time for EEPROM)
    ES_Timer_InitTimer(EEPROM_TIMER, 5);
}