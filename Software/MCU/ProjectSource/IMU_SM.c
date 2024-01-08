/****************************************************************************
 Module
   IMU_SM.c

 Description
   This is a file for implementing reading from the ICM-42688-P 6-axis IMU

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
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
#define READ  0b10000000
#define WRITE 0b00000000
#define FIFO_PACKET_SIZE 16

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitIMU(void);
void SetBank(uint8_t Bank);
void WriteIMU(uint8_t Address, uint8_t Data);
void ReadFIFO(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static ImuState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

uint8_t FIFO_index = 0;
uint8_t Active_FIFO = 1;
uint8_t Readable_FIFO;
int8_t IMU_FIFO[2][16];

AccelGyroData_t Accel[3]; // Holds the current acceleration
AccelGyroData_t Gyro[3]; // Holds the current gyroscope readings

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
  
  // Set interrupt pins to inputs
  TRISDSET = _TRISD_TRISD12_MASK | _TRISD_TRISD13_MASK;
  
  INT2R = 0b1010; // Map RD12 -> External interrupt 2
  
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
  SPI1CONbits.MCLKSEL = 0; // Use PBCLK2 for the Baud Rate Generator (50 MHz)
  SPI1CONbits.ENHBUF = 1; // Enhance buffer enabled (use FIFOs)
  SPI1CONbits.DISSDO = 0; // SDO1 is used by the module
  SPI1CONbits.MODE32 = 0; // 8 bit mode
  SPI1CONbits.MODE16 = 0; // 8 bit mode
  SPI1CONbits.SMP = 0; // Data sampled at middle of data output time
  SPI1CONbits.CKE = 0; // Output data changes on transition from idle to active clock state
  SPI1CONbits.CKP = 1; // Idle state for the clock is high level
  SPI1CONbits.MSTEN = 1; // Host mode
  SPI1CONbits.DISSDI = 0; // The SDI pin is controlled by the module
  SPI1CONbits.STXISEL = 0b00; // Interrupt generated when last transfer shifted out of SPISR and transmit operations are complete
  SPI1CONbits.SRXISEL = 0b11; // Interrupt is generated when the buffer is full

  SPI1CON2 = 0; // Reset SPI1CON2 register settings
  SPI1CON2bits.AUDEN = 0; // Audio protocol is disabled
  
  while (!SPI1STATbits.SPIRBE){
      uint8_t ClearData = SPI1BUF;
  }
  SPI1STATbits.SPIROV = 0; // Clear the Receive overflow bit
  
  SPI1BRG = 15; // 1.56 MHz clock frequency, IMU Has max frequency of 24 MHz
  
  // Setup Timer 6 (Used for integrating the accels)
  T6CON = 0; // Reset the timer register settings
  T6CONbits.TCKPS = 0b111; // 1:256 p;rescale value
  T6CONbits.T32 = 0; // Use 16 bit timer not 32 bit
  T6CONbits.TCS = 0; // Internal peripheral clock
    
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC27bits.SPI1RXIP = 7; // SPI2RX
  IPC3bits.INT2IP = 7; // External Interrupt 2
  IPC7bits.T6IP = 7; // T6
  
  // Clear interrupt flags
  IFS3CLR = _IFS3_SPI1RXIF_MASK | _IFS3_SPI1TXIF_MASK;
  IFS0CLR = _IFS0_INT2IF_MASK | _IFS0_T6IF_MASK;
  
  // Local enable interrupts
  IEC3SET = _IEC3_SPI1RXIE_MASK;
  IEC0SET = _IEC0_T6IE_MASK;
  
  // Disable TX interrupt
  IEC3CLR = _IEC3_SPI1TXIE_MASK;
  
  // Disable Int 2 Interrupt
  IEC0CLR = _IEC0_INT2IE_MASK;
  INTCONbits.INT2EP = 0; // Interrupt on falling edge
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
  SPI1CONbits.ON = 1; // Finally turn SPI1 on
    
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
        InitIMU();

        // now put the machine into the actual initial state
        CurrentState = IMUWait;
      }
    }
    break;

    case IMUWait:      
    {
      switch (ThisEvent.EventType)
      {
        case EV_IMU_DATA_UPDATE:  
        { 
          uint8_t indx = ThisEvent.EventParam;
          Accel[0].DataStruct.LowerByte = IMU_FIFO[indx][2];
          Accel[0].DataStruct.UpperByte = IMU_FIFO[indx][1];
          Accel[1].DataStruct.LowerByte = IMU_FIFO[indx][4];
          Accel[1].DataStruct.UpperByte = IMU_FIFO[indx][3];
          Accel[2].DataStruct.LowerByte = IMU_FIFO[indx][6];
          Accel[2].DataStruct.UpperByte = IMU_FIFO[indx][5];
          
          Gyro[0].DataStruct.LowerByte = IMU_FIFO[indx][8];
          Gyro[0].DataStruct.UpperByte = IMU_FIFO[indx][7];
          Gyro[1].DataStruct.LowerByte = IMU_FIFO[indx][10];
          Gyro[1].DataStruct.UpperByte = IMU_FIFO[indx][9];
          Gyro[2].DataStruct.LowerByte = IMU_FIFO[indx][12];
          Gyro[2].DataStruct.UpperByte = IMU_FIFO[indx][11];
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

/****************************************************************************
 Function
     InitImu

 Parameters
     None

 Returns
     None

 Description
     Sends the correct sequence of writes to the IMU via SPI to correctly set 
     up the IMU and the IMU FIFO
****************************************************************************/
void InitIMU(void)
{
    /************************ Bank 0 Registers ************************/
    SetBank(0);
    
    // Configure interrupt types:
    WriteIMU(0x14, 0b00110110); // Push-Pull, Active Low, Latched Interrupt
    
    // Set up the IMU to write to FIFO (FIFO_CONFIG)
    WriteIMU(0x16, 0b01000000); // Stream to FIFO
    
    // Needed for correct operation of Interrupts (INT_CONFIG1)
    WriteIMU(0x64, 0b00000000);
    
    // Set Gyro ODR rate to 25 Hz and sensitivity to +/- 250 degrees/sec (GYRO_CONFIG0)
    WriteIMU(0x4F, 0b01101010);
    
    // Set Accel ODR rate to 25 Hz and sensitivity to +/- 8g (Accel_CONFIG0)
    WriteIMU(0x50, 0b00101010);
    
    // Set the FIFO watermark threshold (FIFO_CONFIG2)
    WriteIMU(0x60, 1); // Set to watermark of 1
    
    // Set the FIFO threshold interrupt to clear after reading a byte (INT_CONFIG0)
    WriteIMU(0x63, 0b00001000);
    
    // Route the FIFO threshold interrupt to Int1 (INT_SOURCE0)
    WriteIMU(0x65, 0b00000100); 
    
    // Set FIFO Config settings (FIFO_CONFIG1 register)
    WriteIMU(0x5F, 0b00100111);
    
    // Turn Accel and Gyro on (PWR_MGMT0 register)
    WriteIMU(0x4E, 0b00001111);
    
    
    /************************ Bank 1 Registers ************************/
    SetBank(1);
    
    // Set pin 9 to be Interrupt 2 functionality (Rather than FSYNC or CLKIN)
    WriteIMU(0x7B, 0b00000000);
    
    SetBank(0); // Return to Bank 0
    
    // Enable the Int 2 Interrupt on the PIC32
    IFS0CLR = _IFS0_INT2IF_MASK; // Clear the interrupt flag
    IEC0SET = _IEC0_INT2IE_MASK;
    
    return;
}

/****************************************************************************
 Function
     SetBank

 Parameters
     uint8_t Bank: The bank number that we will send to

 Returns
     None

 Description
     Sends the correct sequence of writes to the IMU via SPI to correctly set 
     the bank for the next write or read
****************************************************************************/
void SetBank(uint8_t Bank)
{
    SPI1BUF = WRITE | 0x76;
    SPI1BUF = Bank;
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    return;
}

/****************************************************************************
 Function
     WriteIMU

 Parameters
     uint8_t Address: The address of the register to write to
     uint8_t Data: The data to write to the register

 Returns
     None

 Description
     Sends the correct sequence of bytes to the IMU via SPI to write to the 
     desired register. 
 
    ***Assumes the Bank is already correctly selected***
****************************************************************************/
void WriteIMU(uint8_t Address, uint8_t Data)
{
    SPI1BUF = WRITE | Address;
    SPI1BUF = Data;
    while (SPI1STATbits.SPIBUSY) {
        // Blocking code --- OK Since we are only calling this function during initialization
    }
    return;
}

/****************************************************************************
 Function
     ReadFIFO

 Parameters
     None

 Returns
     None

 Description
     Sends the correct sequence of bytes to read from the IMU FIFO
 
    ***Assumes the Bank is already correctly selected***
****************************************************************************/
void ReadFIFO(void)
{
    FIFO_index = 0;
    if (Active_FIFO) {
        Active_FIFO = 0;
    } else {
        Active_FIFO = 1;
    }
    
    // Turn on TX interrupt -- Interrupt will be generated after this is 
    // completely transferred
    IEC3SET = _IEC3_SPI1TXIE_MASK;
    
    // Write To FIFO_Data Register
    SPI1BUF = READ | 0x30;
}

void __ISR(_SPI1_RX_VECTOR, IPL7SRS) SPI1RXHandler(void)
{
    static ES_Event_t NewEvent = {EV_IMU_DATA_UPDATE, 0}; // static for speed
    
    // Read all the data from SPI1Buffer -- Must make sure we visit this before 
    // sending more data to ensure we don't have overflow
    for (uint8_t i=0; i<16; i++) {
        IMU_FIFO[Active_FIFO][FIFO_index] = SPI1BUF;
    }
    
    Readable_FIFO = Active_FIFO; // Update what FIFO contains valid info
    IFS3CLR = _IFS3_SPI1RXIF_MASK; // clear the interrupt flag
    
    NewEvent.EventParam = Active_FIFO;
    PostImuSM(NewEvent);
    
    Accel[0].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][2];
    Accel[0].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][1];
    Accel[1].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][4];
    Accel[1].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][3];
    Accel[2].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][6];
    Accel[2].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][5];

    Gyro[0].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][8];
    Gyro[0].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][7];
    Gyro[1].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][10];
    Gyro[1].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][9];
    Gyro[2].DataStruct.LowerByte = IMU_FIFO[Active_FIFO][12];
    Gyro[2].DataStruct.UpperByte = IMU_FIFO[Active_FIFO][11];
}

void __ISR(_SPI1_TX_VECTOR, IPL7SRS) SPI1TXHandler(void)
{
    uint8_t temp; // static for speed
    
    IEC3CLR = _IEC3_SPI1TXIE_MASK; // Disable TX interrupt
    
    temp = SPI1BUF; // Empty buffer that holds junk data

    IFS3CLR = _IFS3_SPI1TXIF_MASK; // clear the interrupt flag
    
    // Write 16 times via SPI to get data from IMU FIFO
    while (!SPI1STATbits.SPIRBE) {
        for (uint8_t i=0; i<FIFO_PACKET_SIZE; i++) {
            SPI1BUF = 0;
        }
    }
}

// Interrupt indicates new IMU data is read: Initiate FIFO reading
void __ISR(_EXTERNAL_2_VECTOR, IPL7SRS) External2Handler(void)
{
    IFS0CLR = _IFS0_INT2IF_MASK; // Clear the interrupt flag
    ReadFIFO();   
}