/****************************************************************************
 Module
   MotorSM.c

 Description
   This is a file for implementing the Motors

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
#include "MotorSM.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
#define IC_PERIOD 65535 // Input capture period
#define OC_PERIOD 833   // Output compare period
#define CONTROL_PERIOD 200 // Control update period
#define Kp 1 // Proportional constant for PID law
#define Ki 1 // Integral constant for PID law

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static MotorState_t CurrentState;
static volatile MotorTimer_t MyTimer;
static uint16_t DesiredLeftRPM;
static uint16_t DesiredRightRPM;

static Direction_t LeftDirection = Forward;
static Direction_t RightDirection = Forward;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitMotorSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  
  // Set Motor Driving/Direction pins to outputs
  TRISFCLR = _TRISF_TRISF2_MASK | _TRISF_TRISF8_MASK;
  TRISDCLR = _TRISD_TRISD5_MASK;
  TRISJCLR = _TRISJ_TRISJ3_MASK;
  
  LATFbits.LATF8 = 0; // Start direction pins low
  LATJbits.LATJ3 = 0; // Start direction pins low
  
  // Set encoder pins and fault pins to digital inputs
  ANSELCCLR = _ANSELC_ANSC1_MASK | _ANSELC_ANSC4_MASK;
  TRISCSET = _TRISC_TRISC1_MASK | _TRISC_TRISC4_MASK;
  TRISDSET = _TRISD_TRISD0_MASK;
  TRISHSET = _TRISH_TRISH8_MASK;
  TRISASET = _TRISA_TRISA4_MASK;
  TRISJSET = _TRISJ_TRISJ12_MASK;
  
  // Set motor current pins to be analog inputs
  ANSELJSET = _ANSELJ_ANSJ9_MASK;
  ANSELASET = _ANSELA_ANSA1_MASK;
  TRISJSET = _TRISJ_TRISJ9_MASK;
  TRISASET = _TRISA_TRISA1_MASK;
  
  // Setup Timers
  // Timer 1 (for control update)
  T1CON = 0; // Reset the timer 1 register settings
  T1CONbits.TCKPS = 0b00; // 1:1 prescale value
  T1CONbits.TCS = 0; // User internal peripheral clock
  PR1 = CONTROL_PERIOD; // The amount of time we should do a control update
  TMR1 = 0; // Set TMR1 to 0
  
  // Timer 2 (for Output Compare)
  T2CON = 0; // Reset the timer 2 register settings
  T2CONbits.TCKPS = 0b000; // 1:1 prescale value
  T2CONbits.T32 = 0; // Use separate 16 bit timers
  T2CONbits.TCS = 0; // Use internal peripheral clock
  PR2 = OC_PERIOD; // Use output compare period
  TMR2 = 0; // Set TMR2 to 0
  
  // Timer 3 (For Input Capture)
  T3CON = 0; // Reset the timer 2 register settings
  T3CONbits.TCKPS = 0b000; // 1:1 prescale value
  T3CONbits.TCS = 0; // Use internal peripheral clock
  PR3 = IC_PERIOD; // Use input capture period
  TMR3 = 0; // Set TMR3 to 0
  
  // Setup Output compare
  OC1CON = 0; // Reset OC1CON register settings
  OC2CON = 0; // Reset OC2CON register settings
  OC1CONbits.OC32 = 0; // Use 16-bit timer source
  OC2CONbits.OC32 = 0; // Use 16-bit timer source
  OC1CONbits.OCTSEL = 0; // Use timerx (timer2)
  OC2CONbits.OCTSEL = 0; // Use timerx (timer2)
  OC1CONbits.OCM = 0b110; // PWM with fault pin disabled
  OC2CONbits.OCM = 0b110; // PWM with fault pin disabled 
  
  // Set the OC register to 0 to initialize
  OC1R = 0;
  OC1RS = 0;
  OC2R = 0;
  OC2RS = 0;
  
  // Setup Input capture
  IC1CON = 0; // Reset IC1CON register settings 
  IC2CON = 0; // Reset IC2CON register settings 
  IC3CON = 0; // Reset IC3CON register settings 
  IC4CON = 0; // Reset IC4CON register settings 
  IC1CONbits.ICTMR = 0; // User timery (timer3)
  IC2CONbits.ICTMR = 0; // User timery (timer3)
  IC3CONbits.ICTMR = 0; // User timery (timer3)
  IC4CONbits.ICTMR = 0; // User timery (timer3)
  IC1CONbits.ICI = 0b00; // Interrupt on every capture event
  IC2CONbits.ICI = 0b00; // Interrupt on every capture event
  IC3CONbits.ICI = 0b00; // Interrupt on every capture event
  IC4CONbits.ICI = 0b00; // Interrupt on every capture event
  IC1CONbits.ICM = 0b001; // Every edge mode
  IC2CONbits.ICM = 0b001; // Every edge mode
  IC3CONbits.ICM = 0b001; // Every edge mode
  IC4CONbits.ICM = 0b011; // Every edge mode
  
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  
  // Set interrupt priorities
  IPC1bits.IC1IP = 7; // IC1
  IPC2bits.IC2IP = 7; // IC2
  IPC4bits.IC3IP = 7; // IC3
  IPC5bits.IC4IP = 7; // IC4
  IPC1bits.T1IP = 7; // T1
  IPC3bits.T3IP = 7; // T3
  
  // Clear interrupt flags
  IFS0CLR = _IFS0_IC1IF_MASK | _IFS0_IC2IF_MASK | _IFS0_IC3IF_MASK | 
          _IFS0_IC4IF_MASK | _IFS0_T1IF_MASK | _IFS0_T3IF_MASK;
  
  // Local enable interrupts
  IEC0SET = _IEC0_IC1IE_MASK | _IEC0_IC2IE_MASK | _IEC0_IC3IE_MASK | 
          _IEC0_IC4IE_MASK | _IEC0_T1IE_MASK | _IEC0_T3IE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
  // Turn Everything On
  IC1CONbits.ON = 1; // Turn input capture on
  IC2CONbits.ON = 1; // Turn input capture on
  IC3CONbits.ON = 1; // Turn input capture on
  IC4CONbits.ON = 1; // Turn input capture on
  OC1CONbits.ON = 1; // Turn OC1 on
  OC2CONbits.ON = 1; // Turn OC2 on
  T1CONbits.ON = 1; // Turn timer 1 on
  T2CONbits.ON = 1; // Turn timer 2 on
  T3CONbits.ON = 1; // Turn timer 3 on
  
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState_Motor;
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
     PostMotorSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostMotorSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMotorSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
****************************************************************************/
ES_Event_t RunMotorSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState_Motor:       
    {
      if (ThisEvent.EventType == ES_INIT) 
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = Motor1;
      }
    }
    break;

    case Motor1:      
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  
        { 
          CurrentState = Motor1;  
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
     QueryMotorSM

 Parameters
     None

 Returns
     MotorState_t The current state of the Motor state machine

 Description
     returns the current state of the Motor state machine
****************************************************************************/
MotorState_t QueryMotorSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

////////////////////// Interrupt Service Routines //////////////////////
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SRS) IC1Handler(void)
{
    
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SRS) IC2Handler(void)
{
    
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL7SRS) IC3Handler(void)
{
    
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, IPL7SRS) IC4Handler(void)
{
    
}

/****************************************************************************
 Function
    T1Handler

 Description
   Computes the control law for the motors
****************************************************************************/
void __ISR(_TIMER_1_VECTOR, IPL7SRS) T1Handler(void)
{  
    // Initializes the PI Variables
    static float LeftErrorSum = 0.0;
    static float RightErrorSum = 0.0;
    static float LeftError;
    static float RightError;
    static int16_t LeftDutyCycle; // Only static here for speed
    static int16_t RightDutyCycle; // Only static here for speed
    
    // Initialize variables used throughout the ISR (Static for speed)
    static uint16_t ActualLeftRPM;
    static uint16_t ActualRightRPM;
    
    
    IFS0CLR = _IFS0_T1IF_MASK; // Clear the timer interrupt
    
    // Calculate Current RPM based on Pulse Lengths from encoders
    ActualLeftRPM = 0.0;
    ActualRightRPM = 0.0;
    
    // Calculate error from desired RPM
    LeftError = DesiredLeftRPM - ActualLeftRPM;
    RightError = DesiredRightRPM - ActualLeftRPM;
    
    // Integral of error
    LeftErrorSum += LeftError;
    RightErrorSum += RightError;
    
    // Calculate according to PI Law
    LeftDutyCycle = Kp*((LeftError)+(Ki*LeftErrorSum)); 
    RightDutyCycle = Kp*((RightError)+(Ki*RightErrorSum)); 
    
    // Anti-Windup
    if (LeftDutyCycle > 100) {
        LeftDutyCycle = 100;
        LeftErrorSum -= LeftError;
    } else if (LeftDutyCycle < 0) {
        LeftDutyCycle = 0;
        LeftErrorSum -= LeftError;
    }
    
    if (RightDutyCycle > 100) {
        RightDutyCycle = 100;
        RightErrorSum -= RightError;
    } else if (RightDutyCycle < 0) {
        RightDutyCycle = 0;
        RightErrorSum -= RightError;
    }
    
    // Lastly, Set the duty cycle of the motors by updating Output Compare
    if (LeftDirection == Backward) {
        LeftDutyCycle = 100 - LeftDutyCycle;
    }
    OC1RS = (OC_PERIOD + 1)/100 * LeftDutyCycle;
    
    if (RightDirection == Backward) {
        RightDutyCycle = 100 - RightDutyCycle;
    }
    OC2RS = (OC_PERIOD + 1)/100 * RightDutyCycle;
}

/****************************************************************************
 Function
    T3Handler

 Description
   Increments the rollover bit of the timer since maxed out the timer
****************************************************************************/
void __ISR(_TIMER_3_VECTOR, IPL7SRS) T3Handler(void)
{
    __builtin_disable_interrupts(); // Disable interrupts globally
    
    if (IFS0bits.T3IF) {
        MyTimer.TimeStruct.RolloverBits += 1; // Increment rollover counter
        IFS0CLR = _IFS0_T3IF_MASK;
    }
    
    __builtin_enable_interrupts(); // Enable interrupts globally
}
