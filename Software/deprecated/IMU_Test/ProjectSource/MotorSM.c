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
#include <math.h>

/*----------------------------- Module Defines ----------------------------*/
#define IC_PERIOD 65535 // Input capture period
#define OC_PERIOD 312   // Output compare period (10 kHz)
#define CONTROL_PERIOD 1000 // Control update period --- 6250 Hz
#define NO_SPEED_PERIOD 65535 // Period to indicate motor not spinning
#define DEAD_RECKONING_PERIOD 1953 // 3906 // 7812 // Chosen so that we update at 50 Hz rate
#define Kp 3 // Proportional constant for PID law
#define Ki 0.5 // Integral constant for PID law
#define Kd 3 // Derivative constant for PID law

#define ENCODER_RESOLUTION 374 // Number of pulses per revolution
#define GEAR_RATIO 34 // Gear reduction ratio
#define SPEED_CONVERSION_FACTOR (1.6e7*60)/ENCODER_RESOLUTION
#define WHEEL_BASE 0.258572 // Distance between wheels on the robot (m)
#define WHEEL_RADIUS 0.04 // Radius of wheels (m))
#define DEAD_RECKONING_TIME 0.00499968 //0.00999936 //0.01999872 // Time between dead reckoning updates in seconds (depends on DEAD_RECKONING_PERIOD)
#define DEAD_RECKONING_RATIO 2*3.14159 / ENCODER_RESOLUTION / DEAD_RECKONING_TIME * WHEEL_RADIUS // This number times change in encoder clicks is linear velocity in m/second

#define V_MAX 1 // max 1 m/sec
#define w_MAX 2 // max 2 rad/sec
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static MotorState_t CurrentState;

// Everything we need for measuring motor speed
static volatile MotorTimer_t MyTimer;
static volatile uint32_t LeftPulseLength = 4294967295; 
static volatile uint32_t RightPulseLength = 4294967295; 
static volatile uint32_t LeftPrevTime = 0; 
static volatile uint32_t RightPrevTime = 0; 

// Used for dead reckoning to determine current position
static volatile int32_t LeftRotations = 0;
static volatile int32_t RightRotations = 0;
static volatile int32_t LeftPrevRotations = 0;
static volatile int32_t RightPrevRotations = 0;
static volatile float x = 0; // x position of the robot
static volatile float y = 0; // y position of the robot
static volatile float theta = 0; // angular position of the robot

// History variables to keep track of current V and w
static volatile float V_current = 0.;
static volatile float w_current = 0.;

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
  
  RPF2R = 0b1100; // Set RF2 -> OC1
  RPD5R = 0b1011; // Set RD5 -> OC2
  
  LATFbits.LATF8 = 0; // Start direction pins low
  LATJbits.LATJ3 = 0; // Start direction pins low
  
  // Set encoder pins and fault pins to digital inputs
  ANSELCCLR = _ANSELC_ANSC1_MASK | _ANSELC_ANSC4_MASK;
  TRISCSET = _TRISC_TRISC1_MASK | _TRISC_TRISC4_MASK;
  TRISDSET = _TRISD_TRISD0_MASK;
  TRISHSET = _TRISH_TRISH8_MASK;
  TRISASET = _TRISA_TRISA4_MASK;
  TRISJSET = _TRISJ_TRISJ12_MASK;
  
  IC1R = 0b0011; // Set IC1 -> RD0
  IC3R = 0b1010; // Set IC3 -> RC1
  
  // Set motor current pins to be analog inputs
  ANSELJSET = _ANSELJ_ANSJ9_MASK;
  ANSELASET = _ANSELA_ANSA1_MASK;
  TRISJSET = _TRISJ_TRISJ9_MASK;
  TRISASET = _TRISA_TRISA1_MASK;
      
  // Setup Timers
  // Timer 1 (for control update)
  T1CON = 0; // Reset the timer 1 register settings
  T1CONbits.TCKPS = 0b01; // 1:8 prescale value, 6.25 MHz
  T1CONbits.TCS = 0; // User internal peripheral clock (PBCLK3, 50 MHz)
  PR1 = CONTROL_PERIOD; // The amount of time we should do a control update (1000=6250 Hz, 500=12500 Hz)
  TMR1 = 0; // Set TMR1 to 0
  
  // Timer 2 (for Output Compare)
  T2CON = 0; // Reset the timer 2 register settings
  T2CONbits.TCKPS = 0b100; // 1:16 prescale value, 3.125 MHz
  T2CONbits.T32 = 0; // Use separate 16 bit timers
  T2CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR2 = OC_PERIOD; // Use output compare period (800 = 3906 Hz, 500=6250 Hz, 200=15625 Hz)
  TMR2 = 0; // Set TMR2 to 0
  
  // Timer 3 (For Input Capture)
  T3CON = 0; // Reset the timer 2 register settings
  T3CONbits.TCKPS = 0b011; // 1:8 prescale value, 6.25 MHz
  T3CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR3 = IC_PERIOD; // Use input capture period, just use full time
  TMR3 = 0; // Set TMR3 to 0
  
  // Timer 4 (For Left Motor No Speed)
  T4CON = 0;
  T4CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T4CONbits.T32 = 0; // Use separate 16 bit timers
  T4CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR4 = NO_SPEED_PERIOD; // Use no speed period
  TMR4 = 0; // Set TMR4 to 0
  
  // Timer 5 (For Right Motor No Speed)
  T5CON = 0;
  T5CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T5CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR5 = NO_SPEED_PERIOD; // Use no speed period, ~3 Hz @ 65535
  TMR5 = 0; // Set TMR5 to 0
  
  T7CON = 0;
  T7CONbits.TCKPS = 0b111; // 1:256 prescale value, 195.3125  kHz
  T7CONbits.TCS = 0; // Use internal peripheral clock (PBCLK3, 50 MHz)
  PR7 = DEAD_RECKONING_PERIOD;
  TMR7 = 0;
  
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
//  IC2CON = 0; // Reset IC2CON register settings 
  IC3CON = 0; // Reset IC3CON register settings 
//  IC4CON = 0; // Reset IC4CON register settings 
  IC1CONbits.ICTMR = 0; // User timery (timer3)
//  IC2CONbits.ICTMR = 0; // User timery (timer3)
  IC3CONbits.ICTMR = 0; // User timery (timer3)
//  IC4CONbits.ICTMR = 0; // User timery (timer3)
  IC1CONbits.ICI = 0b00; // Interrupt on every capture event
//  IC2CONbits.ICI = 0b00; // Interrupt on every capture event
  IC3CONbits.ICI = 0b00; // Interrupt on every capture event
//  IC4CONbits.ICI = 0b00; // Interrupt on every capture event
  IC1CONbits.ICM = 0b011; // Every rising edge mode
//  IC2CONbits.ICM = 0b001; // Every edge mode
  IC3CONbits.ICM = 0b011; // Every rising edge mode
//  IC4CONbits.ICM = 0b011; // Every edge mode
  
  // Setup Interrupts
  INTCONbits.MVEC = 1; // Use multivector mode
  PRISSbits.PRI7SS = 0b0111; // Priority 7 interrupt use shadow set 7
  PRISSbits.PRI6SS = 0b0110; // Interrupt with a priority level of 6 uses Shadow Set 6
  
  // Set interrupt priorities
  IPC1bits.IC1IP = 7; // IC1 Priority
  IPC1bits.IC1IS = 3; // IC1 Sub-priority
  IPC4bits.IC3IP = 7; // IC3
  IPC4bits.IC3IS = 3; // IC3 Sub-priority
  IPC1bits.T1IP = 7; // T1
  IPC1bits.T1IS = 2; // T1 Sub-priority
  IPC3bits.T3IP = 7; // T3
  IPC3bits.T3IS = 3; // T3 Sub-priority
  IPC4bits.T4IP = 6; // T4
  IPC6bits.T5IP = 6; // T5
  IPC8bits.T7IP = 6; // T7
  
  // Clear interrupt flags
  IFS0CLR = _IFS0_IC1IF_MASK | _IFS0_IC3IF_MASK | _IFS0_T1IF_MASK | 
          _IFS0_T3IF_MASK | _IFS0_T4IF_MASK | _IFS0_T5IF_MASK; // | _IFS0_IC2IF_MASK | _IFS0_IC4IF_MASK
  
  IFS1CLR = _IFS1_T7IF_MASK;
  
  // Local enable interrupts
  IEC0SET = _IEC0_IC1IE_MASK | _IEC0_IC3IE_MASK | _IEC0_T1IE_MASK | 
          _IEC0_T3IE_MASK | _IEC0_T4IE_MASK | _IEC0_T5IE_MASK; //_IEC0_IC2IE_MASK | _IEC0_IC4IE_MASK 
  
  IEC1SET = _IEC1_T7IE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
  // Turn Everything On
  IC1CONbits.ON = 1; // Turn input capture on
//  IC2CONbits.ON = 1; // Turn input capture on
  IC3CONbits.ON = 1; // Turn input capture on
//  IC4CONbits.ON = 1; // Turn input capture on
  OC1CONbits.ON = 1; // Turn OC1 on
  OC2CONbits.ON = 1; // Turn OC2 on
  T1CONbits.ON = 1; // Turn timer 1 on
  T2CONbits.ON = 1; // Turn timer 2 on
  T3CONbits.ON = 1; // Turn timer 3 on
  T4CONbits.ON = 1; // Turn timer 4 on
  T5CONbits.ON = 1; // Turn timer 5 on
  T7CONbits.ON = 1; // Turn timer 7 on
  
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
        LeftRotations = 0;
        RightRotations = 0;
        
        LeftPrevRotations = 0;
        RightPrevRotations = 0;
          
        // now put the machine into the actual initial state
        CurrentState = MotorWait;
        
        ES_Timer_InitTimer(MOTOR_TIMER, 200);
      }
    }
    break;

    case MotorWait:      
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  
        { 
          
        }
        break;
        
        
        case ES_TIMEOUT:
        {
            uint16_t left_rpm = SPEED_CONVERSION_FACTOR / LeftPulseLength;
            uint16_t right_rpm = SPEED_CONVERSION_FACTOR / RightPulseLength;
//            DB_printf("RPM: %d, %d \r\n", left_rpm, right_rpm);
//            DB_printf("Vel: %d\r\n", (uint16_t)(V_current*100));
//            DB_printf("w: %d\r\n", (uint16_t)(w_current*100));
//            DB_printf("x: %d\r\n", (uint16_t)(x*100));
//            DB_printf("y: %d\r\n", (uint16_t)(y*100));
//            DB_printf("theta: %d\r\n", (uint16_t)(theta*100));
//            DB_printf("LR: %d\r\n", LeftRotations);
//            DB_printf("RR: %d\r\n", RightRotations);
            
            ES_Timer_InitTimer(MOTOR_TIMER, 500);
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

/****************************************************************************
 Function
     SetDesiredRPM

 Parameters
     uint16_t LeftRPM: the desired left wheel RPM
     uint16_t RighRPM: the desired right wheel RPM

 Returns
     None

 Description
     Sets the desired RPM for the two motors. Assumes the direction pins are 
     already correctly set to have wheels moving in correct direction
****************************************************************************/
void SetDesiredRPM(uint16_t LeftRPM, uint16_t RightRPM)
{
  DesiredLeftRPM = LeftRPM;
  DesiredRightRPM = RightRPM;
}

/****************************************************************************
 Function
     SetDesiredRPM

 Parameters
     float V: the desired linear velocity
     float w: the desired angular velocity, rad/second

 Returns
     None

 Description
     Sets the desired RPM for the two motors
****************************************************************************/
void SetDesiredSpeed(float V, float w)
{
    // We turn off control for stopped to prevent jittering
    if (V==0 && w == 0) {
        
        // Turn control timer off
        T1CONbits.ON = 0; // Turn timer 1 off
        TMR1 = 0;
                
        // Manually set drive pins to stopped
        LATJbits.LATJ3 = 0; // Set direction pin forward
        LeftDirection = Forward;
        LATFbits.LATF8 = 0; // Set direction pin forward
        RightDirection = Forward;
        
        OC1RS = 0;
        OC2RS = 0;
        
        return;
    } else {
        T1CONbits.ON = 1; // Turn timer 1 back on
    }
    
    
    if (V > V_MAX) {
        V = V_MAX;
    } else if (V < -V_MAX) {
        V = -V_MAX;
    }
    
    if (w > w_MAX) {
        w = w_MAX;
    } else if (w < -w_MAX) {
        w = -w_MAX;
    }
    
    // Calculate the angular velocity of the left/right wheel to achieve 
    // desired linear/angular velocity of the robot
    float v_r = V / WHEEL_RADIUS;
    float w_r = WHEEL_BASE * w / 2 / WHEEL_RADIUS;
    float left_w = v_r - w_r; // (rad/sec)
    float right_w = v_r + w_r; // (rad/sec)
    
    // Convert to revolutions per minute
    left_w = left_w * 60 / 2 / 3.14159; // (rev/min)
    right_w = right_w * 60 / 2 / 3.14159; // (rev/min)
    
    // Set the direction pins to the H-Bridge to get correct forward or 
    // backward motion
    if (left_w  >= 0) {
        LATJbits.LATJ3 = 0; // Set direction pin forward
        LeftDirection = Forward;
    } else {
        LATJbits.LATJ3 = 1; // Set direction pin to backward
        LeftDirection = Backward;
        left_w = -left_w;
    }
    
    if (right_w >= 0) {
        
        LATFbits.LATF8 = 0; // Set direction pin forward
        RightDirection = Forward;
    } else {
        LATFbits.LATF8 = 1; // Set direction pin to backward
        RightDirection = Backward;
        right_w = - right_w;
    }
    
    // Last set the desired RPM variables
    SetDesiredRPM(left_w, right_w);
}

/****************************************************************************
 Function
     WritePositionToSPI

 Parameters
     uint32_t Buffer: the SPI buffer address to write the position data to

 Returns
     None

 Description
     Writes the current position data to the specified SPI buffer
****************************************************************************/
void WritePositionToSPI(uint8_t *Message2Send) {
  
  Message2Send[0] = 8; // 8 indicates we are position data (byte 1)
    
  // The x/y/theta data are all floats. The floats can be sent as 4 chunks of 
  // 8 bits.
  
  // Now write the x data (bytes 2-5)
  uint32_t x_as_int = *((uint32_t*)&x);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+1] = (x_as_int >> (24-8*j)) & 0xFF;
  }
  
  // Now write the y data (bytes 6-9)
  uint32_t y_as_int = *((uint32_t*)&y);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+5] = (y_as_int >> (24-8*j)) & 0xFF;
  }
  
  // Now write the theta data (bytes 10-13)
  uint32_t theta_as_int = *((uint32_t*)&theta);
  for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
    Message2Send[j+9] = (theta_as_int >> (24-8*j)) & 0xFF;
  }
  
  for (uint8_t j = 0; j < 3; j++) {
    Message2Send[j+13] = 0; // Fill rest of buffer with 0's
  }
}

void WriteDeadReckoningVelocityToSPI(uint8_t *Message2Send) {
    
    Message2Send[0] = 7; // 7 indcates the message type (byte 1)
    
    // the V/w data are floats. The floats can be sent as 4 chunks of 8 bits
    
    // Write V (bytes 2-5)
    uint32_t V_as_int = *((uint32_t*)&V_current);
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
        Message2Send[j+1] = (V_as_int >> (24-8*j)) & 0xFF;
    }
    
    // Write w (bytes 6-9)
    uint32_t w_as_int = *((uint32_t*)&w_current);
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
        Message2Send[j+5] = (w_as_int >> (24-8*j)) & 0xFF;
    }
    
    for (uint8_t j = 0; j < 7; j++) {
        Message2Send[j+9] = 0; // Fill rest of buffer with 0's
    }
}

void ResetPosition(void) {
    x = 0;
    y = 0;
    theta = 0;
}

/***************************************************************************
 private functions
 ***************************************************************************/

////////////////////// Interrupt Service Routines //////////////////////

/****************************************************************************
 Function
    IC1Handler

 Description
   Counts time between encoder pulses for measuring right motor pulse lengths
****************************************************************************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SRS) IC1Handler(void)
{
    static uint8_t ChannelB = 0; // static for speed
    
    ChannelB = PORTHbits.RH8;
    
    MyTimer.TimeStruct.TimerBits = (uint16_t)IC1BUF; // grab the captured time 
    IFS0CLR = _IFS0_IC1IF_MASK; // Clear the interrupt
    if (IFS0bits.T3IF && MyTimer.TimeStruct.TimerBits < 0x8000) {            
        MyTimer.TimeStruct.RolloverBits += 1; // increment the rollover counter
        IFS0CLR = _IFS0_T3IF_MASK; // clear the rollover interrupt   
    }                                                     
    
    // Calculate the time length between encoder pulses
    RightPulseLength = MyTimer.FullTime - RightPrevTime;         
    RightPrevTime = MyTimer.FullTime; // update our last time variable 
    
    // Update number of rotations for dead reckoning
    if (ChannelB) {
        RightRotations -= 1;
    } else {
        RightRotations += 1;
    }
    
    // restart Timer4 (timer to indicate if motor is stopped)
    T4CONCLR = _T4CON_ON_MASK;     
    TMR4 = 0;     
    T4CONSET = _T4CON_ON_MASK;     
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SRS) IC2Handler(void)
{
    // Probably don't need this here, we should just use this input to see if wheel moving forwards or backwards
}

/****************************************************************************
 Function
    IC3Handler

 Description
   Counts time between encoder pulses for measuring left motor pulse lengths
****************************************************************************/
void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL7SRS) IC3Handler(void)
{
    static uint8_t ChannelB = 0; // static for speed
    
    ChannelB = PORTCbits.RC4;
    
    MyTimer.TimeStruct.TimerBits = (uint16_t)IC3BUF; // grab the captured time 
    IFS0CLR = _IFS0_IC3IF_MASK; // Clear the interrupt
    if (IFS0bits.T3IF && MyTimer.TimeStruct.TimerBits < 0x8000) {            
        MyTimer.TimeStruct.RolloverBits += 1; // increment the rollover counter
        IFS0CLR = _IFS0_T3IF_MASK; // clear the rollover interrupt   
    }                                                     
    
    // Calculate the time length between encoder pulses
    LeftPulseLength = MyTimer.FullTime - LeftPrevTime;         
    LeftPrevTime = MyTimer.FullTime; // update our last time variable 
    
    // Update number of rotations for dead reckoning
    if (ChannelB) {
        LeftRotations += 1;
    } else {
        LeftRotations -= 1;
    }
    
    // restart Timer5 (timer to indicate if motor is stopped)
    T5CONCLR = _T5CON_ON_MASK;     
    TMR5 = 0;     
    T5CONSET = _T5CON_ON_MASK; 
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, IPL7SRS) IC4Handler(void)
{
    // Probably don't need this here, we should just use this input to see if wheel moving forwards or backwards
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
    static float LeftPrevError = 0.0;
    static float RightPrevError = 0.0;
    static float LeftErrorDiff;
    static float RightErrorDiff;
    static int16_t LeftDutyCycle; // Only static here for speed
    static int16_t RightDutyCycle; // Only static here for speed
    
    // Initialize variables used throughout the ISR (Static for speed)
    static uint16_t ActualLeftRPM;
    static uint16_t ActualRightRPM;
    
    IFS0CLR = _IFS0_T1IF_MASK; // Clear the timer interrupt
    
    // Calculate Current RPM based on Pulse Lengths from encoders
    ActualLeftRPM = SPEED_CONVERSION_FACTOR / LeftPulseLength;
    ActualRightRPM = SPEED_CONVERSION_FACTOR / RightPulseLength;
    
    // Calculate error from desired RPM
    LeftError = DesiredLeftRPM - ActualLeftRPM;
    RightError = DesiredRightRPM - ActualRightRPM;
    
//    DB_printf("%d, %d", (int16_t)LeftError, (int16_t)LeftErrorSum);
    
    // Integral of error
    LeftErrorSum += LeftError;
    RightErrorSum += RightError;
    
    // Derivative of Error
    LeftErrorDiff = LeftError - LeftPrevError;
    RightErrorDiff = RightError - RightPrevError;
    
    // Update for next time
    LeftPrevError = LeftError;
    RightPrevError = RightError;
    
    // Calculate according to PI Law
    LeftDutyCycle = Kp*LeftError + Ki*LeftErrorSum + Kd*LeftErrorDiff; 
    RightDutyCycle = Kp*RightError + Ki*RightErrorSum + Kd*RightErrorDiff;
    
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
    OC2RS = (OC_PERIOD + 1)/100 * LeftDutyCycle;
    
    if (RightDirection == Backward) {
        RightDutyCycle = 100 - RightDutyCycle;
    }
    OC1RS = (OC_PERIOD + 1)/100 * RightDutyCycle;
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

/****************************************************************************
 Function
    T4Handler

 Description
   If we reach here this means the left motor is not moving
****************************************************************************/
void __ISR(_TIMER_4_VECTOR, IPL6SRS) T4Handler(void)
{
    IFS0CLR = _IFS0_T4IF_MASK; // clear the interrupt flag     
    T4CONCLR = _T4CON_ON_MASK; // stop the timer 
    LeftPulseLength = 4294967295; // set LeftPulseLength to max    
}

/****************************************************************************
 Function
    T5Handler

 Description
   If we reach here this means the right motor is not moving
****************************************************************************/
void __ISR(_TIMER_5_VECTOR, IPL6SRS) T5Handler(void)
{
    IFS0CLR = _IFS0_T5IF_MASK; // clear the interrupt flag     
    T5CONCLR = _T5CON_ON_MASK; // stop the timer 
    RightPulseLength = 4294967295; // set RightPulseLength to max    
}

/****************************************************************************
 Function
    T7Handler

 Description
    Performs dead reckoning calculations
****************************************************************************/
// Dead reckoning via Runge Kutta to solve differential equations
//void __ISR(_TIMER_7_VECTOR, IPL6SRS) T7Handler(void)
//{
//    static float V_l; // Left wheel linear velocity (m/s)
//    static float V_r; // Right wheel linear velocity (m/s)
//    static float V;   // Robot linear velocity (m/s)
//    static float omega; // Robot angular velocity (rad/second)
//    
//    // Initialize Runge-Kutta Variables (static for speed)
//    static float k00, k01, k02;
//    static float k10, k11, k12;
//    static float k20, k21, k22;
//    static float k30, k31, k32;
//    
//    static int32_t CurLeftRotations;
//    static int32_t CurRightRotations;
//    
//    static bool status = false;
//    
//    IFS1CLR = _IFS1_T7IF_MASK; // clear the interrupt flag 
//    
//    // First thing we do is grab current number of rotations so this doesn't change mid function
//    CurLeftRotations = LeftRotations;
//    CurRightRotations = RightRotations;
//    
//    // Calculate linear velocity of both wheels    
////    DB_printf("L: %d\r\n", CurLeftRotations- LeftPrevRotations);
////    DB_printf("R: %d\r\n", CurRightRotations- RightPrevRotations);
//    
//    V_l = (CurLeftRotations - LeftPrevRotations) * DEAD_RECKONING_RATIO; 
//    V_r = (CurRightRotations - RightPrevRotations) * DEAD_RECKONING_RATIO;
//    
//    // Store for next time
//    LeftPrevRotations = CurLeftRotations;
//    RightPrevRotations = CurRightRotations;
//    
//    // Calculate linear/angular velocity of robot
//    V = (V_l + V_r) / 2; 
//    omega = (V_r - V_l) / WHEEL_BASE;
//    
//    V_current = V; // used to store current velocity
//    w_current = omega; // used to store current angular velocity
//    
//    // Now calculate the position of the robot using 4th order Runge Kutta
//    k00 = V * cosf(theta);
//    k01 = V * sinf(theta);
//    k02 = omega;
//    
//    k10 = V * cosf(theta + DEAD_RECKONING_TIME / 2 * k02);
//    k11 = V * sinf(theta + DEAD_RECKONING_TIME / 2 * k02);
//    k12 = omega;
//    
//    k20 = V * cosf(theta + DEAD_RECKONING_TIME / 2 * k12);
//    k21 = V * sinf(theta + DEAD_RECKONING_TIME / 2 * k12);
//    k22 = omega;
//    
//    k30 = V * cosf(theta + DEAD_RECKONING_TIME * k22);
//    k31 = V * sinf(theta + DEAD_RECKONING_TIME * k22);
//    k32 = omega;
//    
//    x = x + DEAD_RECKONING_TIME / 6 * (k00 + 2*(k10 + k20) + k30);
//    y = y + DEAD_RECKONING_TIME / 6 * (k01 + 2*(k11 + k21) + k31);
//    theta = theta + DEAD_RECKONING_TIME / 6 * (k02 + 2*(k12 + k22) + k32);
//    
//    // Ensure theta stays within [-pi, pi]
//    if (theta > 3.14159265359) {
//        theta -= 6.28318530718;
//    } else if (theta < -3.14159265359) {
//        theta += 6.28318530718;
//    }
//    
//}

// Using an exact method to solve the differential equations
void __ISR(_TIMER_7_VECTOR, IPL6SRS) T7Handler(void)
{
    static float V_l; // Left wheel linear velocity (m/s)
    static float V_r; // Right wheel linear velocity (m/s)
    static float V;   // Robot linear velocity (m/s)
    static float omega; // Robot angular velocity (rad/second)
    
    static int32_t CurLeftRotations;
    static int32_t CurRightRotations;
    
    static bool status = false;
    
    IFS1CLR = _IFS1_T7IF_MASK; // clear the interrupt flag 
    
    // First thing we do is grab current number of rotations so this doesn't change mid function
    CurLeftRotations = LeftRotations;
    CurRightRotations = RightRotations;
    
    // Next we calculate the linear velocity of each wheel
    V_l = (CurLeftRotations - LeftPrevRotations) * DEAD_RECKONING_RATIO; 
    V_r = (CurRightRotations - RightPrevRotations) * DEAD_RECKONING_RATIO;
    
    // Store the current number of rotations for next time
    LeftPrevRotations = CurLeftRotations;
    RightPrevRotations = CurRightRotations;
    
    // Calculate the instantaneous linear/angular velocity of robot
    V = (V_l + V_r) / 2; 
    omega = (V_r - V_l) / WHEEL_BASE;
    
    V_current = V; // used to store current velocity
    w_current = omega; // used to store current angular velocity
    
    // Calculate the update in theta and ensure theta stays within [-pi, pi]
    float prev_theta = theta;
    theta = theta + omega * DEAD_RECKONING_TIME;
    while (theta > M_PI) {
        theta -= 2*M_PI;
    }
    while (theta < -M_PI) {
        theta += 2*M_PI;
    }
    
    if (omega < 0.01) {
        x = x + V * cosf(prev_theta) * DEAD_RECKONING_TIME;
        y = y + V * sinf(prev_theta) * DEAD_RECKONING_TIME;
    } else {
        x = x + V/omega * (sinf(theta) - sinf(prev_theta));
        y = y - V/omega * (cosf(theta) - cosf(prev_theta));
    }
}