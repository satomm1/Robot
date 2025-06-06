/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef MotorFSM_H
#define MotorFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Motor, MotorWait
}MotorState_t;

typedef struct
{
    uint16_t TimerBits;
    uint16_t RolloverBits;
} TimerByBytes_t;

typedef union
{
    TimerByBytes_t TimeStruct;
    uint32_t FullTime;
} MotorTimer_t;

typedef enum
{
    Forward,
    Backward
} Direction_t;

// Public Function Prototypes

bool InitMotorSM(uint8_t Priority);
bool PostMotorSM(ES_Event_t ThisEvent);
ES_Event_t RunMotorSM(ES_Event_t ThisEvent);
MotorState_t QueryMotorSM(void);
void SetDesiredRPM(uint16_t LeftRPM, uint16_t RightRPM);
void SetDesiredSpeed(float LinearVelocity, float AngularVelocity);
void MultiplyDesiredSpeed(float Factor);
void WritePositionToSPI(uint8_t *Message2Send);
void WriteDeadReckoningVelocityToSPI(uint8_t *Message2Send);
void ResetPosition(void);
void SetPosition(float x_set, float y_set, float theta_set);
void PrintBufferSize(void);
#endif /* MotorFSM_H */

