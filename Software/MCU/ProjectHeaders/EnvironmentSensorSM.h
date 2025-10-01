/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef EnvironmentSensor_FSM_H
#define EnvironmentSensor_FSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_Env, Idle_Env_T_RH, T_RH_Meas, Idle_Env_Air, Air_Meas
}EnvironmentSensorState_t;

typedef enum {
    I2C_ST_IDLE = 0,
    I2C_ST_START,
    I2C_ST_ADDR_W,
    I2C_ST_REG,
    I2C_ST_RESTART,
    I2C_ST_ADDR_R,
    I2C_ST_RECV,
    I2C_ST_ACK,
    I2C_ST_STOP,
    I2C_ST_DONE,
    I2C_ST_ERROR
} I2C2_Stage;

typedef struct {
    volatile I2C2_Stage stage;
    volatile uint8_t dev7;
    volatile bool command_wait;
    volatile uint32_t command_wait_time; // In ms
    volatile uint16_t command_len;
    volatile uint8_t *reg;
    volatile uint16_t reg_idx;
    volatile uint8_t *buf;
    volatile uint16_t len;
    volatile uint16_t idx;
    volatile bool busy;
} I2C2_Trans;

// Public Function Prototypes

bool InitEnvironmentSensorSM(uint8_t Priority);
bool PostEnvironmentSensorSM(ES_Event_t ThisEvent);
ES_Event_t RunEnvironmentSensorSM(ES_Event_t ThisEvent);
EnvironmentSensorState_t QueryEnvironmentSensorSM(void);
#endif /* EnvironmentSensor_FSM_H */

