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
  InitPState_Env, Idle_Env, SGP_Conditioning_Env, SGP_Meas_Env, 
          SHT_Meas_Env, SHT_Read_Env
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

typedef enum {
    Sensor, SerialNumber
} EnvSensData_t;

typedef enum {
    Write, Read
} I2CMessageType_t;

typedef struct {
    volatile I2C2_Stage stage;
    volatile uint8_t dev7;
    volatile I2CMessageType_t message_type;
    volatile bool command_wait;
    volatile uint32_t command_wait_time; // In ms
    volatile uint16_t command_len;
    volatile uint8_t *reg;
    volatile uint16_t reg_idx;
    volatile uint8_t *buf;
    volatile uint16_t len;
    volatile uint16_t idx;
    volatile EnvSensData_t datatype;
    volatile bool busy;
} I2C2_Trans;

// Public Function Prototypes

bool InitEnvironmentSensorSM(uint8_t Priority);
bool PostEnvironmentSensorSM(ES_Event_t ThisEvent);
ES_Event_t RunEnvironmentSensorSM(ES_Event_t ThisEvent);
EnvironmentSensorState_t QueryEnvironmentSensorSM(void);

float GetTemperature(void);
float GetHumidity(void);
int32_t GetVOCIndex(void);
int32_t GetNOXIndex(void);

void WriteTempHumidityToSPI(uint8_t *Message2Send);
void WriteAirQualityToSPI(uint8_t *Message2Send);

#endif /* EnvironmentSensor_FSM_H */

