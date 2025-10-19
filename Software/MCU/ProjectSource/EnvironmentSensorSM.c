/****************************************************************************
 Module
   EnvironmentSensorSM.c

 Description
   This is a file for implementing reading from the Sensiron SGP-41 and SHT-40
   sensors (for air quality and temperature/humidity, respectively). The sensor 
   communicate via an I2C bus. The VOC and NOX measurements are processed by 
   Sensiron provided algorithms to output an air quality index. 

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include <sys/attribs.h>
#include "EnvironmentSensorSM.h"
#include "dbprintf.h"
#include "sensirion_gas_index_algorithm.h"

/*----------------------------- Module Defines ----------------------------*/
#define SHT4x_ADD 0x44
#define SGP41_ADD 0x59
#define I2C_WRITE 0
#define I2C_READ 1

#define CRC8_POLY  0x31  // x^8 + x^5 + x^4 + 1
#define CRC8_INIT  0xFF
#define CRC8_XOROUT 0x00

#define TWO_SIXTEEN 65535U

//#define TESTING
#define PRODUCTION

#define VERBOSE
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
bool I2C2_Busy(void);

// Functions for temp/humidity sensor
bool Get_SHT40_Serial_Num(uint8_t *buf);
bool PerformMeasurement_T_RH(bool heat, uint8_t *buf);
bool ReadMeasurement_T_RH(uint8_t *buf);

// Functions for air quality sensor
bool Get_SGP_Serial_Num(uint8_t *buf);
bool Condition_AirQuality(uint8_t *buf);
bool Get_AirQuality(uint8_t *buf, bool use_measured_t_rh);
bool SelfTest_AirQuality(uint8_t *buf);
bool TurnHeaterOff_AirQuality(uint8_t *buf);

// CRC Checksum functions
uint8_t crc8_poly31_ff( uint8_t *data, uint16_t len);
bool crc8_valid_residue( uint8_t data[3]);

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

// Gas sensor/index values
static int32_t voc_raw_value; // read from sensor
static int32_t voc_index_value; // computed from gas index algorithm
static int32_t nox_raw_value; // read from sensor 
static int32_t nox_index_value; // computed from gas index algorithm

// Gas index algorithm parameters
static GasIndexAlgorithmParams voc_params;
static GasIndexAlgorithmParams nox_params;

// Temperature and humidity values
static uint32_t temperature_raw_value;
static float temperature = 70;
static uint32_t humidity_raw_value;
static float relative_humidity = 50;

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
        CurrentState = Idle_Env;
        
        // Turn I2C2 On to allow I2C operations
        I2C2CONbits.ON = 1;
        
        // Initialize the gas index algorithms
        GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
        GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

        ES_Timer_InitTimer(ENV_TIMER, 1000);
      }
    }
    break;

    case Idle_Env:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
#ifdef TESTING
            if (ThisEvent.EventParam == ENV_TIMER) {
//                Get_SHT40_Serial_Num(T_RH_Buffer);
                Get_SGP_Serial_Num(T_RH_Buffer);
                
//                SelfTest_AirQuality(T_RH_Buffer);
            }     
            
            if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }
#endif
            
#ifdef PRODUCTION
            if (ThisEvent.EventParam == ENV_TIMER) {
                
                CurrentState = SGP_Conditioning_Env; 
#ifdef VERBOSE
                DB_printf("\r\nIn SGP_Conditioning_Env\r\n");
#endif
                ES_Timer_InitTimer(ENV_TIMER, 9500);
                Condition_AirQuality(AirQuality_Buffer);
                
            }     
            
            if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }
#endif
        }
        break;
        
        case EV_I2C_COMPLETE:
        {
            // TODO: Process the data
            
            DB_printf("SHT40 Serial:\r\n %d\r\n", T_RH_Buffer[0]);
            DB_printf("%d\r\n", T_RH_Buffer[1]);
            DB_printf("%d\r\n", T_RH_Buffer[2]);
            DB_printf("%d\r\n", T_RH_Buffer[3]);
            DB_printf("%d\r\n", T_RH_Buffer[4]);
            DB_printf("%d\r\n", T_RH_Buffer[5]);
            DB_printf("%d\r\n", T_RH_Buffer[6]);
            DB_printf("%d\r\n", T_RH_Buffer[7]);
            DB_printf("%d\r\n", T_RH_Buffer[8]);
                        
            bool first_word_valid = crc8_valid_residue(&T_RH_Buffer[0]);
            bool second_word_valid = crc8_valid_residue(&T_RH_Buffer[3]);
            bool third_word_valid = crc8_valid_residue(&T_RH_Buffer[6]);
            
            if (first_word_valid) {
                DB_printf("First word valid\r\n");
            } else {
                DB_printf("First word not valid\r\n");
            }
            
            if (second_word_valid) {
                DB_printf("Second word valid\r\n");
            } else {
                DB_printf("Second word not valid\r\n");
            }
            
            if (third_word_valid) {
                DB_printf("Third word valid\r\n");
            } else {
                DB_printf("Third word not valid\r\n");
            }
        }
        break;


        default:
          ;
      }  
    }
    break;
    
    case SGP_Conditioning_Env:
    {
        switch (ThisEvent.EventType) {
        
            case ES_TIMEOUT: 
            {  
                if (ThisEvent.EventParam == ENV_TIMER) {
                    CurrentState = SGP_Meas_Env;
#ifdef VERBOSE
                    DB_printf("\r\nIn SGP_Meas_Env\r\n");
#endif
                    Get_AirQuality(AirQuality_Buffer, true);
                }
            }
            break;
            
            default:
             ;
        
        }
    }
    break;
    
    case SGP_Meas_Env:
    {
        switch (ThisEvent.EventType) {
        
            case ES_TIMEOUT: 
            {  
                if (ThisEvent.EventParam == ENV_TIMER) {
                    CurrentState = SHT_Meas_Env;
#ifdef VERBOSE
                    DB_printf("\r\nIn SHT_Meas_Env\r\n");
#endif
                    PerformMeasurement_T_RH(false, T_RH_Buffer);
                } else if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                    I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
                }
            }
            break;
            
            case EV_I2C_COMPLETE:
            {
                // Process the measurement data
                
                // Verify checksum
                bool voc_valid = crc8_valid_residue(&AirQuality_Buffer[0]);
                bool nox_valid = crc8_valid_residue(&AirQuality_Buffer[3]);
                
                if (voc_valid) {
#ifdef VERBOSE
                    DB_printf("\r\nVOC is valid\r\n");
#endif
                    voc_raw_value = (uint16_t)AirQuality_Buffer[0] << 8 | (uint16_t)AirQuality_Buffer[1];
                    GasIndexAlgorithm_process(&voc_params, voc_raw_value, &voc_index_value);
#ifdef VERBOSE
                    DB_printf("VOC Raw Value:%d\r\n", voc_raw_value);
                    DB_printf("VOC Index:%d\r\n", voc_index_value);
#endif
                }
                
                if (nox_valid) {
#ifdef VERBOSE
                    DB_printf("NOX is valid\r\n");
#endif
                    nox_raw_value = (uint16_t)AirQuality_Buffer[3] << 8 | (uint16_t)AirQuality_Buffer[4];
                    GasIndexAlgorithm_process(&nox_params, nox_raw_value, &nox_index_value);
#ifdef VERBOSE
                    DB_printf("NOX Raw Value:%d\r\n", nox_raw_value);
                    DB_printf("NOX Index:%d\r\n", nox_index_value);
#endif
                }
                
                // Timer to advance to temp/humidity measurement
                ES_Timer_InitTimer(ENV_TIMER, 450);
            }
            break;
            
            case EV_I2C_ERROR:
            {            
                // Measurement failed, set timer to advance to next stage
                ES_Timer_InitTimer(ENV_TIMER, 450);
            }
            break;
            
            default:
             ;
        
        }
    }    
    break;
    
    case SHT_Meas_Env:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_TIMER) {
                CurrentState = SHT_Read_Env;
#ifdef VERBOSE
                DB_printf("\r\nIn SHT_Read_Env\r\n");
#endif
                ReadMeasurement_T_RH(T_RH_Buffer);
            } else if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }
        }
        break;
        
        case EV_I2C_COMPLETE:
        {            
            ES_Timer_InitTimer(ENV_TIMER, 150);
        }
        break;
        
        case EV_I2C_ERROR:
        {            
            // Measurement failed, advance to next stage
            CurrentState = SHT_Read_Env;
#ifdef VERBOSE
            DB_printf("\r\nIn SHT_Read_Env (from error)\r\n");
#endif
            ES_Timer_InitTimer(ENV_TIMER, 150);
        }
        break;

        default:
          ;
      }  
    }
    break;
    
    case SHT_Read_Env:   
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: 
        {   
            if (ThisEvent.EventParam == ENV_TIMER) {
                CurrentState = SGP_Meas_Env; 
#ifdef VERBOSE
                DB_printf("\r\nIn SGP_Meas_Env\r\n");
#endif
                Get_AirQuality(AirQuality_Buffer, true);
            } else if (ThisEvent.EventParam == ENV_WAIT_TIMER) {
                I2C2CONbits.RSEN = 1; // Time to perform the Repeated Start
            }          
        }
        break;
        
        case EV_I2C_COMPLETE:
        {            
            // First check that data is valid by checking checksum
            bool temp_valid = crc8_valid_residue(&T_RH_Buffer[0]);
            bool rh_valid = crc8_valid_residue(&T_RH_Buffer[3]);
            
            // Next convert raw values to actual temperature/humidity
            if (temp_valid) {
#ifdef VERBOSE
                DB_printf("\r\nTemperature is valid\r\n");
#endif
                temperature_raw_value = (uint16_t)T_RH_Buffer[0] << 8 | (uint16_t)T_RH_Buffer[1];
                temperature = -49 + 315 * ((float)temperature_raw_value) / TWO_SIXTEEN; // in Fahrenheit
                
#ifdef VERBOSE
                DB_printf("The temperature raw value is: %d\r\n", temperature_raw_value);
                DB_printf("The temperature is: %d\r\n\r\n", (uint16_t)temperature);
#endif
            }
            
            if (rh_valid) {
#ifdef VERBOSE
                DB_printf("\r\nHumidity is valid\r\n");
#endif
                humidity_raw_value = (uint16_t)T_RH_Buffer[3] << 8 | (uint16_t)T_RH_Buffer[4];
                relative_humidity = -6 + 125 * ((float)humidity_raw_value) / TWO_SIXTEEN;
#ifdef VERBOSE
                DB_printf("The relative humidity raw value is: %d\r\n", humidity_raw_value);
                DB_printf("The relative humidity is: %d\r\n\r\n", (uint16_t)relative_humidity);
#endif
            }
            
            ES_Timer_InitTimer(ENV_TIMER, 350);
        }
        break;
        
        case EV_I2C_ERROR:
        {                        
            ES_Timer_InitTimer(ENV_TIMER, 350); // Timer to advance to next stage
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

float GetTemperature(void) {
    return temperature;
}

float GetTemperatureCelsius(void) {
    return (temperature-32)*5/9;
}

float GetHumidity(void) {
    return relative_humidity;
}

int32_t GetVOCIndex(void) {
    return voc_index_value;
}

int32_t GetNOXIndex(void) {
    return nox_index_value;
}

/****************************************************************************
 Function
     WriteTempHumidityToSPI

 Parameters
     uint8_t *Message2Send: the SPI buffer address to write the temperature
                            and humidity data to

 Returns
     None

 Description
     Writes the current temperature and humidity data to the specified SPI buffer
****************************************************************************/
void WriteTempHumidityToSPI(uint8_t *Message2Send) {
    Message2Send[0] = 6; // 6 indicates we are temp/rh data (byte 1)
    
    // The temperature and humidity data are both floats. The floats can be sent 
    // as 4 chunks of 8 bits.

    // Now write the temperature data (bytes 2-5)
    uint32_t temp_as_int = *((uint32_t*)&temperature);
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
      Message2Send[j+1] = (temp_as_int >> (24-8*j)) & 0xFF;
    }

    // Now write the humidity data (bytes 6-9)
    uint32_t rh_as_int = *((uint32_t*)&relative_humidity);
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the float
      Message2Send[j+5] = (rh_as_int >> (24-8*j)) & 0xFF;
    }
    
    for (uint8_t j = 0; j < 7; j++) {
        Message2Send[j+9] = 0; // Fill rest of buffer with 0's
    }
}

/****************************************************************************
 Function
     WriteAirQualityToSPI

 Parameters
     uint8_t *Message2Send: the SPI buffer address to write the air quality
                            data to

 Returns
     None

 Description
     Writes the current VOC and NOX data to the specified SPI buffer
****************************************************************************/
void WriteAirQualityToSPI(uint8_t *Message2Send) {
    Message2Send[0] = 5; // 6 indicates we are air quality data (byte 1)
    
    // The air quality indices data are both int32_t's. The floats can be sent 
    // as 4 chunks of 8 bits.

    // Now write the VOC data (bytes 2-5)
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the int32
        Message2Send[j+1] = (voc_index_value >> (24-8*j)) & 0xFF; 
    }

    // Now write the NOX data (bytes 6-9)
    for (uint8_t j=0; j<4; j++) { // iterate through the 4, 8-bit chunks of the uint32
      Message2Send[j+5] = (nox_index_value >> (24-8*j)) & 0xFF;
    }
    
    for (uint8_t j = 0; j < 7; j++) {
        Message2Send[j+9] = 0; // Fill rest of buffer with 0's
    }
}


/***************************************************************************
 private functions
 ***************************************************************************/
bool I2C2_Busy(void) {
    return i2c2_t.busy;
}

bool Get_SHT40_Serial_Num(uint8_t *buf) {
    if (buf == NULL) {
        DB_printf("1\r\n");
        return false;
    } else if (i2c2_t.busy) {
        DB_printf("2\r\n");
        return false;
    } else if (!I2C2STATbits.P) {
//        DB_printf("3\r\n");
//        return false;
    }
    
    i2c2_t.dev7 = SHT4x_ADD;
    
    // Command length = 1 byte, change command depending on heat or no heat
    i2c2_t.message_type = Write; 
    i2c2_t.command_wait = true;
    i2c2_t.command_len = 1;
    commands[0] = 0x89;
    i2c2_t.command_wait_time = 105;
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0;
    
    i2c2_t.buf = buf;
    i2c2_t.len = 6;
    i2c2_t.idx = 0;
    i2c2_t.busy = true;
    i2c2_t.stage = I2C_ST_START;
    i2c2_t.datatype = SerialNumber;
    
    DB_printf("Getting Serial\r\n");
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool PerformMeasurement_T_RH(bool heat, uint8_t *buf) {
    // Check if I2C2 is busy or not ready yet
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    }
    
    i2c2_t.dev7 = SHT4x_ADD;
    
    // Command length = 1 byte, change command depending on heat or no heat
    i2c2_t.message_type = Write; 
    i2c2_t.command_wait = false;
    i2c2_t.command_len = 1;
    if (heat) {
        commands[0] = 0x15;
    } else {
        commands[0] = 0xFD;
    }
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0;
    
    i2c2_t.buf = buf;
    i2c2_t.len = 0;
    i2c2_t.idx = 0;
    i2c2_t.busy = true;
    i2c2_t.stage = I2C_ST_START;
    i2c2_t.datatype = Sensor;
    
#ifdef VERBOSE
    DB_printf("Starting SHT Measurement\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool ReadMeasurement_T_RH(uint8_t *buf) {
    // Start I2C Sequence for reading the temp/humidity from the SHT4x Sensor 
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        DB_printf("1\r\n");
        return false;
    } else if (i2c2_t.busy) {
        DB_printf("2\r\n");
        return false;
    } 
    
    i2c2_t.dev7 = SHT4x_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Read; // Perform a read action
    i2c2_t.command_wait = false; // Wait after providing a command
    i2c2_t.command_len = 0; // Number of bytes of command
    i2c2_t.command_wait_time = 0;  // How long to wait after command
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 6; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = Sensor; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Reading Measurement of SHT\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

///////////////// AIR QUALITY FUNCTIONS (SGP 41 ) /////////////////////////////
bool Get_SGP_Serial_Num(uint8_t *buf) {
    // Start I2C Sequence for obtaining the serial # of the SGP41 Sensor 
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        DB_printf("1\r\n");
        return false;
    } else if (i2c2_t.busy) {
        DB_printf("2\r\n");
        return false;
    } 
    
    i2c2_t.dev7 = SGP41_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Write; // Perform a write action
    i2c2_t.command_wait = true; // Wait after providing a command
    i2c2_t.command_len = 2; // Number of bytes of command
    commands[0] = 0x36; // Command to send
    commands[1] = 0x82;
    i2c2_t.command_wait_time = 105;  // How long to wait after command
    i2c2_t.reg = commands; 
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 9; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = SerialNumber; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Getting Serial of SGP\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool Get_AirQuality(uint8_t *buf, bool use_measured_t_rh) {
    // Starts the I2C Sequence for getting the raw VO2 and NOX data using the 
    // sgp41_measure_raw_signals command
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        return false;
        DB_printf("Error: 1");
    } else if (i2c2_t.busy) {
        DB_printf("Error: 2");
        return false;
    }
    
    i2c2_t.dev7 = SGP41_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Write; // Perform a write action
    i2c2_t.command_wait = true; // Wait after providing a command
    i2c2_t.command_len = 8; // Number of bytes of command
    commands[0] = 0x26; // Command to send
    commands[1] = 0x19;
    if (use_measured_t_rh) {
        uint8_t checksum_buf [2]; // buffer to get checksum values
        
        // Get relative humidity values and checksum
        uint16_t rh = (uint16_t)(GetHumidity()*65535/100);
        checksum_buf[0] = (uint8_t)(rh >> 8); // MSB
        checksum_buf[1] = (uint8_t)(rh & 0xFF); // LSB
        uint8_t rh_checksum = crc8_poly31_ff(checksum_buf, 2);
        
#ifdef VERBOSE
        DB_printf("rh_value: %d\r\n", rh);
#endif
        
        // Store 2 byte word and checksum in commands buffer
        commands[2] = checksum_buf[0];
        commands[3] = checksum_buf[1];
        commands[4] = rh_checksum;
        
        // Get temperature values and checksum
        uint16_t temp_c = (uint16_t)((GetTemperatureCelsius() + 45)*65535/175);
        checksum_buf[0] = (uint8_t)(temp_c >> 8); // MSB
        checksum_buf[1] = (uint8_t)(temp_c & 0xFF); // LSB
        uint8_t temp_c_checksum = crc8_poly31_ff(checksum_buf, 2);
        
#ifdef VERBOSE
        DB_printf("temp_value: %d\r\n", temp_c);
#endif
        
        // Store 2 byte word and checksum in commands buffer
        commands[5] = checksum_buf[0];
        commands[6] = checksum_buf[1];
        commands[7] = temp_c_checksum;   
    } else {
        // Use default values (corresponds to 25C, 50% RH)
        commands[2] = 0x80;
        commands[3] = 0x00;
        commands[4] = 0xA2;
        commands[5] = 0x66;
        commands[6] = 0x66;
        commands[7] = 0x93;    
    }
    i2c2_t.command_wait_time = 55; // How long to wait after command
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 6; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = Sensor; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Getting Raw Sensor Data from SGP\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool Condition_AirQuality(uint8_t *buf){
    // Starts the I2C Sequence for conditioning the sensor using the 
    // sgp41_execute_conditioning command
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    }
    
    i2c2_t.dev7 = SGP41_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Write; // Perform a write action
    i2c2_t.command_wait = false; // Wait after providing a command
    i2c2_t.command_len = 8; // Number of bytes of command
    commands[0] = 0x26; // Command to send
    commands[1] = 0x12;
    commands[2] = 0x80;
    commands[3] = 0x00;
    commands[4] = 0xA2;
    commands[5] = 0x66;
    commands[6] = 0x66;
    commands[7] = 0x93;    
    i2c2_t.command_wait_time = 0; // How long to wait after command
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 0; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = Sensor; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Conditioning the SGP\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}
            
bool SelfTest_AirQuality(uint8_t *buf) {
    // Starts the I2C Sequence for performing the self test using the 
    // sgp41_execute_self_test
    
    // The 2 least significant bits of the 2nd returned byte should be 0 to 
    // pass the self test
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    }
    
    i2c2_t.dev7 = SGP41_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Write; // Perform a write action
    i2c2_t.command_wait = true; // Wait after providing a command
    i2c2_t.command_len = 2; // Number of bytes of command
    commands[0] = 0x28; // Command to send
    commands[1] = 0x0E;
    i2c2_t.command_wait_time = 320; // How long to wait after command
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 3; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = Sensor; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Performing self test on the SGP\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}

bool TurnHeaterOff_AirQuality(uint8_t *buf) {
    // Starts the I2C Sequence for turning of the heater using the 
    // sgp41_turn_heater_off command
    
    // First check buffer exists and the i2c isn't busy
    if (buf == NULL) {
        return false;
    } else if (i2c2_t.busy) {
        return false;
    }
    
    i2c2_t.dev7 = SGP41_ADD; // Address of device (7 bits)
    i2c2_t.message_type = Write; // Perform a write action
    i2c2_t.command_wait = false; // Wait after providing a command
    i2c2_t.command_len = 2; // Number of bytes of command
    commands[0] = 0x36; // Command to send
    commands[1] = 0x15;  
    i2c2_t.command_wait_time = 0; // How long to wait after command
    i2c2_t.reg = commands;
    i2c2_t.reg_idx = 0; // Starting index for command
    
    i2c2_t.buf = buf; // Buffer for storing returned data from sensor 
    i2c2_t.len = 0; // Length of data expected from sensor (in bytes)
    i2c2_t.idx = 0; // Starting index for storing data in buffer
    i2c2_t.busy = true; // Indicate the I2C is busy
    i2c2_t.stage = I2C_ST_START; // Set in start stage
    i2c2_t.datatype = Sensor; // Set data type we are expecting
    
#ifdef VERBOSE
    DB_printf("Turning SGP Heater off\r\n");
#endif
    
    I2C2CONbits.SEN = 1;
    return true; // Successfully started the transmission sequence
}


///////////////////// CRC CHECKSUM FUNCTIONS //////////////////////////////////
uint8_t crc8_poly31_ff( uint8_t *data, uint16_t len) {
    // Creates a CRC Checksum on the provided data 
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

bool crc8_valid_residue( uint8_t data[3]) {
    // Checks if the checksum is correct
    // Input: uint8_t data[3]: first two values are data, thrid value is 
    //                         the checksum for the previous two values
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
            
            if (i2c2_t.message_type == Write) {
                // Start complete --> send device addr (write)
                I2C2TRN = (i2c2_t.dev7 << 1) | I2C_WRITE;
                i2c2_t.stage = I2C_ST_ADDR_W;
                DB_printf("In I2C_ST_START (write)\r\n");
            } else {
                // Start complete --> send device addr (read)
                I2C2TRN = (i2c2_t.dev7 << 1) | I2C_READ;
                i2c2_t.stage = I2C_ST_ADDR_R; // Skip the command writing
                DB_printf("In I2C_ST_START (read)\r\n");
            }
            
            break;

        case I2C_ST_ADDR_W:
            // Address(W) completed; check ACK
            if (I2C2STATbits.ACKSTAT) { 
                i2c2_t.stage = I2C_ST_ERROR; 
                DB_printf("Ack Error\r\n");
                break; 
            }
            
            DB_printf("In I2C_ST_ADDR_W\r\n");
            
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
            
            DB_printf("In I2C_ST_REG\r\n");
            
            if (i2c2_t.len == 0) {
                // Only command, not receiving any data
                I2C2CONbits.PEN = 1;
                i2c2_t.stage = I2C_ST_STOP;
            }
            else if (i2c2_t.command_wait) {
                // Start a timer for waiting
                ES_Timer_InitTimer(ENV_WAIT_TIMER, i2c2_t.command_wait_time);
                i2c2_t.stage = I2C_ST_RESTART;
            } else {
                // Immediately continue with Repeated start
                I2C2CONbits.RSEN = 1;
                i2c2_t.stage = I2C_ST_RESTART;
            }
            break;

        case I2C_ST_RESTART:
            // Restart complete --> send device addr (read)
            DB_printf("In I2C_ST_RESTART\r\n");
            
            I2C2TRN = (i2c2_t.dev7 << 1) | I2C_READ;
            i2c2_t.stage = I2C_ST_ADDR_R;
            break;

        case I2C_ST_ADDR_R:
            if (I2C2STATbits.ACKSTAT) { 
                DB_printf("Ack error (I2C_ST_ADDR_R)\r\n");
                i2c2_t.stage = I2C_ST_ERROR; 
                break; 
            }
            
            DB_printf("In I2C_ST_ADDR_R\r\n");
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
            
            DB_printf("In I2C_ST_RECV\r\n");
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
            DB_printf("In I2C_ST_STOP\r\n");
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