/****************************************************************************

  Header file for EEPROM Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FSMEEPROM_H
#define FSMEEPROM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState_EEPROM, EEPROMWaiting, EEPROMWriting,
  EEPROMReading
}EEPROMState_t;

typedef struct
{
    uint8_t Byte1;
    uint8_t Byte2;
    uint8_t Byte3;
} AddressByBytes_t;

typedef union 
{
    AddressByBytes_t AddressStruct;
    uint32_t FullAddress;
} EEPROM_Address_t;

// Public Function Prototypes

bool InitEEPROMSM(uint8_t Priority);
bool PostEEPROMSM(ES_Event_t ThisEvent);
ES_Event_t RunEEPROMSM(ES_Event_t ThisEvent);
EEPROMState_t QueryEEPROMSM(void);

#endif /* FSMEEPROM_H */

