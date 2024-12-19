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
  InitPState_EEPROM, EEPROMWaiting, EEPROMWriteEnabled, EEPROMWriting
}EEPROMState_t;

// Public Function Prototypes

bool InitEEPROMSM(uint8_t Priority);
bool PostEEPROMSM(ES_Event_t ThisEvent);
ES_Event_t RunEEPROMSM(ES_Event_t ThisEvent);
EEPROMState_t QueryEEPROMSM(void);

void WriteEnable(void);
void WriteByteEEPROM(uint8_t data);
void WriteMultiBytesEEPROM(uint8_t *data, uint16_t N);
void ReadByteEEPROM(uint32_t address);
void ReadMultiBytesEEPROM(uint32_t address, uint16_t N);
void ReadStatusEEPROM(void);

#endif /* FSMEEPROM_H */

