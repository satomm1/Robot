/****************************************************************************

  Header file for LED service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef LEDServ_H
#define LEDServ_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitLEDService(uint8_t Priority);
bool PostLEDService(ES_Event_t ThisEvent);
ES_Event_t RunLEDService(ES_Event_t ThisEvent);

#endif /* LEDServ_H */

