/****************************************************************************

  Header file for Reflect service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ReflectServ_H
#define ReflectServ_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitReflectService(uint8_t Priority);
bool PostReflectService(ES_Event_t ThisEvent);
ES_Event_t RunReflectService(ES_Event_t ThisEvent);

#endif /* ReflectServ_H */

