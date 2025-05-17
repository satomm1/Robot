/****************************************************************************

  Header file for Test Harness Service0
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef UsbService_H
#define UsbService_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"                // needed for definition of REENTRANT
// Public Function Prototypes

bool InitUsbService(uint8_t Priority);
bool PostUsbService(ES_Event_t ThisEvent);
ES_Event_t RunUsbService(ES_Event_t ThisEvent);

#endif /* UsbService_H */

