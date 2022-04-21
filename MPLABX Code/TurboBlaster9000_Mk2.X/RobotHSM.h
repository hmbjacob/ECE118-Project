#ifndef HSM_ROBOT_H
#define HSM_ROBOT_H

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

uint8_t InitRobotHSM(uint8_t Priority);

uint8_t PostRobotHSM(ES_Event ThisEvent);

ES_Event RunRobotHSM(ES_Event ThisEvent);

#endif /* HSM_ROBOT_H */

