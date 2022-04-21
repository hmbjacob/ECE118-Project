/*
 *  Author: Jacob McClellan
 *  11/9/21
 *  PingSensorFSM.c
 *  Ping Sensor state machine overview
 */

#ifndef FSM_Ping_H  // <- This should be changed to your own guard on both
#define FSM_Ping_H  //    of these lines


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/



/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

uint8_t InitPingFSM(uint8_t Priority);

uint8_t PostPingFSM(ES_Event ThisEvent);

ES_Event RunPingFSM(ES_Event ThisEvent);

#endif /* FSM_Template_H */

