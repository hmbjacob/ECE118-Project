

#ifndef SUB_NEW_HSM_H
#define SUB_NEW_HSM_H 


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

uint8_t InitFindNewTowerSubHSM(void);

ES_Event RunFindNewTowerSubHSM(ES_Event ThisEvent);

#endif /* SUB_NEW_TOWER_H */

