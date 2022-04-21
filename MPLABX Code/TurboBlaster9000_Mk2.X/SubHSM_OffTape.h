/*
 * SubHSM_OffTape.H
 * This file includes the three public functions for the overarching HSM.
 * This allows for the HSM to be initialized, ran and posted to.
 */

#ifndef SUB_OFFTAPE_HSM_H  // <- This should be changed to your own guard on both
#define SUB_OFFTAPE_HSM_H  //    of these lines


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

/*
 * Initializes the Off Tape Sub HSM. The first state that will be entered as a 
 * result is the 360 rotate before looking for the beacon
 */
uint8_t InitOffTapeSubHSM(void);

/*
 * Progresses the SubHSM state machine with events passed on by parent HSM
 */
ES_Event RunOffTapeSubHSM(ES_Event ThisEvent);

#endif /* SUB_HSM_OFFTAPE_H */

