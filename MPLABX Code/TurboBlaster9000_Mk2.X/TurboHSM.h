/*
 * TurboHSM.H
 * This file includes the three public functions for the overarching HSM.
 * This allows for the HSM to be initialized, ran and posted to.
 */

#ifndef HSM_Turbo_H
#define HSM_Turbo_H


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT


/**
 * @param Priority - internal variable to track which event queue to use
 * @brief Called to initialize the HSM. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTurboHSM function.
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitTurboHSM(uint8_t Priority);


/**
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @brief Use this function to post events to the HSM
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostTurboHSM(ES_Event ThisEvent);


/**
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief Contains the HSM.
 *        This function will be called recursively to implement the correct
 *        order for a state transition to be.
 */
ES_Event RunTurboHSM(ES_Event ThisEvent);

#endif /* HSM_Template_H */

