/* 
 * File:   TemplateEventChecker.h
 * Author: Gabriel Hugh Elkaim
 *
 * Template file to set up typical EventCheckers for the  Events and Services
 * Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the
 * names will have to be changed to match your code.
 *
 * This EventCheckers file will work with simple services, FSM's and HSM's.
 *
 * Remember that EventCheckers should only return TRUE when an event has occured,
 * and that the event is a TRANSITION between two detectable differences. They
 * should also be atomic and run as fast as possible for good results.
 *
 * This is provided as an example and a good place to start.
 *
 * Created on September 27, 2013, 8:37 AM
 * Modified on September 12, 2016, 7:59 PM
 */

#ifndef PROJECTEVENTCHECKER_H
#define	PROJECTEVENTCHECKER_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT
#include "BOARD.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function TemplateCheckBattery(void)
 * @param none
 * @return TRUE or FALSE
 * @brief This function is a prototype event checker that checks the battery voltage
 *        against a fixed threshold (#defined in the .c file). Note that you need to
 *        keep track of previous history, and that the actual battery voltage is checked
 *        only once at the beginning of the function. The function will post an event
 *        of either BATTERY_CONNECTED or BATTERY_DISCONNECTED if the power switch is turned
 *        on or off with the USB cord plugged into the Uno32. Returns TRUE if there was an 
 *        event, FALSE otherwise.
 * @note Use this code as a template for your other event checkers, and modify as necessary.
 * @author Gabriel H Elkaim, 2013.09.27 09:18
 * @modified Gabriel H Elkaim/Max Dunne, 2016.09.12 20:08 */
uint8_t TemplateCheckBattery(void);

// The following event checkers are related to the PING sensor

/*
 * Checks if the pin designated for the echo input has an edge event. If so,
 * it will post an event of either ECHO_RISE or ECHO_fall
 * These events will be used to deal with the ping sensor logic
 */
uint8_t EchoEdgeDetection(void);

// THIS IS THE MAIN CHECK TAPE SENSORS EVENT CHECKER
// Will run through each tape sensor and compare it against the set hysteresis bounds and previous states.
// Will post a single event if there is any change, and send the current state of all 7 as a param.
uint8_t CheckTapeSensors(void);

/// @brief Detects a track wire event.
// the track wire is on pin V4
// and its voltage is positively correlated with intensity
uint8_t CheckTrackWire(void);

/*
 * This Bumper Event checker will check the current states of the bumpers, compare it to the previous states
 * and post an event if any of the bumpers have been pressed since the last check
 */
uint8_t BumperDetection(void);

/*
 * Posts a beacon detect or beacon lost event if there is a change in value across the hysteresis thresholds
 */
uint8_t BeaconDetection(void);




#endif	/* TEMPLATEEVENTCHECKER_H */

