/*
 * File:   TemplateEventChecker.c
 * Author: Gabriel Hugh Elkaim
 *
 * Template file to set up typical EventCheckers for the  Events and Services
 * Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the
 * names will have to be changed to match your code.
 *
 * This EventCheckers file will work with both FSM's and HSM's.
 *
 * Remember that EventCheckers should only return TRUE when an event has occured,
 * and that the event is a TRANSITION between two detectable differences. They
 * should also be atomic and run as fast as possible for good results.
 *
 * This file includes a test harness that will run the event detectors listed in the
 * ES_Configure file in the project, and will conditionally compile main if the macro
 * EVENTCHECKER_TEST is defined (either in the project or in the file). This will allow
 * you to check you event detectors in their own project, and then leave them untouched
 * for your project unless you need to alter their post functions.
 *
 * Created on September 27, 2013, 8:37 AM
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ProjectEventChecker.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "IO_Ports.h"
#include "PingSensorFSM.h"
#include "RobotHSM.h"
#include <stdio.h>
#include "Global_Macros.h"

/*******************************************************************************
 * EVENTCHECKER_TEST SPECIFIC CODE                                                             *
 ******************************************************************************/

//#define EVENTCHECKER_TEST
#ifdef EVENTCHECKER_TEST
#include <stdio.h>
#define SaveEvent(x) do {eventName=__func__; storedEvent=x;} while (0)

static const char *eventName;
static ES_Event storedEvent;
#endif

//#define BOTT_TAPE_ACTIVE

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this EventChecker. They should be functions
   relevant to the behavior of this particular event checker */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

/* Any private module level variable that you might need for keeping track of
   events would be placed here. Private variables should be STATIC so that they
   are limited in scope to this module. */



/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
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
uint8_t TemplateCheckBattery(void) {
    static ES_EventTyp_t lastEvent = BATTERY_DISCONNECTED;
    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;
    uint16_t batVoltage = AD_ReadADPin(BAT_VOLTAGE); // read the battery voltage

    if (batVoltage > BATTERY_DISCONNECT_THRESHOLD) { // is battery connected?
        curEvent = BATTERY_CONNECTED;
    } else {
        curEvent = BATTERY_DISCONNECTED;
    }
    if (curEvent != lastEvent) { // check for change from last time

        thisEvent.EventType = curEvent;
        thisEvent.EventParam = batVoltage;
        returnVal = TRUE;
        lastEvent = curEvent; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        //        PostGenericService(thisEvent);
#else
        SaveEvent(thisEvent);
#endif   
    }
    return (returnVal);
}

/*
 * Checks if the pin designated for the echo input has an edge event. If so,
 * it will post an event of either ECHO_RISE or ECHO_fall
 * These events will be used to deal with the ping sensor logic
 */
uint8_t EchoEdgeDetection(void) {
    static ES_EventTyp_t lastEvent = ECHO_FALL;
    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;


    uint16_t echoVal = (int) (IO_PortsReadPort(PING_PORT) & ECHO_PIN);


    if (echoVal > 0) { // is the bit high?
        curEvent = ECHO_RISE;
    } else { // the bit is low
        curEvent = ECHO_FALL;
    }
    if (curEvent != lastEvent) { // check for change from last time
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = echoVal;
        PostPingFSM(thisEvent);
        returnVal = TRUE;
        lastEvent = curEvent; // update history 
    }
    return (returnVal);
}

// THIS IS THE MAIN CHECK TAPE SENSORS EVENT CHECKER
// Will run through each tape sensor and compare it against the set hysteresis bounds and previous states.
// Will post a single event if there is any change, and send the current state of all 7 as a param.

uint8_t CheckTapeSensors(void) {


    ES_Event thisEvent; // the event to post later if there is a change in tape state
    static uint8_t lastState = 0; // the last recorded state of the tape sensors
    uint8_t currState = 0; // the current state of the tape sensors
    uint8_t returnVal = FALSE;

    // HANDLER FOR FL TAPE
    uint32_t tapeVal = AD_ReadADPin(FL_TAPE_PIN);
    if (tapeVal < LIGHT_THRESHOLD) currState |= FL_TAPE_BIT; // check light threshold, set the bit high
    else if (tapeVal < DARK_THRESHOLD) currState |= (lastState & FL_TAPE_BIT);

    // HANDLER FOR FR TAPE
    tapeVal = AD_ReadADPin(FR_TAPE_PIN);
    if (tapeVal < LIGHT_THRESHOLD) currState |= FR_TAPE_BIT; // check light threshold, set the bit high
    else if (tapeVal < DARK_THRESHOLD) currState |= (lastState & FR_TAPE_BIT);

    // HANDLER FOR BL TAPE
    tapeVal = AD_ReadADPin(BL_TAPE_PIN);
    if (tapeVal < LIGHT_THRESHOLD) currState |= BL_TAPE_BIT; // check light threshold, set the bit high
    else if (tapeVal < DARK_THRESHOLD) currState |= (lastState & BL_TAPE_BIT);

    // HANDLER FOR BR TAPE
    tapeVal = AD_ReadADPin(BR_TAPE_PIN);
    if (tapeVal < LIGHT_THRESHOLD) currState |= BR_TAPE_BIT; // check light threshold, set the bit high
    else if (tapeVal < DARK_THRESHOLD) currState |= (lastState & BR_TAPE_BIT);


    if (currState != lastState) {
        thisEvent.EventType = TAPE_CHANGE; // mark tape change event
        thisEvent.EventParam = currState; // set the state as the parameter
        lastState = currState; // update the history
#ifdef BOTT_TAPE_ACTIVE
        PostRobotHSM(thisEvent);
        //printf("Tape change FL %d\r\n", currState);
#endif
        returnVal = TRUE;

    }

    return returnVal;


}

/// @brief Detects a track wire event.
// the track wire is on pin V4
// and its voltage is positively correlated with intensity

uint8_t CheckTrackWire(void) {

    static ES_EventTyp_t lastEvent = TW_LOST;
    ES_EventTyp_t curEvent; // the current event/ state of the track wire (TW)
    uint8_t returnVal = FALSE; // will change to true if there is an event posted

    uint32_t curTWval = AD_ReadADPin(TW_PIN); // current AD value of the track wire

    if (curTWval > TW_HIGH_THRESH) curEvent = TW_DETECT; // track wire is currently detecting if above upper threshold
    else if (curTWval < TW_LOW_THRESH) curEvent = TW_LOST; // track wire is not detecting if below lower threshold
    else return returnVal; // if in between, the current event defaults to whatever the last event was, ignore
    /* ENDIF */
    if (curEvent != lastEvent) {
        ES_Event thisEvent;
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = curTWval;
        returnVal = TRUE;
        lastEvent = curEvent;
        //PostRobotHSM(ThisEvent);
    }
    return returnVal;
}

/*
 * This Bumper Event checker will check the current states of the bumpers, compare it to the previous states
 * and post an event if any of the bumpers have been pressed since the last check
 */
uint8_t BumperDetection(void) {
    static uint16_t bumperStates = 0; // the state of the four bumpers, each presenting a bit

    ES_EventTyp_t curEvent = BUMPED; // will only ever return the bumped event, which is what we care about
    uint8_t returnVal = FALSE; // will change to true if there is an event posted

    //uint8_t curState = (~(IO_PortsReadPort(BUMPER_PORT) >> 4)) & 0b1111;
    uint16_t curState = IO_PortsReadPort(BUMPER_PORT);
    //printf("%d\r\n", curState);
    //printf("OLD: FL %d,  FR %d, BL %d, BR %d\r\n", bumperStates & FL_BUMP_BIT, bumperStates & FR_BUMP_BIT, bumperStates & BL_BUMP_BIT, bumperStates & BR_BUMP_BIT);
    //printf("NEW: FL %d,  FR %d, BL %d, BR %d\r\n", curState & FL_BUMP_BIT, curState & FR_BUMP_BIT, curState & BL_BUMP_BIT, curState & BR_BUMP_BIT);

    if (bumperStates != curState) {
        //printf("here\r\n");
        if (((curState & FL_BUMP_BIT) < (bumperStates & FL_BUMP_BIT)) ||
                ((curState & FR_BUMP_BIT) < (bumperStates & FR_BUMP_BIT)) ||
                ((curState & BL_BUMP_BIT) < (bumperStates & BL_BUMP_BIT)) ||
                ((curState & BR_BUMP_BIT) < (bumperStates & BR_BUMP_BIT))) {
            ES_Event thisEvent;
            thisEvent.EventType = curEvent;
            thisEvent.EventParam = ~curState;
            PostRobotHSM(thisEvent);
            returnVal = TRUE;

        }


        bumperStates = curState;
    }
    return returnVal;
}

/*
 * Posts a beacon detect or beacon lost event if there is a change in value across the hysteresis thresholds
 */
uint8_t BeaconDetection(void) {
    static ES_EventTyp_t lastEvent = BEACON_LOST; // default is not detecting the beacon
    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;

    uint16_t beaconVal = (int) AD_ReadADPin(BEACON_A_PIN);

    if (beaconVal > BEACON_HIGH_THRESH) { // if the beacon value is above the analog threshold
        curEvent = BEACON_FOUND; // beacon high event      
    } else if (beaconVal < BEACON_LOW_THRESH) { // if the beacon value is below the analog threshold
        curEvent = BEACON_LOST; // beacon low event
    } else {
        curEvent = lastEvent;
    }
    if (curEvent != lastEvent) { // check for change from last time
        thisEvent.EventType = curEvent; // if there was a change, pass the new event
        thisEvent.EventParam = beaconVal;
        PostRobotHSM(thisEvent);
        returnVal = TRUE;
        lastEvent = curEvent; // update history 
    }
    return (returnVal);
}
