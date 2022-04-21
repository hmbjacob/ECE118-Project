/*
 *  Author: Jacob McClellan
 *  11/9/21
 *  PingSensorFSM.c
 *  implementation of the Ping Sensors state machine to operate it
 * /


/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"

#include "PingSensorFSM.h"
#include <BOARD.h>
#include <stdio.h>
#include "IO_Ports.h"
#include "xc.h"
#include "Timers.h"
#include "Global_Macros.h"

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
// this function takes in the new deltaT variable obtained from the echo pins high time, 
// adds that time to the current running history, and returns the new running average
unsigned int updateHistory(unsigned int deltaT);
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/

static unsigned int timeHistory[5] = {0, 0, 0, 0, 0};

typedef enum {
    InitPState,
    PingHigh,
    WaitForEcho,
    EchoHigh,
    WaitForPing,

} PingFSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"PingHigh",
	"WaitForEcho",
	"EchoHigh",
	"WaitForPing",
};


static PingFSMState_t CurrentState = InitPState; // <- change enum name to match ENUM
static uint8_t MyPriority;
static int echoTime;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/


uint8_t InitPingFSM(uint8_t Priority)
{
    MyPriority = Priority;
    // put us into the Initial PseudoState
    CurrentState = InitPState;
    // post the initial transition event
    if (ES_PostToService(MyPriority, INIT_EVENT) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t PostPingFSM(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunPingFSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    PingFSMState_t nextState; // current state of the FSM

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
    case InitPState: // If current state is initial Pseudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            ES_Timer_InitTimer(PING_WAIT_TIMER, PING_WAIT_TICKS); // initialize the Ping wait timer
            ES_Timer_InitTimer(PING_HIGH_TIMER, PING_HIGH_TICKS); // initialize the Ping high timer

            // now put the machine into the actual initial state
            nextState = WaitForPing; // transition to wait for ping stage
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case PingHigh: // if Ping is high, wait for 1ms timer before lowering pin
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == PING_HIGH_TIMER) {// only respond to correct timer expired
            ES_Timer_StopTimer(PING_HIGH_TIMER); // Stop the timer for now until the cycle begins again
            IO_PortsClearPortBits(PING_PORT, TRIG_PIN); // Set the trig pin low

            // transition to wait for echo state
            nextState = WaitForEcho;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            ES_Timer_InitTimer(PING_WAIT_TIMER, PING_WAIT_TICKS); // reset the timer until FSM goes again
        }
        break;

    case WaitForEcho: // wait for the echo signal to go high, record time
        if (ThisEvent.EventType == ECHO_RISE) {
            echoTime = TIMERS_GetTime(); // record the time the rise goes high

            // transition to wait for echo high state
            nextState = EchoHigh;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        } else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == PING_WAIT_TIMER) {
            InitPingFSM(MyPriority); // emergency catch case if the echo pin ever fails
            //printf("Echo never high\r\n");
        }
        break;

    case EchoHigh: // wait for the echo signal to go low, find the delta time
        if (ThisEvent.EventType == ECHO_FALL) {
            // calculate and post the deltaT
            unsigned int deltaT = TIMERS_GetTime() - echoTime; // calculate delta time from recorded time
            unsigned int returnVal = updateHistory(deltaT);
            //printf("%d\r\n", returnVal); // prints the delta time, more logic needed to go here

            // transition to wait for ping state
            nextState = WaitForPing;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;


        } else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == PING_WAIT_TIMER) { // case where it times out while looking for echo fall
            //in this case, the ping sensor will put in arbitrary high value, and transition straight to the ping high state

            // set the high to 200, could be bigger, but higher numbers unneeded
            int deltaT = 200;
            unsigned int returnVal = updateHistory(deltaT);
            //printf("Out of Range: %d\r\n", returnVal);

            // transition to wait for ping state
            IO_PortsWritePort(PING_PORT, PIN3);
            ES_Timer_InitTimer(PING_HIGH_TIMER, PING_HIGH_TICKS);
            nextState = PingHigh;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    case WaitForPing: // wait for the system to ask to ping again
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == PING_WAIT_TIMER) {// only respond to correct timer expired
            IO_PortsWritePort(PING_PORT, PIN3); // Set the Ping pin high

            ES_Timer_InitTimer(PING_HIGH_TIMER, PING_HIGH_TICKS); // start the ping high timer

            // transition to ping high state
            nextState = PingHigh;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
        }
        break;

    default: // all unhandled states fall into here
        break;
    } // end switch on Current State
    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunPingFSM(EXIT_EVENT);
        CurrentState = nextState;
        RunPingFSM(ENTRY_EVENT);
    }
    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

// this function takes in the new deltaT variable obtained from the echo pins high time, 
// adds that time to the current running history, and returns the new running average

unsigned int updateHistory(unsigned int deltaT)
{
    static unsigned int sum = 0; // running sum
    sum = sum + deltaT - timeHistory[0]; // update sum amount
    for (int i = 0; i < 5; i++) {
        timeHistory[i] = timeHistory[i + 1]; // shift the history
    }
    timeHistory[4] = deltaT; // add the new data point to the history

    ES_Event thisEvent;
    //printf("here\r\n");
    thisEvent.EventType = NEW_PING;
    thisEvent.EventParam = sum/5;
    PostRobotHSM(thisEvent);

    return sum / 5; // return the new running average
}
