/*
 * TurboHSM.c
 * Implementation of the HSM that will run the Turbo Blaster 9000.
 * Very big and important file. Very!
 */


/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "TurboHSM.h"
#include "SubHSM_OffTape.h" //#include all sub state machines called
/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//Include any defines you need to do

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

// Timers defines
#define PING_HIGH_TICKS 500 // how long to back up for before re-evaluating

// States for the machine

typedef enum {
    InitPState,
    SearchTower,
    SearchHole,
} TurboHSM_State_t;

static const char *StateNames[] = {
    "InitPState",
    "OnTape",
    "OffTape",
};


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

// Helper function to detect whether any of the bottom tape sensors are active
uint8_t BottomTapeSensorsActive(void);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/

static TurboHSM_State_t CurrentState = InitPState; // THIS VARAIBLE IS THE CURRENT STATE OF THE HSM
static uint8_t MyPriority;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @param Priority - internal variable to track which event queue to use
 * @brief Called to initialize the HSM. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTurboHSM function.
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t InitTurboHSM(uint8_t Priority) {
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

/**
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @brief Use this function to post events to the HSM
 *        Returns TRUE if successful, FALSE otherwise
 */
uint8_t PostTurboHSM(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief Contains the HSM.
 *        This function will be called recursively to implement the correct
 *        order for a state transition to be.
 */
ES_Event RunTurboHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    TurboHSM_State_t nextState; // <- change type to correct enum

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case InitPState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {

                // Initialize all sub-state machines on this level
                InitSearchTowerSubHSM();

                nextState = SearchTower; // now put the machine into the actual initial state
                makeTransition = TRUE; // Tell the state machine to transition
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;

        case SearchTower: // State for being off the tape

            ThisEvent = PostSearchTowerSubHSM(ThisEvent); // pass event down to corresponding subHSM first, update event
            switch (ThisEvent.EventType) { // switch the event

                case FL_PRESS:
                case FR_PRESS:
                    nextState = SearchHole;
                    makeTransition = TRUE; // Tell the state machine to transition
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_ENTRY: // initialize the new relevant subHSM, will reset
                    InitSearchTowerSubHSM();
                    break;
                case ES_EXIT:
                    /* STOP MOTORS IMMEDIATELY */
                    break;

                default:// if no event or other event, do nothing
                    break;

            }
            break;
            
            case SearchTower:
                ThisEvent = PostSearchTowerSubHSM(ThisEvent); // pass event down to corresponding subHSM first, update event
            switch (ThisEvent.EventType) { // switch the event

                case LAUNCH_COMPLETE:
                case ROBOT_CLEARED:
                    nextState = SearchHole;
                    makeTransition = TRUE; // Tell the state machine to transition
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_ENTRY: // initialize the new relevant subHSM, will reset
                    InitSearchTowerSubHSM();
                    break;
                case ES_EXIT:
                    /* STOP MOTORS IMMEDIATELY */
                    break;

                default:// if no event or other event, do nothing
                    break;

            }
            break;

        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunTurboHSM(EXIT_EVENT); // let the HSM do anything it needs to when exiting a state
        CurrentState = nextState; // official change of state
        RunTurboHSM(ENTRY_EVENT); // let the HSM do anything it needs to when entering a state
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * Helper function to detect whether any of the bottom tape sensors are active
 */
uint8_t BottomTapeSensorsActive(void) {
    /* PLACE HOLDER FOR CODE */
    return 1;
}