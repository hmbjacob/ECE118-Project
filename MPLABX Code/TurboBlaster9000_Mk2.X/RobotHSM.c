
/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotHSM.h"
#include "SearchForHoleSubHSM.h"
#include "SearchForTowerSubHSM.h"
#include "FindNewTowerSubHSM.h"
#include "AD.h"
#include "Global_Macros.h" // contains all of the macros
#include "Motor_Control.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/


typedef enum {
    InitPState,
    SearchForTower,
    SearchForHole,
    FindNewTower,

} RobotHSMState_t;

static const char *StateNames[] = {
	"InitPState",
	"SearchForTower",
	"SearchForHole",
	"FindNewTower",
};


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine
   Example: char RunAway(uint_8 seconds);*/
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static RobotHSMState_t CurrentState = InitPState;
static uint8_t MyPriority;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

uint8_t InitRobotHSM(uint8_t Priority) {
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

uint8_t PostRobotHSM(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunRobotHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    RobotHSMState_t nextState;

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case InitPState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                InitSearchForTowerSubHSM(); // initialize the Search for tower subHSM
                InitSearchForHoleSubHSM(); // initialize the Search for hole subHSM
                InitFindNewTowerSubHSM();

                nextState = SearchForTower; // first state is search for tower, need to find tower and then hole
                makeTransition = TRUE; // transition to the next state
                ThisEvent.EventType = ES_NO_EVENT; // clear the event
            }
            break;

        case SearchForTower: // if still searching for tower
            ThisEvent = RunSearchForTowerSubHSM(ThisEvent); // pass down to lower subhsm first
            switch (ThisEvent.EventType) {
                
                case ES_ENTRY:
                    InitSearchForTowerSubHSM();
                    break;

                case BUMPED: // if there is a bumped event that gets passed to this level
                    // we can assume that the resolve bump state determined that the robot hit the tower and passed the event back up
                    // transition to search for hole
                    if (AD_ReadADPin(BEACON_A_PIN) > BEACON_CLOSE_THRESH) {// only transition if close
                        makeTransition = TRUE;
                        nextState = SearchForHole;
                    }
                    ThisEvent.EventType = ES_NO_EVENT; // clear the event
                    SetMotors(0, 0);
                    break;

                default:
                    break;
            }
            break;

        case SearchForHole: // if searching for a hole within a tower
            ThisEvent = RunSearchForHoleSubHSM(ThisEvent); // pass it down to a lower level
            switch (ThisEvent.EventType) {

                case ES_ENTRY:
                    InitSearchForHoleSubHSM();
                    break;
                    
                case TOWER_LOST:
                    nextState = SearchForTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT; // clear the event
                    break;
                    

                case LAUNCH_COMPLETE: // if the robot has completed a ball launch
                    nextState = FindNewTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT; // clear the event
                    break;

                default:
                    break;
            }
            break;

        case FindNewTower:
            ThisEvent = RunFindNewTowerSubHSM(ThisEvent); // pass it down to a lower level
            switch (ThisEvent.EventType) {

                case ES_ENTRY:
                    InitFindNewTowerSubHSM();
                    break;

                case NEW_TOWER:
                    nextState = SearchForTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunRobotHSM(EXIT_EVENT); // post an exit event to itself to be handled
        CurrentState = nextState;
        RunRobotHSM(ENTRY_EVENT); // post an entry event
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
