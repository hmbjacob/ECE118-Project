

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotHSM.h"
#include "Motor_Control.h"
#include "FindNewTowerSubHSM.h"
#include "Global_Macros.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    ExitHole,
    Align,
    Drive,
    Turn,
    Forward,
    Pivot,
    Adjust,
} FindNewTowerSubHSMState_t;

static const char *StateNames[] = {
	"InitPSubState",
	"ExitHole",
	"Align",
	"Drive",
	"Turn",
	"Forward",
	"Pivot",
	"Adjust",
};

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/

static FindNewTowerSubHSMState_t CurrentState = InitPSubState; // first pseudo state
uint32_t pingData;
uint32_t myTime;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

uint8_t InitFindNewTowerSubHSM(void) {
    ES_Event returnEvent;
    CurrentState = InitPSubState;
    returnEvent = RunFindNewTowerSubHSM(INIT_EVENT); // run the init event through the subHSM
    if (returnEvent.EventType == ES_NO_EVENT) { // if there was no issue, return true
        return TRUE;
    }
    return FALSE;
}

ES_Event RunFindNewTowerSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    FindNewTowerSubHSMState_t nextState; // the state variable

    ES_Tattle(); // trace call stack

    switch (CurrentState) { // determine what to do based on current state
        case InitPSubState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                // now put the machine into the actual initial state
                nextState = ExitHole; // begin by exiting the hole
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT; // consume the init event
            }
            break;

        case ExitHole: // will be exiting the hole by briefly backing up before rotating to align
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(RESET_TIMER, EXIT_TICKS); // how long to back up for
                    SetLeftMotor(-EXIT_SPEED); // the speed at which to back up
                    SetRightMotor(-EXIT_SPEED);
                    break;

                case ES_TIMEOUT: // on timeout event
                    if (RESET_TIMER == ThisEvent.EventParam) { // if its the reset timer which we care about
                        nextState = Align; // go to the state where we align the bot with the tower
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        ES_Timer_StopTimer(RESET_TIMER); // stop the reset timer for now
                    }

                default:
                    break;
            }
            break;

        case Align: // now rotating to align with the tower before going in reverse
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(RESET_TIMER, ALIGN_TICKS);
                    SetLeftMotor(ALIGN_SPEED); // speed at which to align with the tower
                    SetRightMotor(-ALIGN_SPEED);
                    break;

                case ES_TIMEOUT: // on timeout event
                    if (RESET_TIMER == ThisEvent.EventParam) { // if its the reset timer which we care about
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        ES_Timer_InitTimer(RESET_TIMER, 900);
                    }

                    break;


                default:
                    break;
            }
            break;

        case Forward:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:

                    SetLeftMotor(-50); // the speed at which to back up
                    SetRightMotor(-50);
                    break;

                case ES_TIMEOUT: // on timeout event
                    if (RESET_TIMER == ThisEvent.EventParam) { // if its the reset timer which we care about
                        nextState = Pivot;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        ES_Timer_InitTimer(RESET_TIMER, PIVOT_TICKS); // how long to back up for
                    }
            }
            break;

        case Pivot:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:

                    SetLeftMotor(0); // the speed at which to back up
                    SetRightMotor(-100);
                    break;

                case ES_TIMEOUT: // on timeout event
                    if (RESET_TIMER == ThisEvent.EventParam) { // if its the reset timer which we care about
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        ES_Timer_InitTimer(RESET_TIMER, BACK_TICKS); // how long to back up for
                    }
                    break;

                case BEACON_FOUND:
                    if ((AD_ReadADPin(BEACON_A_PIN) > BEACON_HIGH_THRESH) && (AD_ReadADPin(BEACON_A_PIN) > 200)) { // wait some time before commiting to new tower
                        nextState = Adjust;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;


        case Adjust:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetLeftMotor(-ADJUST_SPEED);
                    SetRightMotor(ADJUST_SPEED); // turn speed
                    ES_Timer_InitTimer(RESET_TIMER, ADJUST_TICKS);
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == RESET_TIMER) {
                        ES_Event towerEvent;
                        towerEvent.EventType = NEW_TOWER;
                        PostRobotHSM(towerEvent);
                    }
                    break;
            }
            break;
    }

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunFindNewTowerSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunFindNewTowerSubHSM(ENTRY_EVENT);
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

