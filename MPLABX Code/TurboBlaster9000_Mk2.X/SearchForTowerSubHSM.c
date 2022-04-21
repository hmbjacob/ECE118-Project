

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotHSM.h"
#include "Motor_Control.h"
#include "SearchForTowerSubHSM.h"
#include "ResolveObstacleSubHSM.h"
#include "Global_Macros.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    AcquireTower,
    AlignTower,
    ApproachTower,
    ResolveObstacle,
    ReAdjust,
} SearchForTowerSubHSMState_t;

static const char *StateNames[] = {
	"InitPSubState",
	"AcquireTower",
	"AlignTower",
	"ApproachTower",
	"ResolveObstacle",
	"ReAdjust",
};

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/

static SearchForTowerSubHSMState_t CurrentState = InitPSubState; // first pseudo state
static uint8_t MyPriority; // subhsm process priority
static uint8_t turnDir; // the direction the feedback system is having the robot turn when approaching the tower
static uint32_t lastBeaconVal; // the last recorded value from the beacon
static uint8_t reAdjust = 0;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

uint8_t InitSearchForTowerSubHSM(void) {
    ES_Event returnEvent;
    CurrentState = InitPSubState;
    returnEvent = RunSearchForTowerSubHSM(INIT_EVENT); // run the init event through the subHSM
    if (returnEvent.EventType == ES_NO_EVENT) { // if there was no issue, return true
        return TRUE;
    }
    return FALSE;
}

ES_Event RunSearchForTowerSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    SearchForTowerSubHSMState_t nextState; // the state variable

    ES_Tattle(); // trace call stack

    switch (CurrentState) { // determine what to do based on current state
        case InitPSubState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                InitResolveObstacleSubHSM(); // only subHSM, used for resolving non-tower obstacles
                // now put the machine into the actual initial state
                nextState = AcquireTower; // go to the acquire tower state
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT; // consume the init event
            }
            break;

        case AcquireTower: // first state is the Acquire tower state
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("searching for tower tower\r\n");
                    ES_Timer_InitTimer(TURN_TIMER, TURN_360_TICKS); // initialize the timer used to let the robot do a full 360 rotate
                    SetMotors(ACQUIRE_SPEED, -ACQUIRE_SPEED); // let the robot spin in place
                    break;

                case ES_TIMEOUT: // if there is a timeout event
                    if (ThisEvent.EventParam == TURN_TIMER) { // if the turn timer expires
                        ES_Timer_InitTimer(TURN_TIMER, TURN_360_TICKS); // try again for now
                        ThisEvent.EventType = ES_NO_EVENT; // consume event
                    }
                    break;

                case BEACON_FOUND: // if a beacon is found
                    printf("acquired\r\n");
                    nextState = ApproachTower; // go to the acquire tower state
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT; // consume event
                    break;

                case BUMPED:
                    printf("Bumped Event while acquiring!!\r\n");
                    if (AD_ReadADPin(BEACON_A_PIN) < BEACON_CLOSE_THRESH) { // if there was a bumped event, and the robot is NOT suffiecently close to a tower, assume it hit an obstacle
                        //printf("collided with non tower object\r\n");
                        //nextState = ResolveObstacle;
                        //makeTransition = TRUE;
                        //ThisEvent.EventType = ES_NO_EVENT; // consume event
                    }
                    break;


                case ES_EXIT:
                    // on exit, turn off the turn timer, so that their events no longer clog up the HSM queue
                    ES_Timer_StopTimer(TURN_TIMER);
                    break;

                case ES_NO_EVENT:
                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case ApproachTower:
            
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("approaching\r\n");
                    ES_Timer_InitTimer(TURN_TIMER, APR_TIMEOUT);
                    lastBeaconVal = AD_ReadADPin(BEACON_A_PIN);
                    SetMotors(100, 60); // let the robot drive forward (80 left, 100 right)
                    turnDir = 1;
                    break;

                case BEACON_LOST: // if the beacon was lost, transition back to acquire tower subHSM
                    ES_Timer_InitTimer(TURN_TIMER, APR_TIMEOUT);
                    if (turnDir) {
                        SetMotors(APR_SPEED - APR_DIFF, APR_SPEED);
                        turnDir = 0;
                    } else {
                        SetMotors(APR_SPEED, APR_SPEED - APR_DIFF);
                        turnDir = 1;
                    }
                    //nextState = ReAdjust; // go to the acquire tower state
                    //makeTransition = TRUE;
                    //ThisEvent.EventType = ES_NO_EVENT; // consume the event
                    break;

                case BEACON_FOUND:
                    ES_Timer_InitTimer(TURN_TIMER, APR_TIMEOUT);
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == TURN_TIMER) { // if the turn timer expires
                        ES_Timer_StopTimer(TURN_TIMER);
                        nextState = AcquireTower; // go to the acquire tower state
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT; // consume the event
                    }
                    break;


                case BUMPED:
                    printf("Bumped Event while approaching!\r\n");
                    if (AD_ReadADPin(BEACON_A_PIN) < BEACON_CLOSE_THRESH) { // if there was a bumped event, and the robot is NOT suffiecently close to a tower, assume it hit an obstacle
                        printf("collided with non tower object\r\n");
                        nextState = ResolveObstacle;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT; // consume event
                    }
                    break;

                case TAPE_CHANGE: // if there was a change in the tape state
                    //if ((ThisEvent.EventParam & (FL_TAPE_BIT | FR_TAPE_BIT)) == 0) { // when driving forward only care about front tape, fixes some edge cases
                        nextState = ResolveObstacle; // need to resolve the tape detection if it has occurred
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT; // consume event
                    //}
                    
                    break;

                case ES_EXIT:
                    ES_Timer_StopTimer(TURN_TIMER);
                    break;

                default: // all unhandled states fall into here
                    break;
            }
            break;

        case ResolveObstacle: // if currently trying to resolve an obstacle

            ThisEvent = RunResolveObstacleSubHSM(ThisEvent); // pass events down to substate first
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    InitResolveObstacleSubHSM();
                    printf("resolving\r\n");
                    break;
                case ES_TIMEOUT: // if there is a timeout event
                    if (OBSTACLE_TIMER == ThisEvent.EventParam) { // and that event is the obstacle timer, and it was not consumed by the sub state machine
                        nextState = AcquireTower; // we can assume the drive state expired, and its time to reacquire the tower
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT; // consume event
                    }
                    break;
                case ES_EXIT:
                    printf("resolved\r\n");
                    SetMotors(0, 0); // turn off motors when exiting
                    ES_Timer_StopTimer(OBSTACLE_TIMER); // now stop the timer until next time to resolve
                    break;
                default: // all unhandled states fall into here
                    break;
            }

        default: // all unhandled states fall into here
            break;


    }

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunSearchForTowerSubHSM(EXIT_EVENT);
        CurrentState = nextState;
        RunSearchForTowerSubHSM(ENTRY_EVENT);
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

