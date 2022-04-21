
/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotHSM.h"
#include "ResolveObstacleSubHSM.h"
#include "Global_Macros.h"
#include "Motor_Control.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    FL_Resolve,
    FR_Resolve,
    BL_Resolve,
    BR_Resolve,
} ResolveObstacleSubHSMState_t;

static const char *StateNames[] = {
	"InitPSubState",
	"FL_Resolve",
	"FR_Resolve",
	"BL_Resolve",
	"BR_Resolve",
};


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */
void handleTape(ES_Event ThisEvent);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static ResolveObstacleSubHSMState_t CurrentState = InitPSubState; // <- change name to match ENUM
static uint8_t MyPriority;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitResolveObstacleSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateFSM function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitResolveObstacleSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = InitPSubState;
    returnEvent = RunResolveObstacleSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunResolveObstacleSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the heirarchical state
 *        machine, as this is called any time a new event is passed to the event
 *        queue. This function will be called recursively to implement the correct
 *        order for a state transition to be: exit current state -> enter next state
 *        using the ES_EXIT and ES_ENTRY events.
 * @note Remember to rename to something appropriate.
 *       The lower level state machines are run first, to see if the event is dealt
 *       with there rather than at the current level. ES_EXIT and ES_ENTRY events are
 *       not consumed as these need to pass pack to the higher level state machine.
 * @author J. Edward Carryer, 2011.10.23 19:25
 * @author Gabriel H Elkaim, 2011.10.23 19:25 */
ES_Event RunResolveObstacleSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    ResolveObstacleSubHSMState_t nextState; // <- change type to correct enum

    ES_Tattle(); // trace call stack



    switch (CurrentState) {
        case InitPSubState: // If current state is initial Psedudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                // find the current area of conflict
                if ((AD_ReadADPin(FL_TAPE_PIN) > DARK_THRESHOLD) || ((IO_PortsReadPort(BUMPER_PORT) & FL_BUMP_BIT) == 0)) {
                    nextState = FL_Resolve;
                } else if ((AD_ReadADPin(FR_TAPE_PIN) > DARK_THRESHOLD) || ((IO_PortsReadPort(BUMPER_PORT) & FR_BUMP_BIT) == 0)) {
                    nextState = FR_Resolve;
                } else if ((AD_ReadADPin(BL_TAPE_PIN) > DARK_THRESHOLD) || ((IO_PortsReadPort(BUMPER_PORT) & BL_BUMP_BIT) == 0)) {
                    nextState = BL_Resolve;
                } else if ((AD_ReadADPin(BR_TAPE_PIN) > DARK_THRESHOLD) || ((IO_PortsReadPort(BUMPER_PORT) & BR_BUMP_BIT) == 0)) {
                    nextState = BR_Resolve;
                }

                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;

        case FL_Resolve: // resolve the Front left collision state by backing away and 
            switch (ThisEvent.EventType) {
                case ES_ENTRY: // when entering this state do the following
                    printf("Entered FL Resolve\r\n");
                    SetLeftMotor(-RESOLVE_SPEED);
                    SetRightMotor(-(RESOLVE_SPEED-REVERSE_DIFF));
                    ES_Timer_InitTimer(OBSTACLE_TIMER, RESOLVE_TIME); // if this timer expires without incident, exit resolve
                    break;

                case BUMPED: // if there was a bumped event while in this state

                    if (AD_ReadADPin(BEACON_A_PIN) > BEACON_CLOSE_THRESH) break;
                    // identify which bumper was pressed and handle transfer to the new state accordingly
                    if ((ThisEvent.EventParam & FL_BUMP_BIT) > 0) {

                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_BUMP_BIT) > 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_BUMP_BIT) > 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_BUMP_BIT) > 0) {
                        nextState = BR_Resolve;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case TAPE_CHANGE: // if there was a tape event while in this state

                    // identify which tape sensor was triggered and change to the new state accordingly
                    if ((ThisEvent.EventParam & FL_TAPE_BIT) == 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_TAPE_BIT) == 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_TAPE_BIT) == 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_TAPE_BIT) == 0) {
                        nextState = BR_Resolve;
                    } else {
                        break;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case FR_Resolve: // resolve the Front left collision state by backing away and 
            switch (ThisEvent.EventType) {
                case ES_ENTRY: // when entering this state do the following
                    printf("Entered FR Resolve\r\n");
                    SetLeftMotor(-RESOLVE_SPEED);
                    SetRightMotor(-(RESOLVE_SPEED-REVERSE_DIFF));
                    //SetRightMotor(-RESOLVE_SPEED);
                    //SetLeftMotor(-(RESOLVE_SPEED-REVERSE_DIFF));
                    ES_Timer_InitTimer(OBSTACLE_TIMER, RESOLVE_TIME);
                    break;

                case BUMPED: // if there was a bumped event while in this state

                    if (AD_ReadADPin(BEACON_A_PIN) > BEACON_CLOSE_THRESH) break;
                    // identify which bumper was pressed and handle transfer to the new state accordingly
                    if ((ThisEvent.EventParam & FL_BUMP_BIT) > 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_BUMP_BIT) > 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_BUMP_BIT) > 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_BUMP_BIT) > 0) {
                        nextState = BR_Resolve;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case TAPE_CHANGE: // if there was a tape event while in this state

                    // identify which tape sensor was triggered and change to the new state accordingly
                    if ((ThisEvent.EventParam & FL_TAPE_BIT) == 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_TAPE_BIT) == 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_TAPE_BIT) == 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_TAPE_BIT) == 0) {
                        nextState = BR_Resolve;
                    } else {
                        break;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

            }
            break;

        case BL_Resolve: // resolve the Front left collision state by backing away and 
            switch (ThisEvent.EventType) {
                case ES_ENTRY: // when entering this state do the following
                    printf("Entered BL Resolve\r\n");
                    SetRightMotor(RESOLVE_SPEED-FORWARD_DIFF);
                    SetLeftMotor(RESOLVE_SPEED);
                    ES_Timer_InitTimer(OBSTACLE_TIMER, RESOLVE_TIME);
                    break;

                case BUMPED: // if there was a bumped event while in this state

                    if (AD_ReadADPin(BEACON_A_PIN) > BEACON_CLOSE_THRESH) break;
                    // identify which bumper was pressed and handle transfer to the new state accordingly
                    if ((ThisEvent.EventParam & FL_BUMP_BIT) > 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_BUMP_BIT) > 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_BUMP_BIT) > 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_BUMP_BIT) > 0) {
                        nextState = BR_Resolve;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case TAPE_CHANGE: // if there was a tape event while in this state

                    // identify which tape sensor was triggered and change to the new state accordingly
                    if ((ThisEvent.EventParam & FL_TAPE_BIT) == 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_TAPE_BIT) == 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_TAPE_BIT) == 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_TAPE_BIT) == 0) {
                        nextState = BR_Resolve;
                    } else {
                        break;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case BR_Resolve: // resolve the Front left collision state by backing away and 
            switch (ThisEvent.EventType) {
                case ES_ENTRY: // when entering this state do the following
                    printf("Entered BR Resolve\r\n");
                    SetLeftMotor(RESOLVE_SPEED-FORWARD_DIFF);
                    SetRightMotor(RESOLVE_SPEED);
                    ES_Timer_InitTimer(OBSTACLE_TIMER, RESOLVE_TIME);
                    break;

                case BUMPED: // if there was a bumped event while in this state

                    if (AD_ReadADPin(BEACON_A_PIN) > BEACON_CLOSE_THRESH) break;
                    // identify which bumper was pressed and handle transfer to the new state accordingly
                    if ((ThisEvent.EventParam & FL_BUMP_BIT) > 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_BUMP_BIT) > 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_BUMP_BIT) > 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_BUMP_BIT) > 0) {
                        nextState = BR_Resolve;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case TAPE_CHANGE: // if there was a tape event while in this state

                    // identify which tape sensor was triggered and change to the new state accordingly
                    if ((ThisEvent.EventParam & FL_TAPE_BIT) == 0) {
                        nextState = FL_Resolve;
                    } else if ((ThisEvent.EventParam & FR_TAPE_BIT) == 0) {
                        nextState = FR_Resolve;
                    } else if ((ThisEvent.EventParam & BL_TAPE_BIT) == 0) {
                        nextState = BL_Resolve;
                    } else if ((ThisEvent.EventParam & BR_TAPE_BIT) == 0) {
                        nextState = BR_Resolve;
                    } else {
                        break;
                    }
                    makeTransition = TRUE; // make the transition to the new state
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

            }
            break;


    }

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunResolveObstacleSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunResolveObstacleSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end

    return ThisEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

