/*
 * File: TemplateSubHSM.c
 * Author: J. Edward Carryer
 * Modified: Gabriel H Elkaim
 *
 * Template file to set up a Heirarchical State Machine to work with the Events and
 * Services Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the names
 * will have to be changed to match your code.
 *
 * There is for a substate machine. Make sure it has a unique name
 *
 * This is provided as an example and a good place to start.
 *
 * History
 * When           Who     What/Why
 * -------------- ---     --------
 * 09/13/13 15:17 ghe      added tattletail functionality and recursive calls
 * 01/15/12 11:12 jec      revisions for Gen2 framework
 * 11/07/11 11:26 jec      made the queue static
 * 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 * 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
 */


/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotHSM.h"
#include "SearchForHoleSubHSM.h"
#include "Motor_Control.h"
#include <stdio.h>
#include "Global_Macros.h"
#include "Timers.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    AlignSensor,
    Traverse,
    DrivePass,
    TurnIn,
    DriveTo,
    AlignLauncher,
    AlignDrive,
    PrecisionAlign,
    PrecisionBack,
    RevUpFlywheel,
    Launch,
    Reset,

} HoleSubHSMState_t;

static const char *StateNames[] = {
	"InitPSubState",
	"AlignSensor",
	"Traverse",
	"DrivePass",
	"TurnIn",
	"DriveTo",
	"AlignLauncher",
	"AlignDrive",
	"PrecisionAlign",
	"PrecisionBack",
	"RevUpFlywheel",
	"Launch",
	"Reset",
};



/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static HoleSubHSMState_t CurrentState = InitPSubState; // <- change name to match ENUM
static uint8_t MyPriority;
static uint8_t firstPass = TRUE; // if this is the first wall face seen or not
static uint8_t tapeSeen = FALSE; // whether or not tape has been seen since the last turn
static uint8_t tapeLost = FALSE;
static uint8_t towerSeen = FALSE;
static uint8_t alignDir = 0;
static uint32_t myTime = 0;
static uint32_t attempts = 0;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTemplateSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateFSM function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitSearchForHoleSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = InitPSubState;
    returnEvent = RunSearchForHoleSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunTemplateSubHSM(ES_Event ThisEvent)
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
ES_Event RunSearchForHoleSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    HoleSubHSMState_t nextState; // <- change type to correct enum
    uint32_t pingData;
    ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case InitPSubState: // If current state is initial Psedudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                // now put the machine into the actual initial state
                nextState = AlignSensor;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;

        case AlignSensor: // in the first state, replace this with correct names
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    myTime = TIMERS_GetTime();
                    firstPass = TRUE;
                    setServoPos(0);
                    printf("Aligning Sensor\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, ALIGN_TIME);
                    SetLeftMotor(ALIGN_SPEED);
                    SetRightMotor(-(ALIGN_SPEED));
                    break;


                case NEW_PING:
                    printf("New Ping %d\r\n", ThisEvent.EventParam);
                    if (ThisEvent.EventParam < PING_IN_RANGE - 2) {
                        nextState = Traverse;
                        makeTransition = TRUE;
                    }
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        nextState = AlignDrive;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_NO_EVENT:
                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case AlignDrive: // ede case where the ping sensor has not found the tower while aligning
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(OBSTACLE_TIMER, LOST_TIMEOUT); // if timer is expired, assume total failure
                    SetLeftMotor(0);
                    SetRightMotor(70); // drive forward and to the left
                    break;

                case NEW_PING: // on new ping data
                    if (ThisEvent.EventParam < PING_IN_RANGE - 1) { // if the new data is within range
                        nextState = Traverse; // begin traversing
                        makeTransition = TRUE;
                    }
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BUMPED: // if a bumped is pressed upon adjustment
                    nextState = AlignSensor; // go back to the align stage
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        SetLeftMotor(0);
                        SetRightMotor(0);
                        ES_Timer_StopTimer(OBSTACLE_TIMER); // not needed for now

                        ES_Event failEvent;
                        failEvent.EventType = TOWER_LOST;
                        PostRobotHSM(failEvent);
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
            }


        case Traverse:

            switch (ThisEvent.EventType) {

                case ES_ENTRY:
                    tapeSeen = FALSE;
                    tapeLost = FALSE;
                    towerSeen = FALSE;
                    printf("Traversing\r\n");
                    break;

                case NEW_PING:
                    pingData = ThisEvent.EventParam;
                    printf("New Ping %d\r\n", ThisEvent.EventParam);

                    if (pingData < PING_MAX && AD_ReadADPin(TW_PIN) > TW_HIGH_THRESH && AD_ReadADPin(S_TAPE_PIN) > 500 && !firstPass && !tapeLost &&
                            ((TIMERS_GetTime() - myTime) < 15000)) { // if while traversing the robot meets all of the alignment criteria
                        nextState = AlignLauncher;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }

                    /*
                    if (AD_ReadADPin(S_TAPE_PIN) < 500) {
                        towerSeen = TRUE;
                    }
                    if (AD_ReadADPin(S_TAPE_PIN) > 550 && towerSeen) {
                        tapeSeen = TRUE;
                    }
                    if (AD_ReadADPin(S_TAPE_PIN) < 500 && tapeSeen) {
                        tapeLost = TRUE;
                    }
                     * */

                    if ((pingData >= PING_MAX) && ((TIMERS_GetTime() - myTime) > 10000)) { // if driving past the tower
                        nextState = TurnIn; // begin to turn in
                        makeTransition = TRUE;
                    } else if (pingData > PING_IN_RANGE - 1) { // if a little too far from the tower
                        SetLeftMotor(TRAVERSE_SPEED);
                        SetRightMotor(TRAVERSE_SPEED + TRAVERSE_CORRECTION); // continue to drive forward but with a bias 
                    } else {
                        SetLeftMotor(TRAVERSE_SPEED);
                        SetRightMotor(TRAVERSE_SPEED);
                    }
                    ThisEvent.EventType = ES_NO_EVENT;

                    break;
            }
            break;

        case TurnIn:

            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("Turning\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, TURN_TIME); // amount of time to turn
                    SetLeftMotor(1);
                    SetRightMotor(TURN_SPEED); // turn speed
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        nextState = DriveTo;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        firstPass = FALSE;
                    }
                    break;

                case BUMPED: // edge case where it get misalinged on the turn
                    nextState = AlignSensor; // go back to the align stage
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;


            }
            break;

        case DriveTo: // drive for a period of time straight after the turn
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    myTime = TIMERS_GetTime();
                    printf("Driving to Reacquire\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, PASS_TIME);
                    SetLeftMotor(TRAVERSE_SPEED);
                    SetRightMotor(TRAVERSE_SPEED);
                    break;
                case ES_TIMEOUT: // if there is a timeout before ping reacquires
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        nextState = AlignSensor; // attempt to align again
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        ES_Timer_StopTimer(OBSTACLE_TIMER);
                    }
                    break;
                case NEW_PING:
                    if (ThisEvent.EventParam < PING_MAX) {
                        nextState = Traverse;
                        makeTransition = TRUE;
                        ES_Timer_StopTimer(OBSTACLE_TIMER);
                    }
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case DrivePass: // when the hole is found and time to drive pass for brief period of time
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(OBSTACLE_TIMER, 250); // timeout for how long to drive pass for
                    SetLeftMotor(15); // drive slowly pass
                    SetRightMotor(15);
                    break;
                case ES_TIMEOUT: // drive pass time has expired, time to align
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        nextState = AlignLauncher; // now align with the hole
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

            }
            break;
        case AlignLauncher:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    attempts = 0;
                    ES_Timer_InitTimer(OBSTACLE_TIMER, ALIGN_LAUNCH_TIME);
                    SetLeftMotor(-85);
                    SetRightMotor(60);
                    break;
                case ES_TIMEOUT: // timeout on alignement
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        SetMotors(0, 0);
                        nextState = PrecisionAlign;
                        makeTransition = TRUE;
                    }
                    break;
            }
            break;

        case PrecisionAlign: // fix any issues with the timer based alignment in this state
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    attempts++;
                    printf("Driving Forward\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, ALIGN_LAUNCH_FOR_TICKS * 1.5); // set the timer for when to begin backing up
                    SetLeftMotor(ALIGN_LAUNCH_SPEED); // set the speed upon entry
                    SetRightMotor(ALIGN_LAUNCH_SPEED);
                    break;

                case BUMPED: // turn off certain motors if the bumper is pressed
                    if ((IO_PortsReadPort(BUMPER_PORT) & (FL_BUMP_BIT | FR_BUMP_BIT)) == 0) {
                        if ((AD_ReadADPin(CR_TAPE_PIN) > C_TAPE_THRESH) || (AD_ReadADPin(CL_TAPE_PIN) > C_TAPE_THRESH)) {
                            nextState = RevUpFlywheel;
                            makeTransition = TRUE;
                        }

                    } else if ((IO_PortsReadPort(BUMPER_PORT) & FL_BUMP_BIT) == 0) {
                        SetLeftMotor(0);
                    } else if ((IO_PortsReadPort(BUMPER_PORT) & FR_BUMP_BIT) == 0) {
                        SetRightMotor(0);
                    }
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT: // timeout on alignement
                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        printf("CR: %d, CL: %d\r\n", AD_ReadADPin(CR_TAPE_PIN), AD_ReadADPin(CL_TAPE_PIN));
                        if ((AD_ReadADPin(CR_TAPE_PIN) < C_TAPE_THRESH) && (AD_ReadADPin(CL_TAPE_PIN) > C_TAPE_THRESH)) { // shifted left, back up slightly to the left
                            printf("Left Shifted\r\n");
                            SetLeftMotor(-ALIGN_LAUNCH_SPEED);
                            SetRightMotor(-ALIGN_LAUNCH_SPEED + ALIGN_SPEED_DIFF / 2);
                            attempts = 0;
                            nextState = PrecisionBack; // buffer state to allow for backing up for a period of time
                        } else if ((AD_ReadADPin(CR_TAPE_PIN) > C_TAPE_THRESH) && (AD_ReadADPin(CL_TAPE_PIN) < C_TAPE_THRESH)) { // shifted right, back up slightly to the right
                            printf("Right Shifted\r\n");
                            SetLeftMotor(-ALIGN_LAUNCH_SPEED + (int) (ALIGN_SPEED_DIFF));
                            SetRightMotor(-ALIGN_LAUNCH_SPEED);
                            nextState = PrecisionBack; // buffer state to allow for backing up for a period of time
                            attempts = 0;
                        } else if ((AD_ReadADPin(CR_TAPE_PIN) > C_TAPE_THRESH) && (AD_ReadADPin(CL_TAPE_PIN) > C_TAPE_THRESH)) {
                            printf("Centered\r\n");
                            SetLeftMotor(-ALIGN_LAUNCH_SPEED);
                            SetRightMotor(-ALIGN_LAUNCH_SPEED + (int) (ALIGN_SPEED_DIFF));
                            nextState = PrecisionBack;
                        } else if (attempts <= 20) {
                            SetLeftMotor(-ALIGN_LAUNCH_SPEED);
                            SetRightMotor(-ALIGN_LAUNCH_SPEED + (int) (ALIGN_SPEED_DIFF * 1.2));
                            nextState = PrecisionBack;
                        } else {
                            printf("Off Completely\r\n");
                            nextState = AlignSensor;
                        }

                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;

                    }
                    break;


            }
            break;

        case PrecisionBack: // go backwards if the tape criteria were not met
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("backing up\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, ALIGN_LAUNCH_BAC_TICKS); // motors were already set in the previous state, so just reset the timer on entry
                    break;

                case ES_TIMEOUT: // on timeout kick it back to the previous state

                    if (ThisEvent.EventParam == OBSTACLE_TIMER) {
                        printf("back up time expired\r\n");
                        nextState = PrecisionAlign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;


            }
            break;

        case RevUpFlywheel:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMotors(0, 0);
                    setServoPos(0);
                    setFlyMotor(FLY_POWER); // set flywheel to a reasonable speed
                    ES_Timer_InitTimer(LAUNCH_TIMER, REV_UP_TIME);
                    break;
                case ES_TIMEOUT:
                    if (LAUNCH_TIMER == ThisEvent.EventParam) {
                        ES_Timer_StopTimer(LAUNCH_TIMER);
                        nextState = Launch;
                        makeTransition = TRUE;
                    }
                    break;
            }
            break;
        case Launch:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(LAUNCH_TIMER, LAUNCH_TICKS);
                    setServoPos(1); // deliver a ball to the flywheel
                    break;

                case ES_TIMEOUT:
                    if (LAUNCH_TIMER == ThisEvent.EventParam) {
                        ES_Timer_StopTimer(LAUNCH_TIMER);
                        setServoPos(0);
                        setFlyMotor(0);
                        ThisEvent.EventType = ES_NO_EVENT;

                        // create a new event to post to the top level to indicate that the launch sequence has been completed and is reseting
                        ES_Event launchEvent;
                        launchEvent.EventType = LAUNCH_COMPLETE;
                        PostRobotHSM(launchEvent);
                    }

            }
            break;

    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunSearchForHoleSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunSearchForHoleSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

