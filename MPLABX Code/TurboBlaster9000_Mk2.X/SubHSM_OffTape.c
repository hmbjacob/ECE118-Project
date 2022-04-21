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
#include "TurboHSM.h"
#include "SubHSM_OffTape.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    SearchTower,
    SearchHole,
} OffTapeSubHSMState_t;

static const char *StateNames[] = {
	"InitPSubState",
	"SearchTower",
	"SearchHole",
};

// BUMPRE DEFINES
#define FL_BUMPER 0
#define FR_BUMPER 1
#define BL_BUMPER 2
#define BR_BUMPER 3



/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

// Reads the bumpers, returns output corresponding to which bumper was pressed
// Four bit output, first bit FL, 2nd FR, 3rd BL, 4th BR
uint8_t ReadBumpers(void);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static OffTapeSubHSMState_t CurrentState = InitPSubState; // set the first state to temp psuedo state
static uint8_t MyPriority;


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/*
 * Initializes the Off Tape Sub HSM. The first state that will be entered as a 
 * result is the 360 rotate before looking for the beacon
 */
uint8_t InitOffTapeSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = InitPSubState; // go back to first pseudo state whenever initialized
    RunOffTapeSubHSM(INIT_EVENT); // pass through the initialize event before continuing 

    if (returnEvent.EventType == ES_NO_EVENT) { // if there are no issues with initialization                                 
        return TRUE; // return true, which implies no issues have occurred while initializing
    }
    return FALSE; // an issue has occurred, return false
}

/*
 * Progresses the SubHSM state machine with events passed on by parent HSM
 */
ES_Event RunOffTapeSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    OffTapeSubHSMState_t nextState; // declare state variable used for transitioning

    ES_Tattle(); // trace call stack

    // Switching here. Two possible states that are not pseudo: Search for tower,
    // search for hole. One state navigates towards the VAT tower, the other around
    // the VAT tower with the end goal of aligning the bot with the correct hole
    switch (CurrentState) {
        case InitPSubState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                // Initialize the relevant subsubHSMs
                InitSearchTowerSubHSM();
                InitSearchHoleSubHSM();

                // Make transition to the first state
                if (ReadBumpers() != 0) { // corner case where already at a tower after exiting, occurs when a bumper is pressed
                    nextState = SearchForHole; // begin searching for hole immediately
                } else {
                    nextState = SubFirstState; // begin tower search
                }
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;

        case SearchTower: // when searching for the tower, in this state
            ThisEvent = RunSearchHoleSubHSM(ThisEvent); // pass event down to corresponding subHSM first, update event
            switch (ThisEvent.EventType) { // process the current event


                    // on entry to this state, initialize its subHSM
                case ES_ENTRY:
                    InitSearchTowerSubHSM();
                    break;

                    // if either of the front bumpers are pressed its time to conclude
                    // that robot is at VAT tower
                    // To note: bumping into the other robot is an actual possibility that 
                    // will probably need to be dealt with in the future
                case FR_PRESS:
                case FL_PRESS:

                    // transition to the hole searching state and consume the press event
                    nextState = SearchForHole;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;



                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case SearchHole: // when tower is found, in this state
            ThisEvent = RunSearchHoleSubHSM(ThisEvent); // pass event down to corresponding subHSM first, update event
            
            switch (ThisEvent.EventType) { // process the current event
                // On entry, reinitialize the subHSM
                case ES_ENTRY:
                    InitSearchHoleSubHSM();
                    
                case LAUNCH_COMPLETE: // ball has been deposited, time to look for a new tower
                case LOST_TOWER: // if the tower was lost, go back to the start of search for tower
                    
                    //update, transition, consume as usual
                    nextState = SearchTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunOffTapeSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunOffTapeSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

// Reads the bumpers, returns output corresponding to which bumper was pressed
// Four bit output, first bit FL, 2nd FR, 3rd BL, 4th BR
uint8_t ReadBumpers(void);
uint8_t ReadBumpers(void) {
    uint8_t bumpers = 0b0000;
    return FL_BUMPER;
}