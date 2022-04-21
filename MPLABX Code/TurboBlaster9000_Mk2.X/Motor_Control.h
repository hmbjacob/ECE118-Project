

#ifndef MOTOR_H
#define	MOTOR_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT
#include "BOARD.h"

// DA FUNCTIONS

// initializes the motors' pins

void InitMotors(void);

/* FOR THE MAIN TWO MOTORS */

/*
 * set the duty cycle and direction of the Right Motor
 * input is power, a value from -100 to 100. Negative indicates direction, 
 * magnitude is the duty cycle percentage
 */
void SetRightMotor(int pow);

/*
 * set the duty cycle and direction of the Left Motor
 * input is power, a value from -100 to 100. Negative indicates direction, 
 * magnitude is the duty cycle percentage
 */
void SetLeftMotor(int pow);

// set the power on both motors, same convention as singles
void SetMotors(int powL, int powR);

// get the left motor's power
int getLeftPow(void);

// get the right motor's power
int getRightPow(void);

/* FOR THE LAUNCHER'S FLYWHEEL AND SERVO */

// set the position of the servo. Takes in a binary number, 0 for initial position, >0 for ball load position
void setServoPos(uint8_t pos);

// set the power on the fly wheel. Input is pow, range 0-100%
// for the love of god, do not actually do 100%
void setFlyMotor(uint32_t pow);

// returns current flywheel power
uint32_t getFlyDuty(void);




#endif	/* MOTOR_H */

