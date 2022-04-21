/*
 * File:   Ping_Sensor_Test.c
 * Author: jemcclel
 * Blocking code to test ping sensor hardware and IO ports
 */


#include <stdio.h>
#include "timers.h"
#include "xc.h"
#include "IO_Ports.h"
#include "pwm.h"
#include "Motor_Control.h"
#include "Global_Macros.h"
#include "RC_Servo.h"

#define ENABLE_MOTORS

static int rightPow = 0;
static int leftPow = 0;
static int flyPow = 0;

// initializes the motors' pins

void InitMotors(void) {

    printf("Motors Initialized\r\n");

    // PWM INIT
    PWM_Init(); // activate pwm for the motors
    PWM_SetFrequency(PWM_30KHZ); // set the PWM Frequency
    PWM_AddPins(LEFT_EN | RIGHT_EN | SERVO_PIN | FLY_PIN); // add the left and right enable pins as a PWM

    // set all the default duty cycles to 0
    PWM_SetDutyCycle(RIGHT_EN, 0);
    PWM_SetDutyCycle(LEFT_EN, 0);
    PWM_SetDutyCycle(SERVO_PIN, 0);
    PWM_SetDutyCycle(FLY_PIN, 0);
    
    // servo control init
    RC_Init();
    RC_AddPins(RC_PORTX04);
    setServoPos(0);

    // AD SETUP
    IO_PortsSetPortOutputs(MOTOR_PORT, RIGHT_IN1_PIN | RIGHT_IN2_PIN | LEFT_IN1_PIN | LEFT_IN2_PIN); // set the ports for controlling the left and right motors to output
    IO_PortsClearPortBits(MOTOR_PORT, RIGHT_IN1_PIN | RIGHT_IN2_PIN | LEFT_IN1_PIN | LEFT_IN2_PIN); // set each port low as default
}

/* FOR THE MAIN TWO MOTORS */

/*
 * set the duty cycle and direction of the Right Motor
 * input is power, a value from -100 to 100. Negative indicates direction, 
 * magnitude is the duty cycle percentage
 */
void SetRightMotor(int pow) {

    // if out of range pow, adjust it 
    if (pow > 100) {
        pow = 100;
    } else if (pow < -100) {
        pow = -100;
    }

    int direction = 0; // used to set the direction, default is reverse
    int stop = 0; // used for active braking, default is not stopping
    if (pow > 0) {
        direction = 1;
    } else if (pow == 0) { // if power is set to 0, active stop
        stop = 1; // set stop to high
    }
    // NOTE: there is no case for setting to reverse, since thats the implied default
#ifdef ENABLE_MOTORS
    PWM_SetDutyCycle(RIGHT_EN, (abs(pow)*40 / 100 + 60)*10 * !stop); // set the new duty cycle
#endif

    if (direction == 1) { // 1 is forward, 0 is reverse
        IO_PortsSetPortBits(MOTOR_PORT, RIGHT_IN1_PIN*!stop); // set pin given by direction
        IO_PortsClearPortBits(MOTOR_PORT, RIGHT_IN2_PIN);
    } else {
        IO_PortsSetPortBits(MOTOR_PORT, RIGHT_IN2_PIN*!stop);
        IO_PortsClearPortBits(MOTOR_PORT, RIGHT_IN1_PIN);
    }

    rightPow = pow; // set the global variable for returns
}

/*
 * set the duty cycle and direction of the Left Motor
 * input is power, a value from -100 to 100. Negative indicates direction, 
 * magnitude is the duty cycle percentage
 */
void SetLeftMotor(int pow) {

    // if out of range pow, adjust it 
    if (pow > 100) {
        pow = 100;
    } else if (pow < -100) {
        pow = -100;
    }

    int direction = 0; // used to set the direction, default is reverse
    int stop = 0; // used for active braking, default is not stopping
    if (pow > 0) {
        direction = 1;
    } else if (pow == 0) { // if power is set to 0, active stop
        stop = 1; // set stop to high
    }
    // NOTE: there is no case for setting to reverse, since thats the implied default
#ifdef ENABLE_MOTORS
    PWM_SetDutyCycle(LEFT_EN, (abs(pow)*40 / 100 + 60)*10 * !stop); // set the new duty cycle
#endif
    if (direction == 1) { // 1 is forward, 0 is reverse
        IO_PortsSetPortBits(MOTOR_PORT, LEFT_IN1_PIN*!stop); // set pin given by direction
        IO_PortsClearPortBits(MOTOR_PORT, LEFT_IN2_PIN);
    } else {
        IO_PortsSetPortBits(MOTOR_PORT, LEFT_IN2_PIN*!stop);
        IO_PortsClearPortBits(MOTOR_PORT, LEFT_IN1_PIN);
    }

    leftPow = pow; // set the global variable for returns
}

// set the power on both motors, same convention as singles

void SetMotors(int powL, int powR) {
    SetLeftMotor(powL);
    SetRightMotor(powR);
}

// get the left motor's power

int getLeftPow(void) {
    return leftPow;
}

// get the right motor's power

int getRightPow(void) {
    return rightPow;
}

/* FOR THE LAUNCHER'S FLYWHEEL AND SERVO */

// set the position of the servo. Takes in a binary number, 0 for initial position, >0 for ball load position

void setServoPos(uint8_t pos) {
    if (pos == 0) {
        RC_SetPulseTime(RC_PORTX04, 2500); // idle position
    } else {
        RC_SetPulseTime(RC_PORTX04, 1000); // loading position
    }

}

// set the power on the fly wheel. Input is pow, range 0-100%
// for the love of god, do not actually do 100%

void setFlyMotor(uint32_t pow) {
    if (pow > 100) {
        printf("Error: Fly Wheel Pow too large\r\n");
        return;
    }
    PWM_SetDutyCycle(FLY_PIN, pow * 10);
    flyPow = pow;
}

// returns current flywheel power

uint32_t getFlyDuty(void) {
    return flyPow;
}