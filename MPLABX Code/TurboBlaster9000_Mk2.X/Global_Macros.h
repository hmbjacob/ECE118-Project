// THIS FILE CONTAINS ALL OF THE GLOBAL MACROS USED THROUGHOUT THIS PROJECT. ALL STORED IN ONE COVENIENT LOCATION!
#include <stdio.h>
#include "timers.h"
#include "xc.h"
#include "IO_Ports.h"
#include "pwm.h"
#include "AD.h"

#ifndef _MACROS_H
#define _MACROS_H

/***************************************************
 * macros for directly controlling the state machine 
 ***************************************************/

// Aquire Tower
#define TURN_360_TICKS 5000
#define ACQUIRE_SPEED 75
#define BEACON_CLOSE_THRESH 0
#define BEACON_HIGH_THRESH 220
#define BEACON_LOW_THRESH 200

// Approach Tower
#define APR_SPEED 100
#define APR_DIFF 40
#define APR_TIMEOUT 1200

// Align Sensor
#define ALIGN_SPEED 30
#define PING_IN_RANGE 4
#define PING_MAX 15
#define ALIGN_TIME 2000
#define LOST_TIMEOUT 4000

// Traverse Tower
#define TRAVERSE_SPEED 45
#define TRAVERSE_CORRECTION 25
#define TW_HIGH_THRESH 220

// Drive pass
#define PASS_TIME 5000

// Turn Around Tower
#define TURN_TIME 2100
#define TURN_SPEED 100

// Align Launcher
#define ALIGN_LAUNCH_TIME 635
#define CL_TAPE_THRESH 37
#define CR_TAPE_THRESH 150
#define C_TAPE_THRESH 350
#define ALIGN_LAUNCH_FOR_TICKS 600
#define ALIGN_LAUNCH_BAC_TICKS 500
#define ALIGN_LAUNCH_SPEED 50
#define ALIGN_SPEED_DIFF 35

// Launch Ball
#define FLY_POWER 97
#define REV_UP_TIME 7000
#define LAUNCH_TICKS 1500

// Reset after launch
#define RESET_TIME 3000
#define RESET_SPEED 37
#define RESET_DIFF 75

// Resolve Obstacle
#define RESOLVE_TIME 800
#define RESOLVE_SPEED 75
#define REVERSE_DIFF 35
#define FORWARD_DIFF 0
#define LIGHT_THRESHOLD 700
#define DARK_THRESHOLD 750

// New Tower
#define EXIT_SPEED 70
#define EXIT_TICKS 300
#define ALIGN_TICKS 1300
#define REVERSE_TURN_SPEED 95
#define ADJUST_SPEED 20
#define ADJUST_TICKS 200
#define TURN_TIMEOUT 4000

//encircle
#define INIT_BACK_TICKS 1000
#define BACK_TICKS 2300
#define PIVOT_TICKS 1900

/*******************************************************
 * macros for defining pins in motors, sensors and misc
 *******************************************************/

// defines for the motor pins
#define LEFT_EN PWM_PORTY12
#define RIGHT_EN PWM_PORTY10
#define MOTOR_PORT PORTY
#define RIGHT_IN1_PIN PIN5
#define RIGHT_IN2_PIN PIN7
#define LEFT_IN1_PIN PIN6
#define LEFT_IN2_PIN PIN8

// servo control pin
#define SERVO_PIN PWM_PORTZ06

// fly wheel control pin
#define FLY_PIN PWM_PORTX11

// Ping sensors timing
#define PING_HIGH_TICKS 1 // how long to leave the trigger high for in ms
#define PING_WAIT_TICKS 50 // how long to wait between echo finish and new trigger in ms

// ports for the ping sensor
#define PING_PORT PORTV
#define TRIG_PIN PIN3 // trigger output
#define ECHO_PIN PIN4 // echo input

// Hysteresis thresholds
#define BATTERY_DISCONNECT_THRESHOLD 175 // battery

// light states
#define LIGHT 1
#define DARK 0

// beacon states
#define LOW 0
#define HIGH 1

// Track wire
#define TW_PIN AD_PORTV5   // pin connection
#define TW_LOW_THRESH 50  // hysteresis

// ordering the tape sensors, used in an array for data collection
#define FL_TAPE 0
#define FR_TAPE 1
#define BL_TAPE 2
#define BR_TAPE 3
#define CL_TAPE 4
#define CR_TAPE 5
#define S_TAPE 6

// tape sensor pins, all analog inputs
#define FL_TAPE_PIN AD_PORTW3 // BLUE WIRE
#define FR_TAPE_PIN AD_PORTW4 // GREEN WIRE
#define BL_TAPE_PIN AD_PORTW5 // WHITE WIRE
#define BR_TAPE_PIN AD_PORTW6 // BROWN WIRE
#define CL_TAPE_PIN AD_PORTW7 // ORANGE WIRE
#define CR_TAPE_PIN AD_PORTW8 // RED WIRE
#define S_TAPE_PIN  AD_PORTV7 // PURPLE WIRE

// binary values of the tape sensors used for storing the values in a passable variable
#define FL_TAPE_BIT 0b00000001
#define FR_TAPE_BIT 0b00000010
#define BL_TAPE_BIT 0b00000100
#define BR_TAPE_BIT 0b00001000
#define CL_TAPE_BIT 0b00010000
#define CR_TAPE_BIT 0b00100000
#define S_TAPE_BIT  0b01000000

// defining the bumper pins, all digital inputs
#define BUMPER_PORT PORTZ 
#define FL_BUMP_PIN PIN11 // blue
#define FR_BUMP_PIN PIN9  // stripped
#define BL_BUMP_PIN PIN7  // purple
#define BR_BUMP_PIN PIN5  // green

// bumper binaries for passing in a variable
#define FL_BUMP_BIT 0b100000000000
#define FR_BUMP_BIT 0b1000000000
#define BL_BUMP_BIT 0b10000000
#define BR_BUMP_BIT 0b100000

// beacon analog and digital input pins
#define BEACON_PORT PORTX
#define BEACON_A_PIN AD_PORTV6 // analog
#define BEACON_D_PIN PIN5      // digital

#endif
