#include <BOARD.h>
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "Timers.h"
#include "IO_Ports.h"
#include "Global_Macros.h"
#include "Motor_Control.h"
#include "RC_Servo.h"

//#define MOTORTEST
//#define BUMPERTEST
#define TAPETEST
//#define PINGTEST
//#define BEACON_TEST
//#define TW_TEST
//#define FLY_TEST
void initHardware(void);
void testHardware(void);
void busyDelay(int time);

void main(void) {
    ES_Return_t ErrorType;

    BOARD_Init();

    printf("Starting ES Framework Sensor Testing\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");


    // Your hardware initialization function calls go here

    initHardware();
    //busyDelay(50);
    //testHardware();

    // now initialize the Events and Services Framework and start it running
    ErrorType = ES_Initialize();
    if (ErrorType == Success) {
        ErrorType = ES_Run();

    }
    //if we got to here, there was an error
    switch (ErrorType) {
        case FailedPointer:
            printf("Failed on NULL pointer");
            break;
        case FailedInit:
            printf("Failed Initialization");
            break;
        default:
            printf("Other Failure: %d", ErrorType);
            break;
    }
    for (;;)
        ;

};

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

// initializes all of the pins / other hardware needed for the robot

void initHardware(void) {
    // general inits relative to the project
    ES_Timer_Init();
    TIMERS_Init();

    // motor inits: includes servos, h bridge control pins, fly wheel control
    InitMotors();

    // init the tape sensors
    AD_Init();
    AD_AddPins(FL_TAPE_PIN | FR_TAPE_PIN | BL_TAPE_PIN | BR_TAPE_PIN | CL_TAPE_PIN | CR_TAPE_PIN | S_TAPE_PIN);

    // init the beacon pins
    AD_AddPins(BEACON_A_PIN);
    IO_PortsSetPortInputs(BEACON_PORT, BEACON_D_PIN);

    // init trackwire pin
    AD_AddPins(TW_PIN);

    // init bumpers
    IO_PortsSetPortInputs(BUMPER_PORT, FL_BUMP_PIN | FR_BUMP_PIN | BL_BUMP_PIN | BR_BUMP_PIN);

    // init ping sensor
    IO_PortsSetPortOutputs(PING_PORT, TRIG_PIN); // set up the trigger pin to output
    IO_PortsClearPortBits(PING_PORT, TRIG_PIN);
    IO_PortsSetPortInputs(PING_PORT, ECHO_PIN); // set up the echo pin to input
}
// tests much of the relevant hardware to make sure its working / plugged in properly

void testHardware(void) {
    printf("BEGGINING HARDWARE TESTING!\r\n\n");

#ifdef MOTORTEST
    printf("MOTOR TESTS:\r\n");
    printf("LEFT FORWARD:\r\n");
    SetLeftMotor(100);
    busyDelay(2000); // two seconds
    SetLeftMotor(50);
    busyDelay(2000);
    printf("LEFT REVERSE:\r\n");
    SetLeftMotor(-100);
    busyDelay(2000);
    SetLeftMotor(-50);
    busyDelay(2000);
    SetLeftMotor(0);
    printf("RIGHT FORWARD:\r\n");
    SetRightMotor(100);
    busyDelay(2000);
    SetRightMotor(50);
    busyDelay(2000);
    printf("RIGHT REVERSE:\r\n");
    SetRightMotor(-100);
    busyDelay(2000);
    SetRightMotor(-50);
    busyDelay(2000);
    SetRightMotor(0);
    printf("MOTOR TEST COMPLETE:\r\n\n");
#endif

#ifdef BUMPERTEST
    printf("BUMPER TESTS:\r\n");
    printf("Push the FL Bumper\r\n");
    while ((IO_PortsReadPort(BUMPER_PORT) & FL_BUMP_BIT) > 0);
    //while ((IO_PortsReadPort(PORTV) & 0b100000000000) > 0);
    printf("FL Bump Detect\r\n");
    printf("Push the FR Bumper\r\n");
    while ((IO_PortsReadPort(BUMPER_PORT) & FR_BUMP_BIT) > 0);
    printf("FR Bump Detect\r\n");
    printf("Push the BL Bumper\r\n");
    while ((IO_PortsReadPort(BUMPER_PORT) & BL_BUMP_BIT) > 0);
    printf("BL Bump Detect\r\n");
    printf("Push the BR Bumper\r\n");
    while ((IO_PortsReadPort(BUMPER_PORT) & BR_BUMP_BIT) > 0);
    printf("BR Bump Detect\r\n");
#endif

#ifdef TAPETEST
    // FL TAPE TEST
    while (1) {
        printf("\r\n");
        busyDelay(2000);
        int pinVal = (int) AD_ReadADPin(FL_TAPE_PIN);
        if (pinVal > 10) {
            printf("FL Tape Value: %d\r\n", pinVal);
        } else {
            printf("Very Low Value From FL Tape, possible bad connection: %d\r\n", pinVal);
        }

        // FR TAPE TEST
        pinVal = (int) AD_ReadADPin(FR_TAPE_PIN);
        if (AD_ReadADPin(FR_TAPE_PIN) > 10) {
            printf("FR Tape Value: %d\r\n", (int) AD_ReadADPin(FR_TAPE_PIN));
        } else {
            printf("Very Low Value From FR Tape, possible bad connection: %d\r\n", pinVal);
        }

        // BL TAPE TEST
        pinVal = (int) AD_ReadADPin(BL_TAPE_PIN);
        if (AD_ReadADPin(BL_TAPE_PIN) > 10) {
            printf("BL Tape Value: %d\r\n", (int) AD_ReadADPin(BL_TAPE_PIN));
        } else {
            printf("Very Low Value From BL Tape, possible bad connection: %d\r\n", pinVal);
        }

        // BR TAPE TEST
        pinVal = (int) AD_ReadADPin(BR_TAPE_PIN);
        if (AD_ReadADPin(BR_TAPE_PIN) > 10) {
            printf("BR Tape Value: %d\r\n", (int) AD_ReadADPin(BR_TAPE_PIN));
        } else {
            printf("Very Low Value From BR Tape, possible bad connection: %d\r\n", pinVal);
        }

        // CR TAPE TEST
        pinVal = (int) AD_ReadADPin(CR_TAPE_PIN);
        if (AD_ReadADPin(CR_TAPE_PIN) > 10) {
            printf("CR Tape Value: %d\r\n", (int) AD_ReadADPin(CR_TAPE_PIN));
        } else {
            printf("Very Low Value From CR Tape, possible bad connection: %d\r\n", pinVal);
        }

        // CL TAPE TEST
        pinVal = (int) AD_ReadADPin(CL_TAPE_PIN);
        if (AD_ReadADPin(CL_TAPE_PIN) > 10) {
            printf("CL Tape Value: %d\r\n", (int) AD_ReadADPin(CL_TAPE_PIN));
        } else {
            printf("Very Low Value From CL Tape, possible bad connection: %d\r\n", pinVal);
        }

        // S TAPE TEST
        pinVal = (int) AD_ReadADPin(S_TAPE_PIN);
        if (AD_ReadADPin(S_TAPE_PIN) > 10) {
            printf("S Tape Value: %d\r\n", (int) AD_ReadADPin(S_TAPE_PIN));
        } else {
            printf("Very Low Value From S Tape, possible bad connection: %d\r\n", pinVal);
        }
    }

#endif
#ifdef BEACON_TEST
    while (1) {
        busyDelay(1000);
        printf("Beacon Value: %d\r\n", AD_ReadADPin(BEACON_A_PIN));

    }
#endif

#ifdef PINGTEST
    printf("PING SENSOR TEST:\r\n");
    // Trigger the Sensor
    while (1) {
        IO_PortsSetPortBits(PING_PORT, TRIG_PIN);
        busyDelay(1);
        IO_PortsClearPortBits(PING_PORT, TRIG_PIN);

        // wait for echo rising edge


        while ((int) (IO_PortsReadPort(PING_PORT) & ECHO_PIN) == 0);
        int startTime = TIMERS_GetTime(); // get time echo goes high

        // wait for echo falling edge
        while ((int) (IO_PortsReadPort(PING_PORT) & ECHO_PIN) > 0) {
            if (TIMERS_GetTime() > 40 - startTime) break;
        };
        int deltaT = TIMERS_GetTime() - startTime; // calculate time echo was high
        printf("Ping time: %d\r\n", deltaT);
    }
#endif

#ifdef TW_TEST
    printf("TRACK WIRE TEST:\r\n");
    SetLeftMotor(100);
    while (1) {
        busyDelay(1000);
        printf("TW: %d\r\n", AD_ReadADPin(TW_PIN));
    }
#endif


#ifdef FLY_TEST
    RC_AddPins(RC_PORTX04);
    setServoPos(0);
    busyDelay(2000);
    
    setFlyMotor(FLY_POWER);
    busyDelay(7000);
    setServoPos(1);
    busyDelay(2000);
    setFlyMotor(00);
    setServoPos(0);
    busyDelay(2000);
#endif

    // if doing a hardware test, dont begin ES Framework
    while (1) {

    };


}

// busy delay function used in testing

void busyDelay(int time) {
    time = time * 10;
    TIMERS_ClearTimerExpired(1);
    TIMERS_SetTimer(1, time);
    TIMERS_StartTimer(1);
    while (!TIMERS_IsTimerExpired(1));
    TIMERS_ClearTimerExpired(1);
}
