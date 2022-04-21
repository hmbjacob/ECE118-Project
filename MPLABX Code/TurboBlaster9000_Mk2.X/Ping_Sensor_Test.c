/*
 * File:   Ping_Sensor_Test.c
 * Author: jemcclel
 * Blocking code to test ping sensor hardware and IO ports
 * this is testing code and not a part of the final code compilation
 */

#include "BOARD.h"
#include <stdio.h>
#include "timers.h"
#include "xc.h"
#include "IO_Ports.h"

int main(void) {
    
    // Initialzation Sequence
    BOARD_Init();
    TIMERS_Init();
    
    // Setup pins
    IO_PortsSetPortOutputs(PORTW, PIN3); // Ping Pin
    IO_PortsWritePort(PORTW, 0);
    
    IO_PortsSetPortInputs(PORTW, PIN4); // Echo Pin
    
    printf("Initialized\r\n");
    
    int myTime = TIMERS_GetTime();
    while (1) {
        // Trigger the Sensor
        IO_PortsWritePort(PORTW, PIN3); 
        myTime = TIMERS_GetTime();
        while (TIMERS_GetTime() < myTime + 2);
        IO_PortsWritePort(PORTW, 0);
        
        // wait for echo rising edge
        while ((int)(IO_PortsReadPort(PORTW) & PIN4) == 0);
        int startTime = TIMERS_GetTime(); // get time echo goes high

        // wait for echo falling edge
        while ((int)(IO_PortsReadPort(PORTW) & PIN4) > 0);
        int deltaT = TIMERS_GetTime() - startTime; // calculate time echo was high
        printf("%d\r\n", deltaT);


        // wait for time to expire before another ping
        while (TIMERS_GetTime() < myTime + 500);
        myTime = TIMERS_GetTime();
        
    }
    
    return 0;
}
