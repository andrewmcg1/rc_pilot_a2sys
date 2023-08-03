#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>

#include <delta_arm.h>
#include <xbee_receive.h>

#include <rc/uart.h>


int delta_pid;
char command[64];

int delta_init(int delta_bus, char* xbee_port, int enabled)
{
    if(enabled)
    {
        XBEE_init(xbee_port, 0);
        system("mkfifo xbee_temp");
        snprintf(command, 64, "cat %s | tee xbee_temp /dev/ttyO%d > /dev/null &", xbee_port, delta_bus);
        delta_pid = system(command);
        return delta_pid;
    }
    else
    {
        return -1;
    }
}

void delta_cleanup()
{
    system("rm xbee_temp");
    kill(delta_pid, SIGKILL);
}