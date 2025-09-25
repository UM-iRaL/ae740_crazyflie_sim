/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (c) 2018  Eric Goubault, Sylvie Putot, Franck Djeumou
 *             Cosynux , LIX , France
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main_sitl.c - Containing the main function.for the SITL simulation based on main.c
 */



/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* Project includes */
#include "socketlink.h"
#include "config.h"
#include "platform.h"
#include "system.h"
#include "usec_time.h"


#define CRTP_PORT 19950
char CRTP_SERVER_ADDRESS[] = "INADDR_ANY";

/* Store the UDP port to communicate with CF */
uint16_t crtp_port;
/* Store an IP address or INADDR_ANY for the server (CF) */
char* address_host;
// /* Store the cf instance unique Identifiant */
// uint8_t cf_id;

int main(int argc, char **argv) 
{

  if (argc == 3){
    crtp_port = atoi(argv[1]);
    address_host = argv[2];
    printf("address : %s , port : %d \n" , address_host , crtp_port );
  } else if (argc == 2) {
    // Initiaze socket parameters
    crtp_port =  atoi(argv[1]);
    address_host = CRTP_SERVER_ADDRESS;
    printf("address : %s , port : %d \n" , address_host , crtp_port );
  }else {
    crtp_port =  CRTP_PORT;
    address_host = CRTP_SERVER_ADDRESS;
    printf("No port and ADDRESS selected ! 19950-INADDR_ANY selected as default\n");
  }

  //Initialize the platform.
  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while(1);
  }

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //Should never reach this point!
  while(1);

  return 0;
}

/*
* FreeRTOS assert name
*/
void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
    printf("ASSERT: %s : %d\n", pcFileName, (int)ulLine);
    while(1);
}

unsigned long ulGetRunTimeCounterValue(void)
{
    return 0;
}

void vConfigureTimerForRunTimeStats(void)
{
    return;
}

/* For memory management */
void vApplicationMallocFailedHook(void)
{
    while(1);
}