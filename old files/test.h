/****************************************************************************
 
  Header file for SpectreXbee.c

 ****************************************************************************/
 
#ifndef GONDOLA_H
#define GONDOLA_H

/*----------------------------- Include Files -----------------------------*/
// set up a global header for the project
// Standard headers
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// ES_framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"
#include "ES_Events.h"

// TIVA headers
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "bitdefs.h"

/*----------------------------- Module Defines ----------------------------*/
typedef enum {Detached, Run, Sleep} GondolaState;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitGondola ( unit8_t Priority );
bool PostGondola ( ES_Event ThisEvent );

ES_Event RunGondola ( ES_Event CurrentEvent );


#endif /* GONDOLA_H */
