/****************************************************************************
 
  Header file for SpectreXbee.c

 ****************************************************************************/
 
#ifndef CLAMP_H
#define CLAMP_H

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
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "bitdefs.h"
#include "driverlib/gpio.h" 

/*----------------------------- Module Defines ----------------------------*/
typedef enum {Open, Debouncing, Closed, Disengaging} ClampState;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitClamp ( uint8_t Priority );
bool PostClamp ( ES_Event ThisEvent );
ES_Event RunClamp( ES_Event CurrentEvent );


#endif /* CLAMP_H */
