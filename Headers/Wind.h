/****************************************************************************
 
  Header file for Wind.c

 ****************************************************************************/
 
#ifndef WIND_H
#define WIND_H

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
typedef enum {
    Stasis, 
    WaitForStartState, 
    WaitForTypeCode, 
    ProcessWindData, 
    ProcessTempData, 
    CheckSum1, 
    CheckSum2,
} WindState;

typedef struct Wind_Data {
    uint16_t    WindSpeed;      //Wind speed
    uint16_t	WindAngle;      //Wind angle (degrees)
    uint16_t    Temp;           //Air Temperature
    char	    WindUnit;	    //Wind units
    char	    TempUnit;	    //Temperature units
} Wind_Data;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitWind( uint8_t Priority );
bool PostWind ( ES_Event ThisEvent );
ES_Event RunWind ( ES_Event CurrentEvent );

Wind_Data GetWindData(void);

#endif /* WIND_H */
