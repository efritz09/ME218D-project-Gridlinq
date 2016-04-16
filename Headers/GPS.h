/****************************************************************************
 
  Header file for GPS.c

 ****************************************************************************/
#ifndef GPS_H
#define GPS_H

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
    gStasis, 
    gWaitForStartState, 
    gWaitForTypeCode, 
    ProcessGPSState, 
    gCheckSum1, 
    gCheckSum2,
} GPSState;

typedef enum {
    GGA,
    GSA,
    GSV,
    RMC,
    VTG
} GPSID;

typedef struct GPS_Data {
    uint16_t UTC_Time[4];       //[hh][mm][ss].[sss]
    uint16_t Latitude[3];       //[dd][mm].[mmmm]
    char NSIndicator;           //N or S
    uint16_t Longitude[3];      //[ddd][mm].[mmmm]
    char EWIndicator;           //E or W
    uint16_t PosIndicator;      //0,1,2
    uint16_t numOfSatellites;	//range 0 - 14
    uint16_t HDOP;              //horizontal dilution of precision x 100
    uint16_t MSLAltitude;       //antenna altitude above mean sea level x 10
    char AltUnits;              //units of altitude M
    uint16_t GeoidalSep;        //geoidal separation x 10
    char GeoUnits;              //units of geoids separation
} GPS_Data;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitGPS( uint8_t Priority );
bool PostGPS ( ES_Event ThisEvent );
ES_Event RunGPS ( ES_Event CurrentEvent );
GPS_Data GetGPSData(void);

#endif /* GPS_H */
