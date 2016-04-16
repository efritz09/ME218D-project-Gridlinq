/****************************************************************************
 Module
  Gridlinq.c

 Description
  State Machine and core code for running the project
	
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
#include "Gridlinq.h"
#include "UART.h"
#include "Clamp.h"
#include "Wind.h"
#include "GPS.h"
#include "LidarCom.h"
#include "CompassCom.h"

// TivaWare Headers
#include "driverlib/sysctl.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define ONE_SEC 		    976

#define WIND_ID_CHECK       0x4c
#define TEMP_ID_CHECK       0x50

#define TEMP_UNIT           14
#define TEMP_1			    8
#define TEMP_2              9
#define TEMP_3              10
#define TEMP_4              12

#define WIND_UNIT           20
#define	WIND_ANGLE_1        6
#define	WIND_ANGLE_2        7
#define	WIND_ANGLE_3        8
#define	WIND_ANGLE_4        10

#define WIND_SPEED_1        14
#define WIND_SPEED_2        15
#define WIND_SPEED_3        16
#define WIND_SPEED_4        18


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
static void importData(Sensor s);
static void printGPSData(void);
static void printWindData(void);

/* General communication functions
static void importData(void);
uint8_t * GetCommands(void);
uint8_t checkSum(uint8_t chkArray[], uint8_t size);
static void LoadUpConfirmation(void);
static void LoadUpStatus(uint8_t statusVal);
static void SendXbeeData( void );
*/


// Initialization functions


/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static GridlinqState CurrentState;
static Wind_Data CV7;
static GPS_Data GPSdata;
static LidarData_t lidarData;
static CompassReading_t compassData;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitGridlinq

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the UART and Pins necessary for communication
****************************************************************************/
bool InitGridlinq ( uint8_t Priority ) {
    MyPriority = Priority;
    CurrentState = Detached;
	
    InitUART();
    // post the initial transition event
    ES_Event ThisEvent = {ES_INIT,0};
    return ES_PostToService(MyPriority, ThisEvent);
}
/****************************************************************************
 Function
     PostGridlinq

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostGridlinq( ES_Event ThisEvent ) {
  return ES_PostToService( MyPriority, ThisEvent);
}
/****************************************************************************
 Function
    RunGridlinq

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for the Spectre, moves between unpaired and paired. 
    Handles the analysis of the incoming data as described by the 218 
    comm protocol
****************************************************************************/
ES_Event RunGridlinq ( ES_Event CurrentEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT};
    GridlinqState NextState = CurrentState;
  
    // GridLinq state machine
    switch(CurrentState) {
        case Detached :
            //if we've lost the IR signal or whatever detects the cable
			if(CurrentEvent.EventType == CableTrigger) {
				//Prepare to enter the running state
				NextState = Running;
				ES_Event StartData = {Awaken, 0};
				ES_PostList00(StartData);
			}
			break;
		
		case Running :
			//do the running shit. Whatever that may be. 
			if(CurrentEvent.EventType == Disengage) {
                NextState = Detached;
            }
			if(CurrentEvent.EventType == WindReceived) {
				importData(Wind);
				printWindData();
			}
			else if(CurrentEvent.EventType == GPSReceived) {
				importData(GPS);
				printGPSData();
			}
			//once complete, set up hibernate
			if(CurrentEvent.EventType == GoToSleep) {
				NextState = Sleep;
				ES_Event StopData = {GoToSleep, 0};
				ES_PostList00(StopData);
			}
			break;
		
		case Sleep :
			// do the necessary sleeping shit. 
			//NextState = Running;
			if(CurrentEvent.EventType == CableTrigger) {
				//Prepare to enter the running state
				NextState = Running;
				ES_Event StartData = {Awaken, 0};
				ES_PostList00(StartData);
			}
			break;
	}		   
    CurrentState = NextState;
    return ReturnEvent;
}


/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
   importData

 Description
   moves the sensors current measurement information locally for loading
****************************************************************************/
static void importData(Sensor s) {
	switch(s) {
		case Wind :
			CV7 = GetWindData();
			break;
		case Distance :
            lidarData = getLidarData();
			break;
		case GPS :
			GPSdata = GetGPSData();
			break;
		case Compass :
            compassData = getCompassData();
			break;
	}
}

static void printGPSData(void) {
    printf("Time: \t\t%dh:%dm:%d.%ds\r\n",GPSdata.UTC_Time[0],GPSdata.UTC_Time[1],GPSdata.UTC_Time[2],GPSdata.UTC_Time[3]);
    printf("Latitude: \t%dd:%d.%dm %c\r\n",GPSdata.Latitude[0],GPSdata.Latitude[1],GPSdata.Latitude[2],GPSdata.NSIndicator);
    printf("Longitude: \t%dd:%d.%dm %c\r\n",GPSdata.Longitude[0],GPSdata.Longitude[1],GPSdata.Longitude[2],GPSdata.EWIndicator);
    //printf("PosInd: \t%d\r\n",GPSdata.PosIndicator);
    //printf("Satellites: \t%d\r\n",GPSdata.numOfSatellites);
    //printf("HDOP: \t\t%d\r\n",GPSdata.HDOP);
    //printf("Altitude: \t%d%c\r\n",GPSdata.MSLAltitude,GPSdata.AltUnits);
    //printf("Geoidal: \t%d%c\r\n\r\n",GPSdata.GeoidalSep,GPSdata.GeoUnits);
}

static void printWindData(void) {
    printf("Wind Speed = %d%c,\tWind Angle = %d,\tTemp = %d%c\r\n",CV7.WindSpeed,CV7.WindUnit,CV7.WindAngle,CV7.Temp,CV7.TempUnit);
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

