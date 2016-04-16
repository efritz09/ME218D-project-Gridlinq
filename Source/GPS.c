/****************************************************************************
 Module
	GPS.c

 Revision			Revised by: 
	1.0.0				Eric

 Description
	GPS Communication and decoding

 Edits:
	1.0.0				Working comms with decoding

****************************************************************************/


/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
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

#include "GPS.h"
#include "Gridlinq.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000   // 40Mhz
#define ALL_BITS            (0xff<<2)
#define START_DELIMETER     0x24
#define GPS_TIMEOUT         18000	

#define CHECKSUM_ID         0x2A // '*' //0x35 for testing

#define GPS_LENGTH          80

#define ID_INDEX            5
#define GGA_ID              0x56
#define GSA_ID              0x42
#define GSV_ID              0x55
#define RMC_ID              0x4B
#define VTG_ID              0x52

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
static void ParseGGAData(void);
static void ParseGSAData(void);
static void ParseGSVData(void);
static void ParseRMCData(void);
static void ParseVTGData(void);

static char hextodec(char val);
/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static GPSState CurrentState;
static GPSID ID;
GPS_Data data;// = {0.0,0.0,0.0}; //starting values

static uint8_t rawData[GPS_LENGTH] = {0x00};

static uint8_t arrayIndex = 0;
static uint8_t chksum = 0x00;
static uint8_t sensorChecksum = 0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitGPS

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
****************************************************************************/
bool InitGPS( uint8_t Priority ) {
    MyPriority = Priority;
	//CurrentState = gStasis;
    CurrentState = gWaitForStartState;
	printf("GPS comms initialized\r\n");
    // post the initial transition event
    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     PostGPS

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returnss
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostGPS( ES_Event ThisEvent ) {
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunGPS

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Runs the gps state machine
****************************************************************************/
ES_Event RunGPS ( ES_Event CurrentEvent ) {
	ES_Event ReturnEvent = {ES_NO_EVENT};
	GPSState NextState = CurrentState;

	switch(CurrentState) {
		case gStasis :
			if(CurrentEvent.EventType == Awaken) {
				NextState = gWaitForStartState;
				printf("Awakening GPS sensor\r\n");
			}
			break;
		case gWaitForStartState:
			//if a byte has been received, start the transition
			if(CurrentEvent.EventType == GPSByteReceived){
				if(CurrentEvent.EventParam == START_DELIMETER){ // CHANGES
					//printf("%x\r\n",CurrentEvent.EventParam);
					NextState = gWaitForTypeCode;
					ES_Timer_InitTimer(GPSTimer,GPS_TIMEOUT);
					//printf("entering processdatastate\r\n");
					chksum = 0x00;
					arrayIndex = 0;
					//printf("Incoming message...\n\r");
					//printf("received data:   %c\r\n",CurrentEvent.EventParam);
				}
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = gStasis;
				printf("send GPS sensor to stasis\r\n");
			}
			
			break;
		case gWaitForTypeCode :
			if(CurrentEvent.EventType == GPSByteReceived) {
				//printf("received data:   %c\r\n",CurrentEvent.EventParam);
				ES_Timer_InitTimer(GPSTimer,GPS_TIMEOUT);
				
				chksum ^= CurrentEvent.EventParam;
				rawData[arrayIndex++] = CurrentEvent.EventParam;
				//printf("Character = %c : %x, checksum = %x, arrayindex = %d\r\n",CurrentEvent.EventParam,CurrentEvent.EventParam,chksum,arrayIndex);
				if(arrayIndex == ID_INDEX) {
					switch(chksum) {
						case GGA_ID :
							ID = GGA;
							//printf("GGA data incoming\r\n");
							NextState = ProcessGPSState;
							break;
						case GSA_ID :
							ID = GSA;
							//printf("GSA data incoming\r\n");
							NextState = gWaitForStartState;
							break;
						case GSV_ID :
							//printf("GSV data incoming\r\n");
							ID = GSV;
							NextState = gWaitForStartState;
							break;
						case RMC_ID :
							//printf("RMC data incoming\r\n");
							ID = RMC;
							NextState = gWaitForStartState;
							break;
						case VTG_ID :
							//printf("VTG data incoming\r\n");
							ID = VTG;
							NextState = gWaitForStartState;
							break;
						default :
							//printf("Shit data incoming\r\n");
							NextState = gWaitForStartState;
							break;
					}
				}
                else if (arrayIndex > ID_INDEX) {
					NextState = gWaitForStartState;
					printf("error, index overflow: ID\r\n");
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == GPSTimer) {
				NextState = gWaitForStartState;
				//printf("arrayindex = %d\r\n",arrayIndex);
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = gStasis;
				printf("send GPS sensor to stasis\r\n");
			}
			break;
			
			
		case ProcessGPSState:
			if(CurrentEvent.EventType == GPSByteReceived) {
				//printf("\tdata:   %c\r\n",CurrentEvent.EventParam);
				ES_Timer_InitTimer(GPSTimer,GPS_TIMEOUT);
				//while there's still data coming in
				
				if(CurrentEvent.EventParam == CHECKSUM_ID) { //check the checksum value
					//printf("End of message, proceeding to checksum\r\n");
					//printf("checksum %x      recieved: ",chksum);
					//printf(" %c\r\n",CurrentEvent.EventParam);
					NextState = gCheckSum1;
				}
				
				else if(GPS_LENGTH > arrayIndex) { // CHANGES 
					//store the value in the array
					rawData[arrayIndex++] = CurrentEvent.EventParam;
					chksum ^= CurrentEvent.EventParam;
				}				
				else { // we've fucked up somewhere
					//printf("GPS_LENGTH : %d\t\t arrayIndex : %d\r\n",GPS_LENGTH,arrayIndex);
					NextState = gWaitForStartState; // CHANGES
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == GPSTimer) {
				NextState = gWaitForStartState;
				printf("arrayindex = %d\r\n",arrayIndex);
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = gStasis;
				printf("send GPS sensor to stasis\r\n");
			}
			break;
		
		
			
			
		case gCheckSum1 :
			if(CurrentEvent.EventType == GPSByteReceived) {
				CurrentEvent.EventParam = hextodec(CurrentEvent.EventParam);
				//printf("first value = %c : %d : %x\r\n",CurrentEvent.EventParam,CurrentEvent.EventParam,CurrentEvent.EventParam);
				//check for upper/lower case bullshit
				sensorChecksum = 16*(CurrentEvent.EventParam);
				
                //printf("converted = %d\r\n",sensorChecksum);
				NextState = gCheckSum2;
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == GPSTimer) {
				NextState = gWaitForStartState;
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = gStasis;
				printf("send GPS sensor to stasis\r\n");
			}
			
			break;
			
		case gCheckSum2 :
			if(CurrentEvent.EventType == GPSByteReceived) {
				CurrentEvent.EventParam = hextodec(CurrentEvent.EventParam);
				//printf("second value = %c : %d : %x\r\n",CurrentEvent.EventParam,CurrentEvent.EventParam,CurrentEvent.EventParam);
				//printf("converted = %d\r\n",((CurrentEvent.EventParam - '0')));
				sensorChecksum += (CurrentEvent.EventParam);
                
                //printf("converted = %d\r\n",sensorChecksum);
                //printf("chksum = %d,   sensor = %d\r\n",chksum,sensorChecksum);
				if((chksum) == sensorChecksum) {
					//printf("extraction successful\r\n");
					switch(ID) {
						case GGA :
							ParseGGAData();
							break;
						case GSA :
							ParseGSAData();
							break;
						case GSV :
							ParseGSVData();
							break;
						case RMC :
							ParseRMCData();
							break;
						case VTG :
							ParseVTGData();
							break;
					}
					
					ES_Timer_StopTimer(GPSTimer);
					ES_Event ThisEvent = {GPSReceived,0};
					PostGridlinq(ThisEvent);
				}
				else printf("extraction unsuccssful, bad checksum \r\n");
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == GPSTimer) {
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = gStasis;
				printf("send GPS sensor to stasis\r\n");
				break;
			}
			NextState = gWaitForStartState;
			ES_Timer_StopTimer(GPSTimer);
			break;
	}
	CurrentState = NextState;
  return ReturnEvent;
}

/***************************************************************************
 public functions
 ***************************************************************************/
GPS_Data GetGPSData(void) {
	return data;
}

/***************************************************************************
 private functions
 ***************************************************************************/
void UART5ISR( void ) {
	// If the interrupt response is from Rx (new data)
	if((HWREG(UART5_BASE+UART_O_RIS)& UART_RIS_RXRIS) > 0) {
		// Clear the source of the interrupt
		HWREG(UART5_BASE + UART_O_ICR) |= UART_ICR_RXIC;
		//post byte received to the GPS
		ES_Event ThisEvent = {GPSByteReceived, HWREG(UART5_BASE + UART_O_DR)};
		PostGPS(ThisEvent);
        //printf("%c\r\n",ThisEvent.EventParam);
	}
}

static char hextodec(char val) {
	if(val == 'F' || val == 'E' || val == 'D' || val == 'C' || val == 'B' || val == 'A') {
		val = val - 'A' + 10;
	}
	else{
        val = val - '0';
	}
	//printf("converted value = %d\r\n",val);
	return val;
}

static void ParseGGAData(void) {
	if(ID != GGA) return;
	//process dat shit
	int numComma = 0;
	/*        $GPGGA,032620.000,3725.5297,N,12210.4191,W,2,10,0.85,37.2,M,-25.7,M,0000,0000*
	rawData[0] = 'G';   G
	rawData[1] = 'P';   P
	rawData[2] = 'G';   G
	rawData[3] = 'G';   G
	rawData[4] = 'A';   A
	rawData[5] = ',';   ,
	rawData[6] = '0';   0
	rawData[7] = '6';   3
	rawData[8] = '4';   2
	rawData[9] = '9';   6
	rawData[10]= '5';   2
	rawData[11]= '1';   0
	rawData[12]= '.';   .
	rawData[13]= '0';   0
	rawData[14]= '0';   0
	rawData[15]= '0';   0
	rawData[16]= ',';   ,
	rawData[17]= '2';   3
	rawData[18]= '3';   7
	rawData[19]= '0';   2
	rawData[20]= '7';   5
	rawData[21]= '.';   .
	rawData[22]= '1';   5
	rawData[23]= '2';   2
	rawData[24]= '5';   9
	rawData[25]= '6';   7
	rawData[26]= ',';   ,
	rawData[27]= 'N';   N
	rawData[28]= ',';   ,
	rawData[29]= '1';   1
	rawData[30]= '2';   2
	rawData[31]= '0';   2
	rawData[32]= '1';   1
	rawData[33]= '6';   0
	rawData[34]= '.';   .
	rawData[35]= '4';   4
	rawData[36]= '4';   1
	rawData[37]= '3';   9
	rawData[38]= '8';   1
	rawData[39]= ',';   ,
	rawData[40]= 'E';   W
	rawData[41]= ',';   ,
	rawData[42]= '1';   2
	rawData[43]= ',';   ,
	rawData[44]= '8';   1
	rawData[45]= ',';   0           <--- Here is where it deviates
	rawData[46]= '0';   ,
	rawData[47]= '.';   0
	rawData[48]= '9';   .
	rawData[49]= '5';   8
	rawData[50]= ',';   5
	rawData[51]= '3';   ,
	rawData[52]= '9';   3
	rawData[53]= '.';   7
	rawData[54]= '9';   .
	rawData[55]= ',';   2
	rawData[56]= 'M';   ,
	rawData[57]= ',';   M
	rawData[58]= '1';   ,
	rawData[59]= '7';   -
	rawData[60]= '.';   2
	rawData[61]= '8';   5
	rawData[62]= ',';   .
	rawData[63]= 'M';   7
	rawData[64]= ',';   ,
	rawData[65]= ',';   M
    rawData[66];        0
    rawData[67];        0
    rawData[68];        0
    rawData[69];        0
    rawData[70];        ,
    rawData[71];        0
    rawData[72];        0
    rawData[73];        0
    rawData[74];        0
    rawData[75];        * 
	*/
	for(int i = 0; i < GPS_LENGTH; i++) {
		//printf("%c",rawData[i]);
		if(rawData[i] != ',') continue;
		numComma++;
		if(rawData[i+1] == ',') continue;
		
		switch(numComma) {
			case 1 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.UTC_Time[0] = (rawData[i+1]-'0')*10 + (rawData[i+2]-'0');
				data.UTC_Time[1] = (rawData[i+3]-'0')*10 + (rawData[i+4]-'0');
				data.UTC_Time[2] = (rawData[i+5]-'0')*10 + (rawData[i+6]-'0');
				data.UTC_Time[3] = (rawData[i+8]-'0')*100 + (rawData[i+9]-'0')*10 + (rawData[i+10]-'0');
				break;
			case 2 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.Latitude[0] = (rawData[i+1]-'0')*10 + (rawData[i+2]-'0');
				data.Latitude[1] = (rawData[i+3]-'0')*10 + (rawData[i+4]-'0');
				data.Latitude[2] = (rawData[i+6]-'0')*1000 + (rawData[i+7]-'0')*100 + (rawData[i+8]-'0')*10 + (rawData[i+9]-'0');
				break;
			case 3 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.NSIndicator = rawData[i+1];
				break;
			case 4 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.Longitude[0] = (rawData[i+1]-'0')*100 + (rawData[i+2]-'0')*10 + (rawData[i+3]-'0');
				data.Longitude[1] = (rawData[i+4]-'0')*10 + (rawData[i+5]-'0');
				data.Longitude[2] = (rawData[i+7]-'0')*1000 + (rawData[i+8]-'0')*100 + (rawData[i+9]-'0')*10 + (rawData[i+10]-'0');
				break;
			case 5 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.EWIndicator = rawData[i+1];
				break;
			case 6 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.PosIndicator = (rawData[i+1]-'0');
				break;
			case 7 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.numOfSatellites = (rawData[i+1]-'0');
				if(rawData[i+2] != ',') {
					data.numOfSatellites *= 10;
					data.numOfSatellites += (rawData[i+2]-'0');
				}
				break;
			case 8 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.HDOP = (rawData[i+1]-'0')*100 + (rawData[i+3]-'0')*10 + (rawData[i+4]-'0');
				break;
			case 9 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.MSLAltitude = (rawData[i+1]-'0')*100 + (rawData[i+2]-'0')*10 + (rawData[i+4]-'0');
				break;
			case 10 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.AltUnits = rawData[i+1];
				break;
			case 11 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.GeoidalSep = (rawData[i+1]-'0')*100 + (rawData[i+2]-'0')*10 + (rawData[i+4]-'0');
				break;
			case 12 :
				//printf("Comma %d at index: %d\r\n",numComma,i);
				data.GeoUnits = rawData[i+1];
				break;
		}
	}
	//printf("\r\n");
	return;
}

static void ParseGSAData(void) {
	if(ID != GSA) return;
	//process dat shit
	return;
}
static void ParseGSVData(void) {
	if(ID != GSV) return;
	//process dat shit
	return;
}	
static void ParseRMCData(void) {
	if(ID != RMC) return;
	//process dat shit
	return;
}	
static void ParseVTGData(void) {
	if(ID != VTG) return;
	//process dat shit
	return;
}	

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
