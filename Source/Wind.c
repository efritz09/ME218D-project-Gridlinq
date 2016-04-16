/****************************************************************************
 Module
	Wind.c

 Revision			Revised by: 
	1.0.0				Eric

 Description
	Wind Sensor Communication and decoding

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

#include "Wind.h"
#include "Gridlinq.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000				// 40Mhz
#define ALL_BITS 		    (0xff<<2)
#define START_DELIMETER	    0x24
#define WIND_TIMEOUT        18000

#define CHECKSUM_ID         0x2A // '*' //0x35 for testing
#define WIND_ID_CHECK       0x4c
#define TEMP_ID_CHECK       0x50
#define WIND_LENGTH         23
#define TEMP_LENGTH         17

#define TEMP_UNIT           14
#define TEMP_1              8
#define TEMP_2              9
#define TEMP_3              10
#define TEMP_4              12

#define WIND_UNIT           20
#define	WIND_ANGLE_1        6
#define	WIND_ANGLE_2	    7
#define	WIND_ANGLE_3	    8
#define	WIND_ANGLE_4	    10

#define WIND_SPEED_1	    14
#define WIND_SPEED_2	    15
#define WIND_SPEED_3		16
#define WIND_SPEED_4		18

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.*/
static void ParseTempData(void);
static void ParseWindData(void);
static char hextodec(char val);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static WindState CurrentState;
Wind_Data Winddata;

static uint8_t rawData[23] = {0x00};
static uint8_t arrayIndex = 0;
static uint8_t chksum = 0x00;
static uint8_t sensorChecksum = 0;

static uint16_t WindAngle;
static uint16_t WindSpeed;
static uint16_t	Temp;
static bool windVals = true;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitWind

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
****************************************************************************/
bool InitWind( uint8_t Priority ) {
    MyPriority = Priority;
	CurrentState = WaitForStartState;
	printf("Wind comms initialized\r\n");
    
    // post the initial transition event
    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     PostWind

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostWind( ES_Event ThisEvent ) {
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunWind

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here

****************************************************************************/
ES_Event RunWind ( ES_Event CurrentEvent ) {
	ES_Event ReturnEvent = {ES_NO_EVENT};
	WindState NextState = CurrentState;

	switch(CurrentState) {
        
		case Stasis :
			if(CurrentEvent.EventType == Awaken) {
				NextState = WaitForStartState;
				printf("Awakening wind sensor\r\n");
			}
			break;
            
		case WaitForStartState:
			//if a byte has been received, start the transition
			if(CurrentEvent.EventType == WindByteReceived){ 
				if(CurrentEvent.EventParam == START_DELIMETER){ // CHANGES
					//printf("%x\r\n",CurrentEvent.EventParam);
					NextState = WaitForTypeCode;
					ES_Timer_InitTimer(WindTimer,WIND_TIMEOUT);
					//printf("entering processdatastate\r\n");
					chksum = 0x00;
					arrayIndex = 0;
					//printf("Incoming message...\n\r");
				}
			}	else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
			}
			
			break;
            
		case WaitForTypeCode :
			if(CurrentEvent.EventType == WindByteReceived) {
				//printf("received data:  ");
				ES_Timer_InitTimer(WindTimer,WIND_TIMEOUT);
				rawData[arrayIndex++] = CurrentEvent.EventParam;
				chksum ^= CurrentEvent.EventParam;
				if(CurrentEvent.EventParam == 'W') {
					NextState = ProcessTempData;
					//printf(" %x\t\t\t\t Temp Data\r\n",CurrentEvent.EventParam);
				}
				else if(CurrentEvent.EventParam == 'I') {
					NextState = ProcessWindData;
					//printf(" %x\t\t\t\t Wind Data\r\n",CurrentEvent.EventParam);
				}
				else NextState = WaitForStartState;
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == WindTimer) {
				NextState = WaitForStartState;
				//printf("arrayindex = %d\r\n",arrayIndex);
				printf("the fuck? rx timeout\r\n");
			}	else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
			}
				
			break;
            
		case ProcessWindData:
			if(CurrentEvent.EventType == WindByteReceived) {
				//printf("received data:  ");
				ES_Timer_InitTimer(WindTimer,WIND_TIMEOUT);
				//while there's still data coming in
				
				if(WIND_LENGTH > arrayIndex) { // CHANGES 
					//store the value in the array
					rawData[arrayIndex++] = CurrentEvent.EventParam;
					//printf(" %x\r\n",CurrentEvent.EventParam);
					chksum ^= CurrentEvent.EventParam;
				}
				//if we've reached our last value
				else if(WIND_LENGTH == arrayIndex) { //Check for the checksum ID
					//printf("checksum %x      recieved: ",chksum);
					//printf(" %x\r\n",CurrentEvent.EventParam);
					if(CurrentEvent.EventParam == CHECKSUM_ID) { //check the checksum value
						//printf("End of message, proceeding to checksum\r\n");
						NextState = CheckSum1;
						windVals = true;
					} else { // we've fucked up somewhere
					//reset values regardless
						NextState = WaitForStartState; // CHANGES
					}
				}
				else { //somehow we've gone over?
					NextState = WaitForStartState; // CHANGES
					//printf("how the hell did this happen?\r\n");
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == WindTimer) {
				NextState = WaitForStartState;
				//printf("arrayindex = %d\r\n",arrayIndex);
				printf("the fuck? rx timeout\r\n");
			}	else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
			}
			break;
		
		case ProcessTempData :
			if(CurrentEvent.EventType == WindByteReceived) {
				//printf("received data:  ");
				ES_Timer_InitTimer(WindTimer,WIND_TIMEOUT);
				
				if(TEMP_LENGTH > arrayIndex) {
					rawData[arrayIndex++] = CurrentEvent.EventParam;
					//printf(" %x\r\n",CurrentEvent.EventParam);
					chksum ^= CurrentEvent.EventParam;
				}
				else if(TEMP_LENGTH == arrayIndex) {
					//printf("checksum %x      recieved: ",chksum);
					//printf(" %x\r\n",CurrentEvent.EventParam);
					if(CurrentEvent.EventParam == CHECKSUM_ID) { //check the checksum value
						//printf("End of message, proceeding to checksum\r\n");
						NextState = CheckSum1;
						windVals = false;
					} else { // we've fucked up somewhere
                        //reset values regardless
						NextState = WaitForStartState; // CHANGES
					}
				}
				else { //somehow we've gone over?
					NextState = WaitForStartState; // CHANGES
					//printf("how the hell did this happen?\r\n");
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == WindTimer) {
				NextState = WaitForStartState;
				//printf("arrayindex = %d\r\n",arrayIndex);
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
			}
					
			break;
			
		case CheckSum1 :
			if(CurrentEvent.EventType == WindByteReceived) {
				//printf("first value = %d\t\t",CurrentEvent.EventParam);
				//printf("converted = %d\r\n",(16*(CurrentEvent.EventParam - '0')));
				CurrentEvent.EventParam = hextodec(CurrentEvent.EventParam);
				sensorChecksum = 16*(CurrentEvent.EventParam);
				NextState = CheckSum2;
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == WindTimer) {
				NextState = WaitForStartState;
				printf("the fuck? rx timeout\r\n");
			}	
            else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
			}
			
			break;
			
		case CheckSum2 :
			if(CurrentEvent.EventType == WindByteReceived) {
				//printf("second value = %d\t\t",CurrentEvent.EventParam);
				//printf("converted = %d\r\n",((CurrentEvent.EventParam - '0')));
				CurrentEvent.EventParam = hextodec(CurrentEvent.EventParam);
				sensorChecksum += (CurrentEvent.EventParam);
				//printf("chksum = %d,   sensor = %d\r\n",chksum,sensorChecksum);
				if((chksum) == sensorChecksum) {
					//printf("extraction successful\r\n");
					if(windVals) ParseWindData();
					else ParseTempData();
					ES_Timer_StopTimer(WindTimer);
					ES_Event ThisEvent = {WindReceived,0};
					PostGridlinq(ThisEvent);
				}
				else printf("extraction unsuccssful, bad checksum \r\n");
			}
			
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == WindTimer) {
				printf("the fuck? rx timeout\r\n");
			}	else if(CurrentEvent.EventType == GoToSleep) {
				NextState = Stasis;
				printf("send wind sensor to stasis\r\n");
				break;
			}
			NextState = WaitForStartState;
			ES_Timer_StopTimer(WindTimer);
			break;
	}
	CurrentState = NextState;
  return ReturnEvent;
}

/***************************************************************************
 public functions
 ***************************************************************************/
Wind_Data GetWindData(void) {
	return Winddata;
}

/***************************************************************************
 private functions
 ***************************************************************************/
 void UART2ISR( void ) {
	// If the interrupt response is from Rx (new data)
	if((HWREG(UART2_BASE+UART_O_RIS)& UART_RIS_RXRIS) >0){
		// Clear the source of the interrupt
		HWREG(UART2_BASE + UART_O_ICR) |= UART_ICR_RXIC;
		//post byte received to the wind
		ES_Event ThisEvent;
		ThisEvent.EventType = WindByteReceived;
		ThisEvent.EventParam = HWREG(UART2_BASE + UART_O_DR);
		PostWind(ThisEvent);
	}
}

static char hextodec(char val) {
	//printf("value to convert = %c\r\n",val);
	if(val == 'F' || val == 'E' || val == 'D' || val == 'C' || val == 'B' || val == 'A'){
		val = val - 'A' + 10;
	}
	else{
			val = val - '0';
	}
	//printf("converted value = %d\r\n",val);
	return val;
}


static void ParseTempData(void) {

	//check for temp data
	if((rawData[0] ^ rawData[1] ^ rawData[2] ^ rawData[3] ^ rawData[4]) == TEMP_ID_CHECK) { //temp data
		//printf("Temp Data\r\n");
		Temp = (rawData[TEMP_1] - '0')*1000;
		Temp += (rawData[TEMP_2] - '0')*100;
		Temp += (rawData[TEMP_3] - '0')*10;
		Temp += (rawData[TEMP_4] - '0');
		Winddata.Temp = Temp;
		Winddata.TempUnit = rawData[TEMP_UNIT];
		//printf("Temp: %c%c%c%c%c%c\r\n",rawData[TEMP_1],rawData[TEMP_2],rawData[TEMP_3],0x2e,rawData[TEMP_4],TempUnits);
	}
	//check for wind data
}

static void ParseWindData(void) {
	if((rawData[0] ^ rawData[1] ^ rawData[2] ^ rawData[3] ^ rawData[4]) == WIND_ID_CHECK) { //wind data
		//printf("Wind Data\r\n");

		WindAngle = (rawData[WIND_ANGLE_1] - '0')*1000;
		WindAngle += (rawData[WIND_ANGLE_2] - '0')*100;
		WindAngle += (rawData[WIND_ANGLE_3] - '0')*10;
		WindAngle += (rawData[WIND_ANGLE_4] - '0');
		Winddata.WindAngle = WindAngle;
		//printf("Wind Angle: %c%c%c%c%c\r\n",rawData[WIND_ANGLE_1],rawData[WIND_ANGLE_2],rawData[WIND_ANGLE_3],0x2e,rawData[WIND_ANGLE_4]);
		
		WindSpeed = (rawData[WIND_SPEED_1] - '0')*1000;
		WindSpeed += (rawData[WIND_SPEED_2] - '0')*100;
		WindSpeed += (rawData[WIND_SPEED_3] - '0')*10;
		WindSpeed += (rawData[WIND_SPEED_4] - '0');
		Winddata.WindSpeed = WindSpeed;
		Winddata.WindUnit = rawData[WIND_UNIT];
		//printf("Wind Speed: %c%c%c%c%c%c\r\n",rawData[WIND_SPEED_1],rawData[WIND_SPEED_2],rawData[WIND_SPEED_3],0x2e,rawData[WIND_SPEED_4],WindUnits);
	}
	//check for otherwise
	else printf("Bad data\r\n");
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
