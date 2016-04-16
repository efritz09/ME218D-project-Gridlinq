/****************************************************************************
 Module
   LidarCom.c

Revision			Revised by: 
	1.0.0				Denny
	2.0.0				Alex

 Description
    This module handles communication with Lidar. Data is
    provided through the I2C service. The only output from the service
    is the distance and any error indication.

 Notes

 Edits:
	1.0.0	    Initial setup
	2.0.0 		Change to Lidar
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"

#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"


#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "bitdefs.h"
#include "termio.h"

#include "I2C.h"
#include "LidarCom.h"

/*----------------------------- Module Defines ----------------------------*/
// TODO: Will need to account for changing clock speeds (when chip is hibernating)
#define ONE_SEC 976
#define LIDAR_ADDRESS 0x62
#define LIDAR_I2C_MODULE 0

// Register definitions
#define DISTANCE_DATA 0x8F
#define CONTROL 0x00

typedef enum {
    PowerUp,
    IntervalSetup,
    ContinuousReadSetup,
    NormalOperation,
    Idle,
} LIDAR_State_t; 

/*---------------------------- Module Functions ---------------------------*/
static void getRegisterValue(uint8_t reg, uint8_t responseBytes);
static void writeRegisterValue(uint8_t reg, uint8_t value);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static LIDAR_State_t CurrentState;

static uint8_t I2CResponse[4]; // Place to house response when we read a register
static uint8_t I2CCommand[4];

uint16_t distance;
bool newRead = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitLidarCom
 Parameters
     uint8_t : the priorty of this service
 Returns
     bool, false if error in initialization, true otherwise
 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitLidarCom ( uint8_t Priority ) {
    MyPriority = Priority;
    CurrentState = PowerUp;
    
    printf("LidarCom initialized\r\n");
	
    // Post the initial transition event
    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     PostLidarCom
 Parameters
     EF_Event ThisEvent , the event to post to the queue
 Returns
     boolean False if the Enqueue operation failed, True otherwise
 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostLidarCom(ES_Event ThisEvent) {
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunLidarCom
 Parameters
   ES_Event : the event to process
 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description
   Implements LidarCom state machine
 Notes
****************************************************************************/
ES_Event RunLidarCom( ES_Event ThisEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT, 0}; // assume no errors
		
    if(ThisEvent.EventType == GetDistance) {
        distance = (I2CResponse[0] << 8) + I2CResponse[1];
        //printf("Distance: %d \r\n", distance);
    }
    
    switch(CurrentState) {
        case PowerUp :
            //printf("IN POWERUP \r\n");
            // If we just came out of initialization
            if(ThisEvent.EventType == ES_INIT) {
                // Start reads
                setI2CPostFunction(LIDAR_I2C_MODULE, PostLidarCom);
                writeRegisterValue(CONTROL,0x00);
            }
            else if (ThisEvent.EventType == I2C_TransferComplete) {
                // Move to interval setup
                writeRegisterValue(0x45,0xc8);
                CurrentState = IntervalSetup;
            }
            else if(ThisEvent.EventType == I2C_TransferFailure) {
                // Repeat if not successful
                writeRegisterValue(CONTROL,0x00);
            }
            break; 
						
        case IntervalSetup : 
            //printf("In Normal \r\n");
            if (ThisEvent.EventType == I2C_TransferComplete) {
                // Move to continuous read setup
                writeRegisterValue(0x11,0xff);
                CurrentState = ContinuousReadSetup;
            }
            else if (ThisEvent.EventType == I2C_TransferFailure) {
                // Retry read if failure occurs
                writeRegisterValue(0x45,0xc8);
            }
			break;
						
		case ContinuousReadSetup : 
			//printf("In Normal \r\n");
			if (ThisEvent.EventType == I2C_TransferComplete) {
                // Move to Normal operation
				writeRegisterValue(0x00,0x04);
				CurrentState = NormalOperation;
			}
			else if (ThisEvent.EventType == I2C_TransferFailure) {
				// Retry read if failure occurs
				writeRegisterValue(0x11,0xff);
			}				
			break;
            
        case NormalOperation : 
			//printf("In Normal \r\n");
			if (ThisEvent.EventType == I2C_TransferComplete) {
				// Move to idle
				getRegisterValue(DISTANCE_DATA, 2);
				newRead = false;
				CurrentState = Idle;
			}
			else if (ThisEvent.EventType == I2C_TransferFailure) {
				// Retry read if failure occurs
				writeRegisterValue(0x00,0x04);
			}			
            break;
            
		case Idle:
			//printf("In Idle \r\n");
			if(ThisEvent.EventType == GetDistance) {
                // Initiate next read
				getRegisterValue(DISTANCE_DATA, 2);
				newRead = true;
			}
			else if(ThisEvent.EventType == I2C_TransferFailure) {
				// Initiate next read
				getRegisterValue(DISTANCE_DATA, 2);
			}
			else if (ThisEvent.EventType == I2C_TransferComplete && newRead) {
				// Read data in distance data register
                //getRegisterValue(DISTANCE_DATA, 2);
				newRead = false;
			}
			break;
						
    }
    return ReturnEvent;
}


LidarData_t getLidarData(void) {
	// Update data
	ES_Event newEvent = {GetDistance, distance};
	PostLidarCom(newEvent);
	
	// Return most recent data
	LidarData_t newData = {I2CResponse[0], I2CResponse[1]};
	return newData;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void getRegisterValue(uint8_t reg, uint8_t responseBytes) {
    I2CCommand[0] = reg;
    I2CMessage_t newMessage = {LIDAR_I2C_MODULE, LIDAR_ADDRESS, I2CCommand, 1, I2CResponse, responseBytes};
    I2CStart(&newMessage);
    //start a timeout
}

static void writeRegisterValue(uint8_t reg, uint8_t value) {
    I2CCommand[0] = reg;
    I2CCommand[1] = value;
    I2CMessage_t newMessage = {LIDAR_I2C_MODULE, LIDAR_ADDRESS, I2CCommand, 2, I2CResponse, 0};
    I2CStart(&newMessage);
    //start a timeout
}
