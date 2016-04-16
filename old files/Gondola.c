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

#include <stdint.h>

#include "Gridlinq.h"

// TivaWare Headers
#include "driverlib/sysctl.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define ONE_SEC 						976

#define CABLE_TIMER					5000


/* COMMS DEFINES
// Receive defines:
#define API_ID              0
#define SOURCE_ADDR_MSB     1
#define SOURCE_ADDR_LSB     2
#define RSSI                3
#define RX_OPTIONS          4
#define RX_HEADER           5
// For broadcast
#define REQ_PAIR            6
#define BROAD_CHKSUM        7
// For control
#define THRUST              6
#define ORIENT              7
#define ACTION              8
#define EXA                 9
#define EXB                 10
#define EXC                 11
#define CTRL_CHKSUM         12

#define BROADCAST           0x06

// Transmit defines:
#define DEST_ADDR_MSB       5
#define DEST_ADDR_LSB       6
#define TX_OPTIONS          7
#define TX_HEADER           8
#define SDATA               9
#define ACKNOWLEDGE         9
#define TX_CHKSUM           10

#define PAIR_REQ            0x01
#define PAIR_ACK            0x02
#define CTRL                0x03
#define STATUS              0x04        

#define POP_MASK            0x01
#define UNPAIR_MASK         0x80
#define COMM_TIMEOUT        1000    
#define REJECT_TIMEOUT      10000
*/
/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/

// Interrupt functions


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
static GondolaState CurrentState;



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
bool InitGridlinq ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
  CurrentState = Detached;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}
/****************************************************************************
 Function
     PostGridlinq

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostGridlinq( ES_Event ThisEvent )
{
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
ES_Event RunGridlinq ( ES_Event CurrentEvent )
{
  ES_Event ReturnEvent = {ES_NO_EVENT};
  GondolaState NextState = CurrentState;
  
  // SPECTRE Xbee state machine
  switch(CurrentState) {
    case Detached :
			
		
			//if we've lost the IR signal or whatever detects the cable
			if(CurrentEvent.EventType == CableTrigger) {
				//Prepare to enter the running state
				CurrentState = Running;
			}
			
			break;
		
			
		case Running :
			
			//do the running shit. Whatever that may be. 
			break;
		
		
		case Sleep :
			
			// do the necessary sleeping shit. 
		
			break;


	}		
      
  CurrentState = NextState;
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/



/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

