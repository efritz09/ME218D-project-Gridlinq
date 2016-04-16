/****************************************************************************
 Module
  Xbee.c

 Revision     Revised by: 
  1.0.0       Eric

 Description
  UART for Xbee communications

 Edits:

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "XbeeComms.h"
#include "Xbee.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define START_DELIMETER     0x7E
#define XBEE_TIMEOUT        200

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.*/


/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static XbeeCommsState CurrentState;

static uint16_t DataLen = 0x00;
static uint8_t rxData[12] = {0x00};

static uint16_t arrayIndex = 0;
static uint8_t chksum = 0x00;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitXbeeComms

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
****************************************************************************/
bool InitXbeeComms ( uint8_t Priority ) { 
  MyPriority = Priority;

  CurrentState = xWaitForStartState;
  printf("Xbee packet receiver initialized\r\n");
  
  // Post the initial transition event
  ES_Event ThisEvent = {ES_INIT, 0};
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    PostXbeeComms

 Parameters
    EF_Event ThisEvent ,the event to post to the queue

 Returns
    bool false if the Enqueue operation failed, true otherwise

 Description
    Posts an event to this state machine's queue
****************************************************************************/
bool PostXbeeComms( ES_Event ThisEvent ) {
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunXbeeComms

 Parameters
    ES_Event : the event to process

 Returns
    ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
    Decodes xbee packets. Posts to the xbee service when a full packet has 
    been received with a valid checksum
****************************************************************************/
ES_Event RunXbeeComms ( ES_Event CurrentEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT};
    XbeeCommsState NextState = CurrentState;

    switch(CurrentState) {
        case xWaitForStartState:
            //Check if we have received a start delimeter
            if(CurrentEvent.EventType == XbeeByteReceived) { 
                if(CurrentEvent.EventParam == START_DELIMETER) {
                    NextState = xWaitForMSBState;
                    ES_Timer_InitTimer(XbeeTimer,XBEE_TIMEOUT);
                }
            }
            break;
        case xWaitForMSBState:
            if(CurrentEvent.EventType == XbeeByteReceived) {
                // Store MSB value
                DataLen = CurrentEvent.EventParam<<8;
                NextState = xWaitForLSBState;
                ES_Timer_InitTimer(XbeeTimer,XBEE_TIMEOUT);
            }
            // If the data was interrupted for some reason
            else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == XbeeTimer) {
                NextState = xWaitForStartState;
                printf("message timeout\r\n");
            }
            break;
        case xWaitForLSBState:
            if(CurrentEvent.EventType == XbeeByteReceived) {
                // Store LSB value
                DataLen += CurrentEvent.EventParam;
                NextState = xProcessDataState;
                ES_Timer_InitTimer(XbeeTimer,XBEE_TIMEOUT);
                //printf("Data length: %d\r\n", DataLen);
            }
            else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == XbeeTimer) {
                NextState = xWaitForStartState;
                printf("message timeout\r\n");
            }
            break;
        case xProcessDataState:
            if(CurrentEvent.EventType == XbeeByteReceived) {
                ES_Timer_InitTimer(XbeeTimer,XBEE_TIMEOUT);
                // While there's still data coming in
                if(DataLen > arrayIndex) { 
                    // Store the value in the array
                    rxData[arrayIndex++] = CurrentEvent.EventParam;
                    // Update checksum
                    chksum += CurrentEvent.EventParam;
                }
                // If we've reached our last value
                else if(DataLen == arrayIndex) {
                    if((0xFF-chksum) == CurrentEvent.EventParam) { //check the checksum value
                        ES_Timer_StopTimer(XbeeTimer);
                        //printf("Packet received\r\n");
                        ES_Event ThisEvent = {XbeePacketReceived,DataLen}; //return event for a good checksum, otherwise ignore
                        PostXbee(ThisEvent);
                    } 
                    else {
                        printf("Message extraction failed\r\n");
                    }
                    // Reset values regardless
                    NextState = xWaitForStartState;
                    chksum = 0x00;
                    arrayIndex = 0;
                }
                else { //somehow we've gone over?
                    chksum = 0x00;
                    arrayIndex = 0;
                    NextState = xWaitForStartState; 
                }
            }
            else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == XbeeTimer) {
                NextState = xWaitForStartState;
                //printf("arrayindex = %d\r\n",arrayIndex);
                arrayIndex = 0;
                printf("the fuck? timeout\r\n");
            }
            break;
    }
    CurrentState = NextState;
    return ReturnEvent;
}

/****************************************************************************
 Function
   GetRxData

 Description
   Returns a pointer to the saved data
****************************************************************************/
uint8_t* GetRxData(void) {
    return rxData;
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
