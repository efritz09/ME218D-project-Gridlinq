/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1 

 Description
   Unit testing and controlling code execution via keystrokes.
   
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Events.h"
#include "ES_PostList.h"
#include "ES_ServiceHeaders.h"
#include "ES_Port.h"
#include "EventCheckers.h"

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   Checks to see if a new key from the keyboard is detected and, if so, 
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker 
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
****************************************************************************/
bool Check4Keystroke(void) {
    if ( IsNewKeyReady() ) {
        ES_Event ThisEvent;
        ThisEvent.EventType = ES_NEW_KEY;
        ThisEvent.EventParam = GetNewKey();
      
        if ( ThisEvent.EventParam == '1') {
            ES_Event newEvent = {CableTrigger, 1};
            //PostClamp( newEvent );
            PostGridlinq(newEvent);
            printf("POST - Cable Trigger \r\n");
        }
        else if ( ThisEvent.EventParam == '2'){
            ES_Event newEvent = {ES_TIMEOUT, DebounceTimer};
            PostClamp( newEvent );
            printf("POST - Close Event \r\n");
        }
        else if ( ThisEvent.EventParam == '3'){
			ES_Event newEvent = {Disengage, 3};
			PostClamp( newEvent );
			printf("POST - Open Event \r\n");
        }
//		else if ( ThisEvent.EventParam == '4'){
//			ES_Event newEvent = {BeginTransfer, 0};
//			PostI2C( newEvent );
//			printf("POST - Begin Transfer \r\n");
//      }
		else if ( ThisEvent.EventParam == 'q'){
			ES_Event newEvent = {WindByteReceived, 0x24};
			PostWind( newEvent );
			printf("POST - Start byte \r\n");
        }
		else if ( ThisEvent.EventParam == 'w'){
			ES_Event newEvent = {WindByteReceived, '5'};
			PostWind( newEvent );
			printf("POST - Byte \r\n");
        }
		else if ( ThisEvent.EventParam == 'e'){
			ES_Event newEvent = {WindByteReceived, 0x35};
			PostWind( newEvent );
			printf("POST - checksum \r\n");
        }
		else if ( ThisEvent.EventParam == 'a'){
			ES_Event newEvent = {CableTrigger,0};
			PostGridlinq( newEvent );
			printf("POST - CableTrigger \r\n");
        }
		else if ( ThisEvent.EventParam == 's'){
			ES_Event newEvent = {GoToSleep,0};
			PostGridlinq( newEvent );
			printf("POST - GoToSleep \r\n");
        }
		
		return true;
  }
  return false;
}
