/****************************************************************************
 Module
  Balloon.c

 Revision     Revised by: 
  0.1.1       Eric

 Description
  State machine for balloon popping

 Edits:
  0.1.1 - Preliminary setup

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
// Standard headers
#include <stdint.h>
#include <stdbool.h>

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
#include "bitdefs.h"

// Module headers
#include "Balloon.h"
#include "SpectreXbee.h"
#include "PWM.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble         4
#define TicksPerMS            40000       // 40Mhz
#define ALL_BITS              (0xff<<2)

#define WARM_UP               1
#define ACTUATE               2000

#define POPPER_PWM_CHL        6
#define POPPER_IN_PWM_WIDTH   1000
#define POPPER_OUT_PWM_WIDTH  500
/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
static void InitGPIO( void );


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static PopState CurrentState; 


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
  InitBalloon

 Parameters
  uint8_t : the priorty of this service

 Returns
  bool, false if error in initialization, true otherwise

 Description
  B0 = Lift fan
  B4 = Prop1
  B5 = Prop2
****************************************************************************/
bool InitBalloon  ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;

  // Initialize GPIO output for balloon popping
  InitGPIO();           

  // Post the initial transition event
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
  PostBalloon

 Parameters
    ES_Event ThisEvent ,the event to post to the queue

 Returns
    bool false if the Enqueue operation failed, true otherwise

 Description
    Posts an event to this state machine's queue
****************************************************************************/
bool PostBalloon( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
  RunBalloon 

 Parameters
  ES_Event : the event to process

 Returns
  ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
  
****************************************************************************/
ES_Event RunBalloon ( ES_Event CurrentEvent )
{
  ES_Event ReturnEvent = {ES_NO_EVENT}; 
  PopState NextState = CurrentState;
  switch(CurrentState) {
    case IdleState:
      if(CurrentEvent.EventType == Pop) {
        NextState = HeatingWireState;
        ES_Timer_InitTimer(PopTimer,WARM_UP);
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) |= BIT1HI; //start warming the wire
        printf("\r\nPop: Start heating wire");
      }
      break;
    case HeatingWireState:
      if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == PopTimer) {
        // Actuate the mechanism
        printf("\r\nPop: Move out grill");
        SetPWMWidth(POPPER_OUT_PWM_WIDTH, POPPER_PWM_CHL);
        ES_Timer_InitTimer(PopTimer,ACTUATE);
        NextState = ActuateState;
      }
      else if(CurrentEvent.EventType == ShutDown) {
        // Shut it all down
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT1LO;  
        NextState = IdleState;
      }
      break;
    case ActuateState:
      if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == PopTimer) {
        // Shut it all down
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT1LO; 
        printf("\r\nPop: Move in grill");
        SetPWMWidth(POPPER_IN_PWM_WIDTH, POPPER_PWM_CHL);
        // Deactuate mechanism
        NextState = IdleState;
      }
      else if(CurrentEvent.EventType == ShutDown) {
        // Shut it all down
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT1LO; 
        // Deactuate mechanism
        NextState = IdleState;
      }
      break;
  }
  CurrentState = NextState;
  return ReturnEvent;
}


/****************************************************************************
 Function
   InitGPIO

 Description
   Initialize GPIO pins for inputs/outputs

    Inputs:           Outputs:
                        F3
****************************************************************************/
static void InitGPIO( void )
{
  // Enable Port F to the timer
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
  
  // Wait for GPIO Port F to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1 ) != SYSCTL_PRGPIO_R1 )
    ;
  
  // Set Pin F3 as digital output
  HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (BIT1HI);
  HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= (BIT1HI);
  // Print to console if successful initialization
  printf("GPIO Initialized\n\r");
  
  // Init balloon popper to off
  SetPWMWidth(POPPER_IN_PWM_WIDTH, POPPER_PWM_CHL);
  HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT1LO; 
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
