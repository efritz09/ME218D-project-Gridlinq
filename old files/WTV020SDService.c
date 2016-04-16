/****************************************************************************
 Module
  WTV020SDService.c

Revision           Revised by:         Date:
  1.0.0            Denny               05/18/2015

 Description
  This is the interface between the Tiva and WTV020SD audio chip to play sound
  from our hovercraft. The header file defines the symbolic names for the audio
  files stored on the WTV chip, all that is needed is to call PlaySound(sound).

 Edits:
  1.0.0 - Working non-blocking library to interface with Sparkfun WTV020SD chip

 TODO:
  -Integrate into main code and debug any problems
  -Add reset functionality and busy flag monitoring
  -Add lastCommand variable to track what sounds we can interrupt (like idle theme)
  -Add functionality to play Arther theme song on idle. 
  -Get a shitload of archer quotes

****************************************************************************/
// If we are testing
//#define TEST

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
// Standard headers
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// TIVA headers
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "bitdefs.h"

// Framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_PostList.h"
#include "ES_ServiceHeaders.h"
#include "ES_Port.h"

// Module headers
#include "WTV020SDService.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS                (0xff<<2)
#define BitsPerNibble           4
#define TicksPerMS              40000
#define TicksPerMicroS          40
// WTV pins
#define CLOCK_PIN               BIT0HI
#define DATA_PIN                BIT1HI
#define RESET_PIN               BIT2HI

/*---------------------------- Module Functions ---------------------------*/
// prototypes for private functions for this service.
static void InitWTVPins ( void );
static void InitWideTimer0 ( void );
static void SetTimerMicroS( uint16_t timeInMicroS);
static ES_Event DuringReady( ES_Event );
static ES_Event DuringResetting( ES_Event );
static ES_Event DuringTransferring ( ES_Event );

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static WTVState_t CurrentWTVState;
static ResettingState_t CurrentResettingState;
static TransferringState_t CurrentTransferringState;
static uint16_t DataMask;
static uint8_t NextCommand;
static bool CommandWaiting = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
Function
  InitWTV
Parameters
  uint8_t : the priorty of this service
Returns
  bool, false if error in initialization, true otherwise
Description
  Initializes TIVA pins to communicate with the WTV chip
    Pin E0: Clock
    Pin E1: Data
    Pin E2: Reset
****************************************************************************/
bool InitWTV ( uint8_t Priority )
{
    // Set mypriotity and initial WTV state
    MyPriority = Priority;
    CurrentWTVState = WTVResetting;
    // Set up GPIO pins for WTV
    InitWTVPins();
    // Set up Wide Timer 0 Timer A for microsecond timing
    InitWideTimer0();
    printf("WTV: Initialized\n\r");
    // Post the initial transition event
    ES_Event ThisEvent = {ES_ENTRY};
    if (ES_PostToService( MyPriority, ThisEvent)) return true;
    else return false;
}


/****************************************************************************
Function
  PostWTV
Parameters
  ES_Event ThisEvent ,the event to post to the queue
Returns
  bool false if the Enqueue operation failed, true otherwise
Description
  Posts an event to this state machine's queue
****************************************************************************/
bool PostWTV( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}


/****************************************************************************
 Function
  RunWTV

 Parameters
  ES_Event ThisEvent

 Returns
  bool false if the Enqueue operation failed, true otherwise

 Description
  Posts an event to this state machine's queue
****************************************************************************/
ES_Event RunWTV( ES_Event CurrentEvent )
{
    bool MakeTransition = false;    // Assume no state transition initially
    WTVState_t NextWTVState = CurrentWTVState;
    ES_Event ReturnEvent = {ES_NO_EVENT};
    
    switch( CurrentWTVState ) {
        case WTVReady :
            // Execute during function for Ready
            CurrentEvent = DuringReady( CurrentEvent );
            // Check if event is active
            if ( CurrentEvent.EventType != ES_NO_EVENT ) {
                switch( CurrentEvent.EventType ) {
                    case WTVPlaySound :
                        // Process the play sound event
                        NextCommand = CurrentEvent.EventParam;
                        NextWTVState = WTVTransferring;
                        MakeTransition = true;
                        //CommandWaiting = true;
                        //printf("WTV Ready - Transfer command: %d\r\n", NextCommand);
                        break; 
                    default :
                        break;
                }
            }
            break;
            
        case WTVResetting :
            // Execute during function for Resetting
            CurrentEvent = DuringResetting( CurrentEvent );
            // Check if event is active
            if ( CurrentEvent.EventType != ES_NO_EVENT ) {
                switch( CurrentEvent.EventType ) {
                    case WTVPlaySound :
                        // Save command to play once reset is done
                        NextCommand = CurrentEvent.EventParam;
                        CommandWaiting = true;
                        break;
                    case WTVResetComplete :
                        // Set to ready state, bypass to transfer if command waiting
                        if( CommandWaiting) NextWTVState = WTVTransferring;
                        else NextWTVState = WTVReady;
                        MakeTransition = true;
                        //printf("WTV Reset complete\r\n");
                        break;
                    default :
                        break;
                }
            }
            break;
            
        case WTVTransferring :
            // Execute during function for Transferring
            CurrentEvent = DuringTransferring( CurrentEvent );
            // Check if event is active
            if ( CurrentEvent.EventType != ES_NO_EVENT ) {
                switch( CurrentEvent.EventType ) {
                    case WTVPlaySound :
                        // Save command to play once transfer is done
                        NextCommand = CurrentEvent.EventParam;
                        CommandWaiting = true;
                        break;
                    case WTVTransferComplete :
                        // Set to ready state
                        NextWTVState = WTVReady;
                        MakeTransition = true;
                        //printf("WTV Transfer complete\r\n");
                        break;
                    default :
                        break;
                }
            }
            break;
    }

    // If we are making a state transition
    if( MakeTransition == true ){
        // Execute exit function for the current state
        CurrentEvent.EventType = ES_EXIT;
        RunWTV(CurrentEvent);
        // Modify current state
        CurrentWTVState = NextWTVState;
        // Execute entry function for new state
        CurrentEvent.EventType = ES_ENTRY;
        RunWTV(CurrentEvent);
    }
  return ReturnEvent;
}


/****************************************************************************
Function
  PlaySound
Parameters
  uint16_t sound
Returns
  void
Description
  Creates a PlaySound event 
****************************************************************************/
void PlaySound (uint16_t sound )
{
    ES_Event NewEvent = {WTVPlaySound, sound};
    PostWTV(NewEvent);
}


/****************************************************************************
Function
  WTVInterruptResponse
Parameters
  none
Returns
  void
Description
  Posts ES_TimeoutEvent
****************************************************************************/
void WTVInterruptResponse( void )
{
    // Clear the interrupt source
    HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_TATOCINT;
    // Post a timeout event to the service
    ES_Event NewEvent = {ES_TIMEOUT};
    PostWTV(NewEvent);
}


/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
Function
  InitWTVPins
Parameters
  none
Returns
  void
Description
  Initializes the following pins:
    E0: Serial Clock    (Digital Output)
    E1: Data Line       (Digital Output)
    E2: Reset Line      (Digital Output)
****************************************************************************/
static void InitWTVPins ( void )
{
    //Initialize GPIO pins to communicate with WTV
    // Initialize the clock to GPIO Port E
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
    // Wait for GPIO Port E to be ready
    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4 ) != SYSCTL_PRGPIO_R4 )
        ;
    // Enable E0-E2 as digital pins
    HWREG(GPIO_PORTE_BASE + GPIO_O_DEN) |= (CLOCK_PIN | DATA_PIN| RESET_PIN );
    // Set E0-E2 as outputs
    HWREG(GPIO_PORTE_BASE + GPIO_O_DIR) |= (CLOCK_PIN | DATA_PIN | RESET_PIN);
    // Set clock and reset to idle high
    HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  |= (CLOCK_PIN | RESET_PIN);
    // Set data low
    HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  &= ~DATA_PIN;
}


/****************************************************************************
Function
  InitWideTimer0
Parameters
  none
Returns
  void
Description
  Initializes WideTimer0, Timer A as one shot timer
****************************************************************************/
static void InitWideTimer0 ( void )
{
    // Start by enabling the clock to the timer (Wide Timer 0)
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
    // Wait for Wide Timer0 to be ready
    while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R0 ) != SYSCTL_PRWTIMER_R0 )
        ;
    // Make sure that Wide Timer 0 Timer A is disabled before configuring
    HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    // Set Wide Timer 0 to 32bit wide mode
    HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_32_BIT_TIMER;
    // Clear the bits of Timer A mode and then set Timer A to One-Shot mode (0x1)
    HWREG(WTIMER0_BASE+TIMER_O_TAMR) &= ~TIMER_TAMR_TAMR_M;
    HWREG(WTIMER0_BASE+TIMER_O_TAMR) |= TIMER_TAMR_TAMR_1_SHOT;
    // Set the timer to down-counting
    HWREG(WTIMER0_BASE+TIMER_O_TAMR) &= ~TIMER_TAMR_TACDIR;
    // Set Wide Timer 0 Timer A timeout.. don't need this to start
    //HWREG(WTIMER0_BASE+TIMER_O_TAILR) = RESET_TIME;
    // Enable a local timeout interrupt for Timer A
    HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_TATOIM;
    // Enable Wide Timer 0 Timer A interrupt in the NVC
    // Wide Timer 0 is interrupt number 96 in in NVIC, appears in EN2 at Bit30
    // (find these values in table 2-9 on page 105 of Tiva datasheet and hw_nvic.h)
    HWREG(NVIC_EN2) |= BIT30HI;
    // make sure interrupts are enabled globally
    __enable_irq();
    // Now set the timer to stall while stopped by the debugger
    HWREG(WTIMER0_BASE+TIMER_O_CTL) |= TIMER_CTL_TASTALL;
}


/****************************************************************************
Function
  SetTimerMicroS
Parameters
  uint16_t timeInMicroS
Returns
  void
Description
  Sets a one-shot timer for number of micro seconds passed to it
****************************************************************************/
static void SetTimerMicroS( uint16_t timeInMicroS)
{
    // Convert time to ticks
    uint32_t ticks = TicksPerMicroS * timeInMicroS;
    // Make sure that Wide Timer 0 Timer A is disabled before configuring
    HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    // Set Wide Timer 0 Timer A timeout to parameter Ticks
    HWREG(WTIMER0_BASE+TIMER_O_TAILR) = ticks;
    // Enable Wide Timer 0 Timer A
    HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN);
}


/****************************************************************************
Function
  DuringReady
Parameters
  ES_Event
Returns
  ES_Event
Description
  Runs the during process for the Ready State
****************************************************************************/
static ES_Event DuringReady ( ES_Event Event)
{
    // Process ES_ENTRY and ES_EXIT events
    if ( Event.EventType == ES_ENTRY ) {
        //printf("Enter Ready:\r\n");
        // Check if we have a command waiting... if we do then send it
        if(CommandWaiting == true) {
            //printf("\tReady: CommandWaiting\r\n");
            // Modify event to be processed in the main WTV state machine
            // Clear the CommandWaiting flag
            Event.EventType = WTVPlaySound;
            Event.EventParam = NextCommand;
            CommandWaiting = false;
        }
    }
    else if ( Event.EventType == ES_EXIT ) {
        //printf("Exit Ready:\r\n");
        // Don't think we need to do anything here
    }
    else {
        // Do the 'during' function for this state
        // The ReadyState Machine.. which is nothing.. so do nothing
    }
    return Event;
}


/****************************************************************************
Function
  DuringResetting
Parameters
  ES_Event
Returns
  ES_Event
Description
  Runs the during process for the Resetting State
****************************************************************************/
static ES_Event DuringResetting ( ES_Event Event)
{
    // Process ES_ENTRY and ES_EXIT events
    if ( Event.EventType == ES_ENTRY ) {
        //printf("Enter Resetting:\r\n");
        // Set CurrentResetState to ResetPulse
        CurrentResettingState = ResetPulse;
        // Set clock line and reset line low
        HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  &= ~(CLOCK_PIN | RESET_PIN);
        // Start WTV timer for RESET_PULSE (5ms)
        ES_Timer_InitTimer(WTV_TIMER, 6);
    }
    else if ( Event.EventType == ES_EXIT ) {
        // Don't think we need to do anything here
        //printf("Exit Resetting:\r\n");
        //PlaySound(VOLUME_MAX);
    }
    else if ( Event.EventType == ES_TIMEOUT ) {
        // Do the 'during' function for this state
        // ResettingState Machine...
        switch( CurrentResettingState ) {
            case ResetPulse :
                //printf("\tReset: Pulse complete\r\n");
                // Set clock line and reset line high (PD0,PD2)
                HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) |= (RESET_PIN | CLOCK_PIN);
                // Change state to ResetDelay, start ResetDelay Timer
                CurrentResettingState = ResetDelay;
                ES_Timer_InitTimer(WTV_TIMER, 300);
                break;
            case ResetDelay :
                // Modify event to Reset Complete for WTV state to process
                Event.EventType = WTVResetComplete;
                //printf("\tReset: Delay complete\r\n");
                break;
        }
    }
    return Event;
}


/****************************************************************************
Function
  DuringTransferring
Parameters
  ES_Event
Returns
  ES_Event
Description
  Runs the during process for the Transferring State
****************************************************************************/
static ES_Event DuringTransferring( ES_Event Event) 
{
    // Process ES_ENTRY and ES_EXIT events
    if ( Event.EventType == ES_ENTRY ) {
        //printf("Enter Transferring:\r\n");
        // Initialize data mask (MSB sent first)
        DataMask = 0x8000;
        // Clear commandWaiting flag
        CommandWaiting = false;
        // Set CurrentTransferringState to ClockHigh
        CurrentTransferringState = ClockHigh;
        // Write clock line low
        HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  &= ~CLOCK_PIN;
        // 2ms delay before clock/data starts
        ES_Timer_InitTimer(WTV_TIMER, 2);
    }
    else if ( Event.EventType == ES_EXIT ) {
        // Don't think we need to do anything here
        //printf("Exit Transferring:\r\n");
    }
    else {
        // Do the 'during' function for this state
        // TransferringState Machine.. 
        if( Event.EventType == ES_TIMEOUT ) {
            switch( CurrentTransferringState ) {
                case ClockHigh :
                    // Update CurrentTransferingState
                    CurrentTransferringState = SendData;
                    // Set clock line low
                    HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  &= ~CLOCK_PIN;
                    // Dalay before sending data
                    SetTimerMicroS(50);
                    break;
                case SendData :
                    // Update CurrentTransferingState
                    CurrentTransferringState = ClockLow;
                    // Set data pin from NextCommand
                    if(NextCommand & DataMask)
                        HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  |= DATA_PIN;
                    else HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  &= ~DATA_PIN;
                    // Start timer for remaining clocl low time (150 micro seconds)
                    SetTimerMicroS(150);
                    break;   
                case ClockLow :
                    // Update CurrentTransferingState
                    CurrentTransferringState = ClockHigh;
                    // Set clock line high
                    HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS))  |= CLOCK_PIN;
                    // Shift the DataMask
                    DataMask >>= 1;
                    // Check if we are at the end of the data
                    if(DataMask > 0) {
                        // Start 2ms timer for low clock state
                        ES_Timer_InitTimer(WTV_TIMER, 2);
                    }
                    else {
                        // Done: Modify event for WTV state machine
                        Event.EventType = WTVTransferComplete;
                    }
                    break;
            }
        }
    }
    return Event;
}


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
