/****************************************************************************
 Module
  Clamp.c

 Description
  State Machine for running the wire clamp
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
#include "Clamp.h"
#include "Gridlinq.h"
#include "PWM.h"

// TivaWare Headers
#include "driverlib/sysctl.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define ONE_SEC             976


#define DEBOUNCE						1000

#define MOTOR_SPEED					50
#define HALF								2400
#define MOTOR_TIME					2800

/*---------------------------- Module Functions ---------------------------*/
// Prototypes for private functions for this service.

// Interrupt functions
void EncoderInterrputResponse(void);
void ClampInterruptResponse(void);

// Initialization functions
static void InitInputCaptureWT1(void);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static ClampState CurrentState;
uint8_t BeaconValue;
static uint16_t ticks = 0;
bool reverse = true;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitClamp

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the UART and Pins necessary for communication
****************************************************************************/
bool InitClamp ( uint8_t Priority ) {
	ES_Event ThisEvent;
  MyPriority = Priority;
  CurrentState = Open;
	InitPWM();
	InitInputCaptureWT1();
	
	//turn on electromagnets
	HWREG(GPIO_PORTF_BASE + ALL_BITS) |= (BIT0HI);
	
	//begin transmitting the IR
	SetPWMDuty(50,IR_BEACON);
    SetPWMDuty(50,BEACON_HACK);
	
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
     PostClamp

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostClamp( ES_Event ThisEvent ) {
  return ES_PostToService( MyPriority, ThisEvent);
}
/****************************************************************************
 Function
    RunClamp

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for the Spectre, moves between unpaired and paired. 
    Handles the analysis of the incoming data as described by the 218 
    comm protocol
****************************************************************************/
ES_Event RunClamp ( ES_Event CurrentEvent )
{
  ES_Event ReturnEvent = {ES_NO_EVENT};
  ClampState NextState = CurrentState;
  
	switch (CurrentState) {
		case Open :
			if(ticks%1000 == 1) printf("ticks: %d\r\n",ticks);
			if(CurrentEvent.EventType == CableTrigger) {
				ES_Timer_InitTimer(DebounceTimer,DEBOUNCE);
				NextState = Debouncing;
				//printf("debouncing\r\n");
			}
			
			else if(CurrentEvent.EventType == Encoder) {
				if(ticks > HALF) {
					HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
					SetPWMDuty(0,CLAMP_MOTOR);
					printf("opened\r\n");
					ES_Timer_StopTimer(MotorTimer);
				}
			}
			
			else if (CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == MotorTimer) {
				HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
					SetPWMDuty(0,CLAMP_MOTOR);
					printf("opened - timer\r\n");
			}
			break;
		
		case Debouncing :
			if(CurrentEvent.EventType == CableTrigger) {
				NextState = Open;
				//cable not detected
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == DebounceTimer) {
				NextState = Closed;
				//start the motors
				if(reverse) HWREG(GPIO_PORTD_BASE + ALL_BITS) |= BIT2HI;
				else HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~ BIT2HI;
				reverse = !reverse;
				SetPWMDuty(MOTOR_SPEED,CLAMP_MOTOR);
				ticks = 0;
				ES_Timer_InitTimer(MotorTimer,MOTOR_TIME);
				printf("closing the clamp\r\n");
				//shut off electromagnets
				HWREG(GPIO_PORTF_BASE + ALL_BITS) &= ~(BIT0HI);
				//turn off the beacon
				SetPWMDuty(0,IR_BEACON);
			}
			break;
			
		case Closed :
			if(CurrentEvent.EventType == Encoder) {
				if(ticks > HALF) {
					HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
					SetPWMDuty(0,CLAMP_MOTOR);
					ES_Timer_StopTimer(MotorTimer);
					ticks = 0;
					//shut off electromagnets
					HWREG(GPIO_PORTF_BASE + ALL_BITS) &= ~(BIT0HI);
				}
			}
			else if (CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == MotorTimer) {
				HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
				SetPWMDuty(0,CLAMP_MOTOR);
				printf("closed - timer\r\n");
				//shut off electromagnets
				HWREG(GPIO_PORTF_BASE + ALL_BITS) &= ~(BIT0HI);
			}
			
			else if (CurrentEvent.EventType == Disengage) {
				NextState = Disengaging;
				//open the clamp
				if(reverse) HWREG(GPIO_PORTD_BASE + ALL_BITS) |= BIT2HI;
				else HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~ BIT2HI;
				reverse = !reverse;
				SetPWMDuty(MOTOR_SPEED,CLAMP_MOTOR);
				ticks = 0;
				printf("disengaging\r\n");
				ES_Timer_InitTimer(MotorTimer,MOTOR_TIME);
				SetPWMDuty(50,IR_BEACON);
			}
			break;
			
		case Disengaging :
			if(CurrentEvent.EventType == Encoder) {
				//printf("encoder\r\n");
				if(ticks > HALF) {
					HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
					SetPWMDuty(0,CLAMP_MOTOR);
					printf("opened\r\n");
					ES_Timer_StopTimer(MotorTimer);
					//turn on electromagnets
					HWREG(GPIO_PORTF_BASE + ALL_BITS) |= (BIT0HI);
				}
			}
			else if (CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == MotorTimer) {
				HWREG(GPIO_PORTD_BASE + ALL_BITS) &= ~BIT2HI;
				SetPWMDuty(0,CLAMP_MOTOR);
				printf("opened - timer\r\n");
				//turn on electromagnets
				HWREG(GPIO_PORTF_BASE + ALL_BITS) |= (BIT0HI);
			}	
			else if(CurrentEvent.EventType == CableTrigger) {
				NextState = Open;
				printf("opening\r\n");
			}
			else if(CurrentEvent.EventType == Engage) {
				NextState = Debouncing;
				printf("re-engaging\r\n");
				ES_Event newEvent = {ES_TIMEOUT, DebounceTimer};
				PostClamp(newEvent);
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
    ClampInterruptResponse

 Description
    Clears the interrupt response and posts an event to the clamp service.
****************************************************************************/
void ClampInterruptResponse(void) {
    // Clear the interrupt source
	HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CBECINT;
    // Read the pin value
	uint8_t pinVal = HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + ALL_BITS)) & GPIO_PIN_7;
	//printf("triggered\r\n");
    // Post the cable event
	ES_Event CableEvent;
	if(pinVal == BeaconValue) {
        CableEvent.EventType = FalseAlarm;
    }
	else {
        CableEvent.EventType = CableTrigger;
    }
	PostClamp(CableEvent);
}


/****************************************************************************
 Function
    EncoderInterrputResponse

 Description
    Clears the interrupt response and posts an event to the clamp service.
****************************************************************************/
void EncoderInterrputResponse(void) {
	HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	ES_Event CableEvent = {Encoder, 0};
	PostClamp(CableEvent);
	ticks++;
}

/****************************************************************************
 Function
   InitInputCaptureWT1

 Parameters
   none

 Returns
   none

 Description
	 Initialize Input Capture on C6 and C7 (Wide timer 1 A&B)
	 C7 will be a rise and fall interrupt
	 C6 will be the Encoder interrupt
****************************************************************************/
static void InitInputCaptureWT1( void ) {
    /*
	Steps to initialize Tiva Wide Timer Input Capture
	1.	Enable the clock to wide timer 1 using RCGCWTIMER_R1 register
	2.	Enable the clock to the appropriate GPIO module RCGCGPIO
	3.	Disable wtimer 1 timer A to configure with TAEN in TIMER_O_CTL register
	4.	Set timer to 32bit wide mode with TIMER_CFG_16_BIT in TIMER_O_CTL register
	5.	Set timer to use full 32bit count by setting TIMER_O_TAILR to 0xffffffff
	6.	Set timer A to capture mode for edge time and up-counting (Clear TAMMS and Set TACMR,TACDIR,TAMR_CAP in TIMERTAMR)
	7.	Set event to rising edge by clearing TAEVENT_M in TIMER_O_CTL (Rising edge = 00)
	8.	Select the alternate function for the Timer pins (AFSEL)
	9.	Configure the PMCn fields in the GPIOPCTL register to assign the WTIMER pins (WT1CCP0)
	10.	Enable appropriate pint on GPIO port for digital I/O (GPIODIR)
	11.	Set appropriate pins on GPIO as inputs (GPIODEN)
	12. Locally enable interrupts for the capture interrupt mask (CAEIM in TIMERIMR)
	13. Enable WTIMER1 Timer A interrupt vector in NVIC (96 = Bit 0 in NVIC_EN3) (Tiva p.104 for vector table)
	14. Ensure interrupts are enabled globally (__enable_irq())
	14.	Enable WTIMER1 Timer A with TAEN in TIMER_O_CTL register
	
	WTIMER1 Timer A - Tiva PC6
	*/
		
		
    // Enable the clock to the timer (Wide Timer 1)
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
		
	// Enable the clock to Port C
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  
    // Disable timer A & B before configuring
    HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	
    // Set up in 32bit wide (individual, not concatenated) mode
    HWREG(WTIMER1_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Use the full 32 bit count, so initialize the Interval Load
    // register to 0xffff.ffff
    HWREG(WTIMER1_BASE + TIMER_O_TAILR) = 0xffffffff;
	HWREG(WTIMER1_BASE + TIMER_O_TBILR) = 0xffffffff;
	
    // set up timer A in capture mode (TAAMS = 0), 
    // for edge time (TACMR = 1) and up-counting (TACDIR = 1)
    HWREG(WTIMER1_BASE + TIMER_O_TAMR) = 
        (HWREG(WTIMER1_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | 
        (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);

	HWREG(WTIMER1_BASE + TIMER_O_TBMR) = 
        (HWREG(WTIMER1_BASE + TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) | 
        (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);
				
    // To set the event to rising edge, we need to modify the TAEVENT bits 
    // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
    HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TAEVENT_BOTH);

    // Set up the alternate function for Port C bit 6 (WT1CCP0)
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT6HI | BIT7HI);

	// Set mux position on GPIOPCTL to select WT1CCP0 alternate function on C6
	// Mux value = 7 offset mask to clear nibble 6
	HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~0x0f000000) + (7 << ( 6 * BitsPerNibble)) + (7 << ( 7 * BitsPerNibble));
			
    // Enable pin 6 & 7 on Port C for digital I/O
    HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
	
    // Make pin 6 & 7 on Port C into an input
    HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= (BIT6LO & BIT7LO);

    // Enable a local capture interrupt
    HWREG(WTIMER1_BASE + TIMER_O_IMR) |= (TIMER_IMR_CAEIM | TIMER_IMR_CBEIM);

    // Enable Timer A Wide Timer 1 interrupt in the NVIC
    // NVIC number 96 (EN3 bit 0) Tiva p.104
    HWREG(NVIC_EN3) |= (BIT0HI | BIT1HI);

    // Make sure interrupts are enabled globally
    __enable_irq();

    // Enable Timer
    HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL | TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
    
	// Print to console if successful initialization
	printf("Wide Timer 1 A&B interrupt initialized\n\r");
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

