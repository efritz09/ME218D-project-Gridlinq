/****************************************************************************
 Module
   I2C.c

Revision			Revised by: 
	0.1.1				Alex
	0.1.2				Alex
	1.0.0				Alex


 Description
   This is a file for implementing I2C master burst reads

 Notes
 
 Edits:
	0.1.1				Set up template for burst reads
	0.1.2				Changed input variables to yaw rate and heading angle 
	1.0.0				Implement 10 Hz timer to continously grab sensor data
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <stdint.h>
#include <stdbool.h>
#include <cmath>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ES_Port.h"


#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "uartstdio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "bitdefs.h"
#include "termio.h"

#include "I2C.h"

/*----------------------------- Module Defines ----------------------------*/
#define NUM_I2C_DATA 2
#define SLAVE_ADDRESS 0x04 //0x3C
#define ONE_SEC 976
#define I2CTime ONE_SEC/200
#define HOW_MANY 4

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitI2C0(void);
void calc(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

uint8_t DataTx[NUM_I2C_DATA];
uint8_t DataRx[HOW_MANY];
uint8_t Index;

uint16_t a_int;
double a = 0;

bool error = false;
uint8_t i = 0;

double HeadingData[HOW_MANY/2];


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitI2C

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

****************************************************************************/
bool InitI2C ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
	
	// Initialize I2C on TIVA
  InitI2C0( );
	
	printf("I2C Initialized \r\n");
	
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
     PostI2C

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

****************************************************************************/
bool PostI2C( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunI2C

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Implements I2C state machine
 Notes
   
****************************************************************************/
ES_Event RunI2C( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	
	if(ThisEvent.EventType == BeginTransfer) {
		printf("Begin Transfer \r\n");
		// Initialize I2C timer
		ES_Timer_InitTimer(I2CTimer, I2CTime);
	}

  // State Machine here
	if(ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == I2CTimer) {
		
		printf("1 \r\n");
		
		while(I2CMasterBusBusy(I2C0_BASE))
		;
		
		error = false;
		i = 0;
		
		// Modifiy the data direction to true, so that seeing the address will
    // indicate that the I2C Master is initiating a read from the slave.
    //
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);

    // Place the data to be sent in the data register
    I2CSlaveDataPut(I2C0_BASE, Index);

    // Tell the master to first byte of data.
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	}
	
	if(ThisEvent.EventType == CalculateValues) {
		// Only calculate if no errors occured
		if(!error) {
			calc();
		}
	}
	
	if(ThisEvent.EventType == PrintValues) {
		// Only print if no errors occured
		if(!error) {
			getYaw();
			getHeading();
		}
		else {
			ES_Event newEvent = {PrintValues,0};
			PostI2C(newEvent);
		}
	}
	
  return ReturnEvent;
}

/****************************************************************************
 Function
   getYaw

 Parameters
   none

 Returns
   Last recorded yaw rate
   
****************************************************************************/
double getYaw(void) {
	printf("Yaw Rate: %f \r\n", HeadingData[0]);
	return HeadingData[0];
}

/****************************************************************************
 Function
   getHeading

 Parameters
   none

 Returns
   Last recorded heading angle
   
****************************************************************************/
double getHeading(void) {
	printf("Heading: %f \r\n", HeadingData[1]);
	return HeadingData[1];
}

/***************************************************************************
 private functions
 ***************************************************************************/

//Initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    // Enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
 
    // Reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
     
    // Enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
     
    // Clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
		
		// Always address arduino
		I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);
		
		// Enable master interrupts
		I2CMasterIntEnable(I2C0_BASE);
		
		// Enable NVIC
		HWREG(NVIC_EN0) |= BIT8HI;
		
		// Enable global interrupts
		__enable_irq( );
}

// Determine transmitted values
void calc(void) {
		// Calculate A
		for(int j = 0;j < HOW_MANY; j = j + 2) {
			a_int = DataRx[j+1]; 
			a_int = (a_int << 8) + DataRx[j];
			if(a_int%2) {
				a = a_int - 1;
				a = a*(-1);
			}
			else {
				a = a_int;
			}
			if(j/2 == 1) {
				a = abs(a)/100;
			}
			else {
				a = a/1000;
			}
			HeadingData[j/2] = a;
			//printf("%f \r\n",a);
		}
}

void I2CMasterRecieveDone(void) {
	// Clear Interrupt
	I2CMasterIntClear(I2C0_BASE);
	//printf("INTERRUPT");
	
	// Check for errors
	if(I2C_MASTER_ERR_NONE != I2CMasterErr(I2C0_BASE)) {
		// Don't update if error occured
		//printf("I2C Error \r\n");
		//printf("%d \r\n",i);
		error = true;
		ES_Timer_InitTimer(I2CTimer, I2CTime);
	}
	// Get next data bit
	if(i < HOW_MANY-1) {
		// Update data
		DataRx[i] = I2CMasterDataGet(I2C0_BASE);
		//printf("Received: %x\r\n", DataRx[i] & 0xff);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	}
	else if(i == HOW_MANY-1) {
		// Update data
		DataRx[i] = I2CMasterDataGet(I2C0_BASE);
		//printf("Received: %x\r\n", DataRx[i] & 0xff);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	}
	else {
		ES_Event newEvent = {CalculateValues,0};
		PostI2C(newEvent);
		// Restart timer
		ES_Timer_InitTimer(I2CTimer, I2CTime);
		printf("All GOOD \r\n");
	}
	printf("%d \r\n",i);
	// Increment i
	i++;
}

