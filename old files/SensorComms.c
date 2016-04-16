/****************************************************************************
 Module
   SensorComms.c

Revision      Revised by: 
  0.1.1       Alex
  0.1.2       Alex
  1.0.0       Alex
  2.0.0       Alex

 Description
   This is a file for implementing UART master burst reads

 Notes
 
 Edits:
  0.1.1       Set up template for burst reads
  0.1.2       Changed input variables to yaw rate and heading angle 
  1.0.0       Implement 10 Hz timer to continously grab sensor data
  2.0.0       Convert to UART interrupts (no need for timers, just request data when you need it)
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
// Standard headers
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <cmath>

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
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "uartstdio.h"
#include "inc/hw_uart.h"
#include "inc/hw_nvic.h"

// Module header
#include "SensorComms.h"

/*----------------------------- Module Defines ----------------------------*/
#define SLAVE_ADDRESS         0x61
#define BitsPerNibble         4
#define YAW_RATE_CONSTANT     0xEF
#define HEADING_CONSTANT      0xDD

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitUART2(void);
void UART2ISR(void);
void calc(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// Temporary variables for recieve data conversion
uint16_t a_int;
double a = 0;

// Variables being recieved
uint8_t yawRateLow = 0;
uint8_t yawRateHigh = 0;
double yawRate = 0;
double headingAngle = 0;

// Variables to help with recieve logistics
uint8_t Packet;
uint8_t FakeState = 0;
uint8_t dataCounter = 0;
bool error = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSensorComms

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

****************************************************************************/
bool InitSensorComms ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
  
  // Initialize UART2 on TIVA
  InitUART2( );
  
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
     PostSensorComms

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

****************************************************************************/
bool PostSensorComms( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSensorComms

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Implements SensorComms state machine
 Notes
   
****************************************************************************/
ES_Event RunSensorComms( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  // Begin transfer with Bosch sensor
  if(ThisEvent.EventType == BeginTransfer) {
    // Put slave address on UART data line
    HWREG(UART2_BASE + UART_O_DR) = SLAVE_ADDRESS;
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
      PostSensorComms(newEvent);
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
  return yawRate;
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
  return headingAngle;
}

/***************************************************************************
 private functions
 ***************************************************************************/

// Initialize UART2 on TIVA
void InitUART2(void) {
/**************** Initialize UART to communicate with PICs *****************/
  /*
  Steps to initialize Tiva UART
  1.  Enable the clock to the UART module using RCGCUART register
  2.  Enable the clock to the appropriate GPIO module RCGCGPIO
  3.  Wait for the UART to be ready PRUART
  4.  Wait for the GPIO module to be ready
  5.  Configure GPIO pins for in/out/drive-level/drivetype
  6.  Select the alternate function for the UART pins (AFSEL)
  7.  Configure the PMCn fields in the GPIOPCTL register to assign the UART pins
  8.  Disable the UART by clearing the UARTEN bit in the UARTCTL register
  9.  Write the integer portion of the BRD to the UARTIBRD register
  10. Write the fractional portion of the BRD to the UARTBRD register
  11. Write the desired serial parameters to the UARTLCRH register
  12. Configure the UART operation using the UARTCTL register
  13. Locally enable interrupts for the UART receive interruypt mask (RXIM in UARTIM)
  13. Enable UART2 interrupt vector in NVIC (Bit 6 in NVIC_EN0) (Tiva p.104 for vector table)
  13. Enable the UART by setting the UARTEN bit in the UARTCTL register
  
  UART Module 1:
  U2RX - Receive PD6
  U2TX - Transmit PD7
  */
  
  // Enable the clock to the UART module 2
  HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R2;
  
  // Enable clock to the GPIO port D
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
  
  
  // Wait for GPIO port D to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3 ) != SYSCTL_PRGPIO_R3 )
    ;
  
  // Configure GPIO pins for in/out/drive-level/drivetype
  // Set GPIO PD6 and PD7 as digital
  HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
  
  // Set GPIO PD6 as input (receive) and PD7 as output (transfer)
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= BIT7HI;
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= ~BIT6HI;
  
  // Program pins D6 and D7 as alternate functions on the GPIO to use UART (Set High) 
  HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (BIT6HI | BIT7HI);
  
  // Set mux position on GPIOPCTL to select UART alternate function on D6, D7 (Tiva p.1351)
  // Mux value = 1 (p.895) offset mask to clear nibbles 6 and 7
  HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) =                                                                      
    (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & ~0xff000000) + (1 << ( 6 * BitsPerNibble)) +
      (1 << ( 7 * BitsPerNibble));
  
  // Disable UART module 2
  HWREG(UART2_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;
  
  // Baud Rate = 9600, Integer = 260, Fraction = 27
  // Write the integer portion of the BRD (260) - NEW VALUES 5220,53
  HWREG(UART2_BASE + UART_O_IBRD) = 0x208;
  
  // Write the fractional portion of the BRD (27)
  HWREG(UART2_BASE + UART_O_FBRD) = 0x35;
  
  // Write desired serial parameters to the UART line control
  HWREG(UART2_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8; // Set 8-bit frame length and clear everything else 

  // Configure UART operation
  HWREG(UART2_BASE + UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE); // Enable Receive and Transmit
  
  // Locally enable interupts for UART receive interrupt mask (RXIM)
  HWREG(UART2_BASE + UART_O_IM) |= UART_IM_RXIM;

  // Set NVIC enable for UART2 (see Tiva p.104)
  HWREG(NVIC_EN1) |= BIT1HI;
  
  // Make sure interrupts are enabled globally
  __enable_irq( );

  // Enable UART module 2
  HWREG(UART2_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

  // Print to console if successful initialization
  printf("UART Initialized\n\r");
  
  // Clear initial interrupt flag
  HWREG(UART2_BASE + UART_O_ICR) |= UART_ICR_RXIC;
}

// Calculate transmitted yaw rate from two 8 bit integers to one double
void calc(void) {
    // Put together two 8 bit integers into a 16 bit integer
    a_int = yawRateHigh; 
    a_int = (a_int << 8) + yawRateLow;
  
    // Put the correct sign on value and convert to double
    if(a_int%2) {
      a = a_int - 1;
      a = a*(-1);
    }
    else {
      a = a_int;
    }
    
    // Divide by 1000 to obtain the measured value of yaw rate
    a = a/1000;
    yawRate = a;
}

// Interrupt Response Routine handles recieve of yaw and heading data
void UART2ISR(void) {
  // Clear the source of the interrupt
  HWREG(UART2_BASE + UART_O_ICR) |= UART_ICR_RXIC;
  
  // Save the packet (lowest 8 bits of data register)
  Packet = HWREG(UART2_BASE + UART_O_DR);
  
  // If we haven't seen an indicator constant, look for an indicator constant
  if(FakeState == 0) {
    // If we see the yaw rate indicator constant, look for a yaw rate (high and low byte)
    if(Packet == YAW_RATE_CONSTANT) {
      FakeState = 1;
    }
    // If we see the heading indicator constant, look for a heading angle
    if(Packet == HEADING_CONSTANT) {
      FakeState = 2;
    }
  }
  // If we are looking for a yaw rate, set the first byte as the low byte of yawRate and second byte as the high byte of yawRate
  else if(FakeState == 1) {
    if(dataCounter == 0) {
      yawRateLow = Packet;
      dataCounter++;
    }
    else {
      yawRateHigh = Packet;
      // Call calculate function when we get both bytes
      calc();
      // Reset state to look for indicator constant
      dataCounter = 0;
      FakeState = 0;
    }
  }
  // If we are looking for a heading angle, set heading angle equal to the incoming packet
  else if(FakeState == 2) {
    headingAngle = Packet;
    // Reset state to look for indicator constant
    FakeState = 0;
  }
}

