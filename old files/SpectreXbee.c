/****************************************************************************
 Module
  SpectreUART.c

 Revision     Revised by: 
  1.0.0       Vikram
  1.0.1       Eric
  1.1.0       Eric
  1.2.0       Eric

 Description
  UART for TIVA asynchronous communication for Lab 10

 Edits:
  1.0.0       Established connection between Xbees
  1.0.1       Added checkSum function for error checking, added PWM and 
              ADC modules
  1.1.0       SPECTRE Xbee setup
  1.2.0       Added Motor posting functionality
              Fixed "accept every address" bug in broadcast mode
              General Cleanup
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
#include "SpectreXbee.h"
#include "CommRxSM.h"
#include "MotorDriver.h"
#include "Balloon.h"
#include "SuicideDs.h"
#include "PWM.h"
#include "WTV020SDService.h"

// TivaWare Headers
#include "driverlib/sysctl.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define MY_ADDRESS          0x0C        //team 12

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

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/

// Interrupt functions
void UART1InterruptResponse( void );
void InputCaptureResponse( void );
void SoftwareReset( void );

// General communication functions
static void importData(void);
uint8_t * GetCommands(void);
uint8_t checkSum(uint8_t chkArray[], uint8_t size);
static void LoadUpConfirmation(void);
static void LoadUpStatus(uint8_t statusVal);
static void SendXbeeData( void );

// Debug function
void printData(void);

// Initialization functions
static void InitUART( void );
static void InitInputCaptureWT1( void );
static void InitGPIO( void );

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static SpectreState CurrentState;

static uint8_t MasterAddrMSB = 0x21;
static uint8_t MasterAddrLSB = 0x8D;
static uint8_t RejectAddrMSB = 0x00;
static uint8_t RejectAddrLSB = 0x00;
static uint8_t *rx;
static uint8_t rxData[12]; //incoming data array
static uint8_t txData[11]; //array for transmitting
static uint8_t CommandData[6]; //just the commands from CTRL
static int txIndex = 0;
static uint8_t mind = 0x00;
static bool flag = false;

uint8_t unpair;
uint8_t pop;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSpectreXbee

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the UART and Pins necessary for communication
****************************************************************************/
bool InitSpectreXbee ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
  InitUART();               // Initialize UART
  InitInputCaptureWT1();    // Initialize Input Capture
  InitGPIO();               // Initialize Packet Inputs/Outputs
  InitPWM();                // Initialize PWM
  openSuicideDoors();       // Set Suicide Doors to be open
  
  PlaySound(THEME_SONG_ABRIDGED); // Play the theme song on initialization
  CurrentState = UnpairedState;
  mind = (HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + ALL_BITS)) & BIT6HI)>>6 ^ 0x01;
  printf("%x\r\n",mind);
    
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
     PostSpectreXbee

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostSpectreXbee( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}
/****************************************************************************
 Function
    RunSpectreXbee

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for the Spectre, moves between unpaired and paired. 
    Handles the analysis of the incoming data as described by the 218 
    comm protocol
****************************************************************************/
ES_Event RunSpectreXbee ( ES_Event CurrentEvent )
{
  ES_Event ReturnEvent = {ES_NO_EVENT};
  SpectreState NextState = CurrentState;
  
  // SPECTRE Xbee state machine
  switch(CurrentState) {
    case UnpairedState :
      if(CurrentEvent.EventType == PackageReceived) {
        // Import the data for useability (probably not necessary)
        importData();
        // Quickly check if this is a Tx status from the Xbee
        if(rxData[API_ID] == 0x89) {
          break; //just ignore it
        }
        // Check for pair request header
        if(rxData[RX_HEADER] == PAIR_REQ) {
          // Check to see if we're desired
          if(rxData[REQ_PAIR] != MY_ADDRESS) {
            break; //if not, throw the data out
          }
          // If we aren't rejecting this guy
          if(RejectAddrMSB != rxData[SOURCE_ADDR_MSB] || RejectAddrLSB != rxData[SOURCE_ADDR_LSB]) {
            // Update paired indicator
            MasterAddrMSB = rxData[SOURCE_ADDR_MSB]; //store master address values
            MasterAddrLSB = rxData[SOURCE_ADDR_LSB];
            // Send Acknowledge to MI6
            LoadUpConfirmation();
            SendXbeeData();
            
            // Let the motors know we're ready to go
            ES_Event MotorEvent;
            MotorEvent.EventType = StartUp;
            PostMotorDriver(MotorEvent);
            NextState = PairedState;
            
            // Play sounds depending on who connects with us (Boo Team 5)
            if(MasterAddrLSB == 0x85) {
              PlaySound(HUGE_FAN_OF_COCK);
            }else {
              PlaySound(IT_COULD_GO_OFF);
            }
            closeSuicideDoors();
            ES_Timer_InitTimer(CommTimer,COMM_TIMEOUT); //initiate pulse timer
          }else {
            // Ignore
          }
        } else { 
          // Someone is trying to force their will upon us (ignore)
        }
      }
      
      //If enough time has passed that we don't reject the balloon pop MI6 anymore
      else if (CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == RejectTimer) {
        RejectAddrMSB = 0x00;
        RejectAddrLSB = 0x00;
      }
      else if(CurrentEvent.EventType == Balloon) {
        //balloon state has changed, flip the mind bit
        mind = mind ^ 0x01;
      }
      break;  
    case PairedState :
      // New Command Recieved
      if(CurrentEvent.EventType == PackageReceived) {
        importData(); //get that fine piece of data
        // Quickly check if this is a message status from the Xbee
        if(rxData[API_ID] == 0x89) {
          break; //just ignore it
        }
        // Make sure to ignore broadcasts
        if(rxData[RX_OPTIONS] == BROADCAST) {
          break;
        }
        // Make sure people the header is correct
        if(rxData[RX_HEADER] != CTRL) {
          break;
        }
        // Make sure it's our partner
        if(MasterAddrMSB != rxData[SOURCE_ADDR_MSB] || MasterAddrLSB != rxData[SOURCE_ADDR_LSB]) {
          break;
        }
        
        // Send a status response to the master
        LoadUpStatus(mind);
        SendXbeeData();
        
        // Let the motor controller know that new data is ready
        ES_Event MotorEvent;
        MotorEvent.EventType = PackageReceived;
        PostMotorDriver(MotorEvent); //post to the motor driver
        
        // Check for pop command
        if((rxData[ACTION] & POP_MASK) == POP_MASK) {
          // Post pop event
          ES_Event PopEvent;
          PopEvent.EventType = Pop;
          PostBalloon(PopEvent);
          PlaySound(RAMPAGE);
          
        }
        // Reset the comm timer
        ES_Timer_InitTimer(CommTimer,COMM_TIMEOUT);
        
        // See if we should unpair
        if((rxData[ACTION] & UNPAIR_MASK) == UNPAIR_MASK) {
          //go to the unpaired state
          NextState = UnpairedState;
          ES_Timer_StopTimer(CommTimer); //shut off the timer
          // Shut down motors
          ES_Event MotorEvent;
          MotorEvent.EventType = ShutDown;
          openSuicideDoors();
          PostMotorDriver(MotorEvent); //tell the motors to shut down
          PostBalloon(MotorEvent);  //make sure the balloon popper is shut off  
        }
      }
      // Timeouts!
      else if(CurrentEvent.EventType == ES_TIMEOUT) {
        if(CurrentEvent.EventParam == RejectTimer) {
          RejectAddrMSB = 0x00;
          RejectAddrLSB = 0x00;
        }else if(CurrentEvent.EventParam == CommTimer) { 
          // Somehow we lost connection, update paired indicator
          NextState = UnpairedState;
          // Shut the motors down
          ES_Event MotorEvent;
          MotorEvent.EventType = ShutDown;
          openSuicideDoors();
          PostMotorDriver(MotorEvent);
        }
      }
      // Balloon popped
      else if(CurrentEvent.EventType == Balloon) {
        
        // First flip the mind bit
        mind = mind ^ 0x01;
        
        // Send a balloon pop status to our friend the MI6;
        LoadUpStatus(mind);
        SendXbeeData();

        NextState = UnpairedState;
        
        // Shut the motors down
        ES_Event MotorEvent;
        MotorEvent.EventType = ShutDown;
        openSuicideDoors();
        PostMotorDriver(MotorEvent);
        // Set reject address
        RejectAddrMSB = MasterAddrMSB;
        RejectAddrLSB = MasterAddrLSB;
        // Start reject timer
        ES_Timer_InitTimer(RejectTimer,REJECT_TIMEOUT);
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
   UART1InterruptResponse

 Description
  Handles the rx and tx interrupts
****************************************************************************/
void UART1InterruptResponse( void ) 
{
  // If the interrupt response is from Tx FIFO open
  if(((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_TXRIS) >0)||flag){
      //Clear the flag
      HWREG(UART1_BASE + UART_O_ICR) |=UART_ICR_TXIC;

      // Put data to be transfered on UART1 data line
      if((txIndex%sizeof(txData)>0)||flag){
        flag = false;
        HWREG(UART1_BASE + UART_O_DR) = txData[txIndex%sizeof(txData)];
        txIndex++;
      }
  }
  
  // If the interrupt response is from Rx (new data)
  if((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_RXRIS) >0){
    // Clear the source of the interrupt
    HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_RXIC;
    
    // Post that a new bye was recieved
    ES_Event ThisEvent;
    ThisEvent.EventType = ByteReceived;
    ThisEvent.EventParam = HWREG(UART1_BASE + UART_O_DR);
    PostCommRxSM(ThisEvent);
  }
}

/****************************************************************************
 Function
   InputCaptureResponse

 Description
  Handles the interrupt from a pin going high
****************************************************************************/
void InputCaptureResponse( void ) 
{
  // Clear the source of the interrupt
  HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
  
  // Post balloon event
  ES_Event NewEvent;
  NewEvent.EventType = Balloon;
  PostSpectreXbee(NewEvent);
}

/****************************************************************************
 Function
   SoftwareReset

 Description
  Exactly what you think
****************************************************************************/
void SoftwareReset( void ) {
  SysCtlReset();
}

/****************************************************************************
 Function
   importData

 Description
   Moves the data into memory for easier usibility rather than a pointer
****************************************************************************/
void importData(void) {
  rx = GetRxData();
  for(uint8_t i = 0; i < sizeof(rxData); i++) {
    rxData[i] = *(rx + i);
  }
}


/****************************************************************************
 Function
   GetCommands

 Returns
   uint8_t pointer 

 Description
  Returns a pointer to the command data array
****************************************************************************/
uint8_t * GetCommands(void) {
  CommandData[0] = rxData[THRUST];
  CommandData[1] = rxData[ORIENT];
  CommandData[2] = rxData[ACTION];
  CommandData[3] = rxData[EXA];
  CommandData[4] = rxData[EXB];
  CommandData[5] = rxData[EXC];
  return CommandData;
}


/****************************************************************************
 Function
   checkSum

 Parameters
   chkArray, size (needed as the array decays into a point when passed into
              a function, i.e. we can't call sizeof() 

 Returns
   hex value

 Description
  calculates the checkSum for error checking
****************************************************************************/
uint8_t checkSum(uint8_t chkArray[], uint8_t size) {
  uint8_t sum = 0;
  for(int i = 3; i < size-1; i++) {
    sum += chkArray[i];
  }
  return 0xFF - sum;
}


/****************************************************************************
 Function
   LoadUpConfirmation

 Description
   Gets the tx packet ready for a confirmation message
****************************************************************************/
void LoadUpConfirmation(void) {
  txData[0] = 0x7E; //Start delimiter
  txData[1] = 0x00; //length MSB
  txData[2] = 0x07; //length LSB
  txData[3] = 0x01; //API ID
  txData[4] = 0x52; //Frame ID
  txData[DEST_ADDR_MSB] = MasterAddrMSB; //set destination address msb
  txData[DEST_ADDR_LSB] = MasterAddrLSB; //set destination address lsb
  txData[TX_OPTIONS] = 0x00; //set options to not be broadcast
  txData[TX_HEADER] = PAIR_ACK; //set the header to be pair acknowledge
  txData[ACKNOWLEDGE] = 0x01; //set the acknowledge bit
  txData[TX_CHKSUM] = checkSum(txData,11); //calculate the checksum
}

/****************************************************************************
 Function
   LoadUpStatus

 Parameters
   statusVal, the mind control bit

 Description
  Gets the tx packet ready for a status message
****************************************************************************/
void LoadUpStatus(uint8_t statusVal) {
  txData[0] = 0x7E; //Start delimiter
  txData[1] = 0x00; //length MSB
  txData[2] = 0x07; //length LSB
  txData[3] = 0x01; //API ID
  txData[4] = 0x52; //Frame ID
  txData[DEST_ADDR_MSB] = MasterAddrMSB; //set destination address msb
  txData[DEST_ADDR_LSB] = MasterAddrLSB; //set destination address lsb
  txData[TX_OPTIONS] = 0x00; //set options to not be broadcast
  txData[TX_HEADER] = STATUS; //set the header to be pair status
  txData[SDATA] = statusVal; //set the mind status bit
  txData[TX_CHKSUM] = checkSum(txData,11); //calculate the checksum
}


/****************************************************************************
 Function
   SendXbeeData

 Description
   Sends the current txData out via UART1
****************************************************************************/
void SendXbeeData( void ) 
{
  txIndex = 0;
  flag = true;
  UART1InterruptResponse();
}


/****************************************************************************
 Function
   printData

 Description
   Prints the current data packet
****************************************************************************/
void printData(void) {
  for(int i=0;i<sizeof(txData);i++)
  printf("Data Received: %x\r\n",txData[i]);
}


/****************************************************************************
 Function
   InitUart1

 Parameters
   none

 Returns
   none

 Description
   Initialize UART module 1
****************************************************************************/
static void InitUART( void )
{
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
  13. Enable UART1 interrupt vector in NVIC (Bit 6 in NVIC_EN0) (Tiva p.104 for vector table)
  13. Enable the UART by setting the UARTEN bit in the UARTCTL register
  
  UART Module 1:
  U1RX - Receive (PC4 or PB0)     We're using PC4
  U1TX - Transmit (PC5 or PB1)    We're using PC5
  */
  
  // Enable the clock to the UART module 1
  HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R1;
  
  // Enable clock to the GPIO port C
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  
  
  // Wait for GPIO port C to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R2 ) != SYSCTL_PRGPIO_R2 )
    ;
  
  // Configure GPIO pins for in/out/drive-level/drivetype
  // Set GPIO PC4 and PC5 as digital
  HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT4HI | BIT5HI);
  
  // Set GPIO PC4 as input (receive) and PC5 as output (transfer)
  HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) |= BIT5HI;
  HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= ~BIT4HI;
  
  // Configure the transfer line (PC5) as open drain, see TIVA p.676
  //HWREG(GPIO_PORTC_BASE + GPIO_O_ODR) |= BIT5HI;
  
  // Program pins C4, C5 as alternate functions on the GPIO to use UART (Set High) 
  HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT4HI | BIT5HI);
  
  // Set mux position on GPIOPCTL to select UART alternate function on C4, C5 (Tiva p.1351)
  // Mux value = 2 (p.895) offset mask to clear nibbles 4 and 5
  HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) =                                                                      
    (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~0x00ff0000) + (2 << ( 4 * BitsPerNibble)) +
      (2 << ( 5 * BitsPerNibble));
  
  // Disable UART module 1
  HWREG(UART1_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;
  
  // Baud Rate = 9600, Integer = 260, Fraction = 27
  // Write the integer portion of the BRD (260)
  HWREG(UART1_BASE + UART_O_IBRD) = 0x0104;
  
  // Write the fractional portion of the BRD (27)
  HWREG(UART1_BASE + UART_O_FBRD) = 0x1B;
  
  // Write desired serial parameters to the UART line control
  HWREG(UART1_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8; // Set 8-bit frame length and clear everything else 

  // Configure UART operation
  HWREG(UART1_BASE + UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE|UART_CTL_EOT); // Enable Receive and Transmit
  
  // Locally enable interupts for UART receive interrupt mask (RXIM)
  HWREG(UART1_BASE + UART_O_IM) |= (UART_IM_RXIM|UART_IM_TXIM);

  // Set NVIC enable for UART1 (see Tiva p.104)
  HWREG(NVIC_EN0) |= BIT6HI;
  
  // Make sure interrupts are enabled globally
  __enable_irq( );

  // Enable UART module 1
  HWREG(UART1_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

  // Print to console if successful initialization
  printf("UART Initialized\n\r");
}

/****************************************************************************
 Function
   InitInputCaptureWT1

 Parameters
   none

 Returns
   none

 Description
   Initialize Input Capture on C6 and C7 (Wide timer 1/PWM1-WT1CCP0)
****************************************************************************/
static void InitInputCaptureWT1( void )
  {
    /*
  Steps to initialize Tiva Wide Timer Input Capture
  1.  Enable the clock to wide timer 1 using RCGCWTIMER_R1 register
  2.  Enable the clock to the appropriate GPIO module RCGCGPIO
  3.  Disable wtimer 1 timer A to configure with TAEN in TIMER_O_CTL register
  4.  Set timer to 32bit wide mode with TIMER_CFG_16_BIT in TIMER_O_CTL register
  5.  Set timer to use full 32bit count by setting TIMER_O_TAILR to 0xffffffff
  6.  Set timer A to capture mode for edge time and up-counting (Clear TAMMS and Set TACMR,TACDIR,TAMR_CAP in TIMERTAMR)
  7.  Set event to rising edge by clearing TAEVENT_M in TIMER_O_CTL (Rising edge = 00)
  8.  Select the alternate function for the Timer pins (AFSEL)
  9.  Configure the PMCn fields in the GPIOPCTL register to assign the WTIMER pins (WT1CCP0)
  10. Enable appropriate pint on GPIO port for digital I/O (GPIODIR)
  11. Set appropriate pins on GPIO as inputs (GPIODEN)
  12. Locally enable interrupts for the capture interrupt mask (CAEIM in TIMERIMR)
  13. Enable WTIMER1 Timer A interrupt vector in NVIC (96 = Bit 0 in NVIC_EN3) (Tiva p.104 for vector table)
  14. Ensure interrupts are enabled globally (__enable_irq())
  14. Enable WTIMER1 Timer A with TAEN in TIMER_O_CTL register
  
  WTIMER1 Timer A - Tiva PC6
  */
    
  // Enable the clock to the timer (Wide Timer 1)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
    
  // Enable the clock to Port C
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  
  // Disable timer A before configuring
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
  //HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M);
  HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M);

  // Set up the alternate function for Port C bit 6 (WT1CCP0)
  HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT6HI | BIT7HI);

  // Set mux position on GPIOPCTL to select WT1CCP0 alternate function on C6
  // Mux value = 7 offset mask to clear nibble 6
  HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~0x0f000000) + (7 << ( 6 * BitsPerNibble)) + 
                                                           (7 << ( 7 * BitsPerNibble));
      
  // Enable pin 6 on Port C for digital I/O
  HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
  
  // Make pin 6 on Port C into an input
  HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= (BIT6LO & BIT7LO);

  // Enable a local capture interrupt
  HWREG(WTIMER1_BASE + TIMER_O_IMR) |= (TIMER_IMR_CAEIM | TIMER_IMR_CBEIM);

  // Enable Timer A Wide Timer 1 interrupt in the NVIC
  // NVIC number 96 (EN3 bit 0) Tiva p.104
  HWREG(NVIC_EN3) |= (BIT0HI | BIT1HI);

  // Make sure interrupts are enabled globally
  __enable_irq();

  // Enable Timer
  HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL | 
                                        TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
  
  // Print to console if successful initialization
  printf("Wide Timer 1 A&B interrupt initialized\n\r");
}

/****************************************************************************
 Function
   InitGPIO

 Parameters
   none

 Returns
   none

 Description
   Initialize GPIO pins for inputs/outputs

    Inputs:           Outputs:
    Addr1 - PB7       Data2 - PA5
    Addr0 - PB6       Data1 - PA4
    Data2 - PB5       Data0 - PA3
    Data1 - PB4
    Data0 - PB3

****************************************************************************/
static void InitGPIO( void )
{
  // Enable Port F to the timer
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5; 
  
  // Enable Port A Timer
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
    
  // Wait for GPIO Port B to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0 ) != SYSCTL_PRGPIO_R0 )
    ;
  
  // Set Pin F3 as digital output
  HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (BIT3HI);
  HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) &= ~(BIT3HI);
    
  
  // Set Pins A3-A5 as digital
  HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= ( BIT3HI | BIT4HI | BIT5HI );
  // Set Pins A3-A5 as inputs
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) &= ~( BIT3HI | BIT4HI | BIT5HI );
  
  // Print to console if successful initialization
  printf("GPIO Initialized\n\r");
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

