/****************************************************************************
 Module
  MotorDriver.c

 Revision     Revised by: 
  0.1.1       Denny
  0.1.2       Denny
  1.0.0       Eric
  1.0.1       Eric
  2.0.0       Alex

 Description
  Test harness for controlling hovercraft lift and propulsion

 Edits:
  0.1.1       Set up
  0.1.2       Added PWM functionality to drive ESC's 
  1.0.0       Graduated to official module. Modified to take posts from the 
                spectreXbee module
  1.0.1       Fixed improper thrust and orientation control values; now they
                follow the protocol. 
              General cleanup
  2.0.0       Added Controls

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
#include "MotorDriver.h"
#include "PWM.h"
#include "SpectreXbee.h"
#include "SensorComms.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)
#define BLOWER_MASK         0x01
#define PROP_MASK           0x02
#define PROP_WIDTH          1125        // 1000 = full stop , 1125 = slowest, 1350 = fastest
#define MIN_PROP_WIDTH      1000
#define MAX_PROP_WIDTH      1350
#define MAX_THRUST          113         //hard limit for thrust
#define PORT                4           //pwm channel
#define STARBOARD           5
//control array defines
#define THR                 0 
#define ORT                 1
#define ACT                 2
#define EXA                 3
#define EXB                 4
#define EXC                 5

#define BRAKE_MASK          0x02

// Brake defines
#define PWM_WIDTH_DOWN      1450
#define PWM_WIDTH_UP        2350
#define BRAKE_PWM_CHL       7

//control law constants
#define YAW_CONSTANT        0.1
#define K                   25

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
static void InitGPIO( void );
static void importControl(void);
static void writeThrust(void);
static void writeOrient(void);
static void ControlLaw(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static MotorState CurrentState; 

static uint32_t portWidth;      //pwm width
static uint32_t starWidth;

static uint8_t uThrust;         //unsigned thrust value for conversion
static signed int thrust;       //signed value for evaluation
static bool thrustCondition = false;
static uint8_t uOrient;         //unsigned orient value for conversion
static signed int orient;       //signed value for evaluation
static uint8_t action;

static uint8_t *cmd;            //pointer for bringing in data

static double inputAngle = 0;   //input angle (in degrees)
static double desiredYaw = 0;   //desired yaw rate (degrees/sec)
static double steerAngle = 0;   //pwm value


static bool brakeOn = false;    // is the brake on?
static bool control = true;     //true if control law is on

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
  InitMotorDriver

 Parameters
  uint8_t : the priorty of this service

 Returns
  bool, false if error in initialization, true otherwise

 Description
  B0 = Lift fan
  B4 = Prop1
  B5 = Prop2
****************************************************************************/
bool InitMotorDriver  ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
  
  // Initialize GPIO output for lift fan
  InitGPIO(); 
  
  // Disable motors initially
  HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= ~( BIT0HI);
  SetPWMWidth(MIN_PROP_WIDTH, PORT);
  SetPWMWidth(MIN_PROP_WIDTH, STARBOARD);
  CurrentState = UncontrolledState;
  
  printf("motor initiated\r\n");

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
  PostMotorDriver 

 Parameters
    ES_Event ThisEvent ,the event to post to the queue

 Returns
    bool false if the Enqueue operation failed, true otherwise

 Description
    Posts an event to this state machine's queue
****************************************************************************/
bool PostMotorDriver( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
  RunMotorDriver 

 Parameters
  ES_Event : the event to process

 Returns
  ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
  
****************************************************************************/
ES_Event RunMotorDriver ( ES_Event CurrentEvent )
{
  ES_Event ReturnEvent = {ES_NO_EVENT}; 
  MotorState NextState = CurrentState;
  
  switch(CurrentState) {
    case UncontrolledState :
      if(CurrentEvent.EventType == StartUp) {
        NextState = ControlledState;
        // Turn on lift fan by toggling bit (1 = on, 0 = off);
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) = 
        HWREG(GPIO_PORTB_BASE + ALL_BITS) ^ BLOWER_MASK;
        // Turn on headlights and taillights
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) |= BIT2HI; 
        printf("entering controlled state\r\n");
      }
      break;
    case ControlledState :
      importControl();    //bring in the data
      if(CurrentEvent.EventType == ShutDown) {
        // Shut down everything
        SetPWMWidth(MIN_PROP_WIDTH,PORT);
        SetPWMWidth(MIN_PROP_WIDTH,STARBOARD);
        NextState = UncontrolledState;
        
        // Turn off lift fan by toggling bit (1 = on, 0 = off);
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) = 
        HWREG(GPIO_PORTB_BASE + ALL_BITS) ^ BLOWER_MASK;
        printf("Shutting Down...\r\n");
        
        // Make sure the popper is shut off
        HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT4LO; 
        // Turn off head and tail lights
        HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT2LO; 
        break;
      }
      
      // Write initial thrust values
      writeThrust();    
      
      // Implement control law 
      if(control == true) {
        ControlLaw();
      }
      else {
        steerAngle = orient/2;
      }
      
      // Write orientation values
      writeOrient();
      
      // Put calculated values on ports
      SetPWMWidth(portWidth,PORT);
      SetPWMWidth(starWidth,STARBOARD);

      // Check for braking
      if((action & BRAKE_MASK) == BRAKE_MASK) {
        printf("Brake Pressed \r\n");
        // Apply braking if brake is off
        if(!brakeOn) {
          SetPWMWidth(PWM_WIDTH_DOWN,BRAKE_PWM_CHL);
          brakeOn = true;
        }
      }
      else {
        // Release brake if brake is on
        if(brakeOn) {
          SetPWMWidth(PWM_WIDTH_UP,BRAKE_PWM_CHL);
          brakeOn = false;
        }
      }
      
      break;
  }
  
  CurrentState = NextState;
  return ReturnEvent;
}

/****************************************************************************
 Function
   writeThrust

 Description
  determines the proper pwm width using the commanded thrust value
****************************************************************************/
void writeThrust(void) {
  // Check for zero thrust
  if (thrust <= 5 || thrust > 128) {
    thrustCondition = false;
    portWidth = MIN_PROP_WIDTH;
    starWidth = MIN_PROP_WIDTH;
  }
  // Set the minimum stable thrust + user thrust
  else if(thrust <= MAX_THRUST & thrust > 5){
    thrustCondition = true;
    portWidth = PROP_WIDTH + (thrust-5)*2;
    starWidth = PROP_WIDTH + (thrust-5)*2;
  }
  // Same as before but with a maximum thrust value
  else {
    thrustCondition = true;
    portWidth = MAX_PROP_WIDTH;
    starWidth = MAX_PROP_WIDTH;
  }
}

/****************************************************************************
 Function
   writeOrient

 Description
  Modifies the pwm width from writeThrust using the commanded orient value
****************************************************************************/
void writeOrient(void) {

  if(orient >= 0) { //this is a right turn
    if(orient > 63 && thrustCondition == false) {
      portWidth = PROP_WIDTH + (orient - 64)*4; //zero thrust turn
    }
    else if(thrustCondition == true) {
      starWidth -= steerAngle;    //decrease the starboard prop
      portWidth += steerAngle;    //increase the port prop
    }
  }
  else { //this is a left turn
    if(orient < -64 && thrustCondition == false) {
      starWidth = PROP_WIDTH + (-64 - orient)*4; //zero thrust turn
    }
    else if(thrustCondition == true) {
      starWidth -= steerAngle;    //increase the starboard prop
      portWidth += steerAngle;    //decrease the port prop
    }
  }

  // Check each value for a maximum or minimum number, if so, set it to the 
  // relative max/min
  if(portWidth > MAX_PROP_WIDTH) {
    portWidth = MAX_PROP_WIDTH;
  }
  if(starWidth > MAX_PROP_WIDTH) {
    starWidth = MAX_PROP_WIDTH;
  }
  if(portWidth < PROP_WIDTH) {
    portWidth = MIN_PROP_WIDTH;
  }
  if(starWidth < PROP_WIDTH) {
    starWidth = MIN_PROP_WIDTH;
  }
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
                  B0 as impeller

****************************************************************************/
static void InitGPIO( void )
{
  // Enable Port B to the timer
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
  
  // Wait for GPIO Port B to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1 ) != SYSCTL_PRGPIO_R1 )
    ;
  
  // Set Pins B0-B1 as digital
  HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= ( BIT0HI | BIT2HI );
  // Set Pins B0-B1 as outputs
  HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= ( BIT0HI | BIT2HI );
  
  // Print to console if successful initialization
  printf("GPIO Initialized\n\r");
}

/****************************************************************************
 Function
   importControl

 Description
  stores the values from the pointer into local memory for easier acces
    - additionally, applies a uint8_t to signed int transformation for
      thrust and orient
****************************************************************************/
void importControl(void) {
  cmd = GetCommands();

  uThrust = *(cmd);
  uOrient = *(cmd+1);
  action = *(cmd+2);
  if(uThrust >= 128) {
    thrust = uThrust - 256;
  }else {
    thrust = uThrust;
  }
  if(uOrient >= 128) {
    orient = uOrient - 256;
  }else {
    orient = uOrient;
  }
  
  // Convert input vlaue to angle stick makes with neutral vertical axis
  inputAngle = (double)orient * 90 / 128;
}

/****************************************************************************
 Function
   ControlLaw

 Description
  stores the values from the pointer into local memory for easier acces
    - additionally, applies a uint8_t to signed int transformation for
      thrust and orient
****************************************************************************/
void ControlLaw(void) {
  
  // Calculate desired yaw rate based on input from controller
  desiredYaw = inputAngle*YAW_CONSTANT;
  
  // Implement proportional control on yaw rate
  steerAngle = -K*(getYaw() - desiredYaw);
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
