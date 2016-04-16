/****************************************************************************
 Module
   CompassCom.c

Revision			Revised by: 
	1.0.0				Denny

 Description
    This module handles communication with an MM Blue Compass. Data is
    provided through the I2C service. The only output from the service
    is the compass reading and any error indication.

 Notes

 Edits:
	1.0.0	    Initial setup
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"

#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "bitdefs.h"
#include "termio.h"

#include "I2C.h"
#include "CompassCom.h"

/*----------------------------- Module Defines ----------------------------*/
// TODO: Will need to account for changing clock speeds (when chip is hibernating)
#define ONE_SEC                 976
#define PI                      3.14
#define RAD_PER_REV             (360.0/(2.0*PI))
#define COMPASS_ADDRESS         0x28
#define COMPASS_I2C_MODULE      1
#define NUM_INIT_STEPS          7
#define MM_INTERRUPT_PIN_HIGH   0x01
#define MAX_RESPONSE_BYTES      0x04

// MM Register definitions
#define MM_REG_SENTRAL_STATUS    0x37
#define MM_REG_RESET_REQUEST     0x9B
#define MM_REG_MAG_RATE          0x55
#define MM_REG_ACCEL_RATE        0x56
#define MM_REG_GYRO_RATE         0x57
#define MM_REG_QRATE_DIVISOR     0x32
#define MM_REG_ALGORITHM_CONTROL 0x54
#define MM_REG_ENABLE_EVENTS     0x33
#define MM_REG_HOST_CONTROL      0x34
#define MM_REG_EVENT_STATUS      0x35

#define MM_REG_QX0              0x00 // Float32
#define MM_REG_QY0              0x04 // Float32
#define MM_REG_QZ2              0x08 // Float32
#define MM_REG_QW0              0x0C // Float32
#define MM_REG_QTime            0x10 // UInt16

#define MM_REG_HEADING          0x00 // Float32
#define MM_REG_PITCH            0x04 // Float32
#define MM_REG_ROLL             0x08 // Float32

#define MM_REG_ALG_STATUS             0x38
#define MM_REG_PASS_THROUGH_CONTROLL  0xA0
#define MM_REG_PASS_THROUGH_STATUS    0x9E
#define MM_REG_SENSOR_STATUS          0x36


// MM Reset register values
#define MM_REQUEST_RESET        0x01

// MM ESR values
#define MM_ERROR_BIT            BIT1HI
#define MM_NEW_RESULT_BIT       BIT2HI

// MM Error register values
#define MM_NO_ERROR             0x00
#define MM_INVALID_SAMPLE_RATE  0x80
#define MM_MATH_ERROR           0x30
#define MM_MAG_INIT_FAILURE     0x21
#define MM_ACCEL_INIT_FAILURE   0x22
#define MM_GYRO_INIT_FAILURE    0x24
#define MM_MAG_RATE_FAILURE     0x11
#define MM_ACCEL_RATE_FAILURE   0x12
#define MM_GYRO_RATE_FAILURE    0x14

// Sentral status register values
#define MM_EEPROM_DETECTED      BIT0HI
#define MM_EEPROM_UPLOAD_DONE   BIT1HI
#define MM_EEPROM_UPLOAD_ERROR  BIT2HI
#define MM_IDLE                 BIT3HI
#define MM_NO_EEPROM            BIT4HI

typedef enum {
    PowerUp,
    RegisterSetUp,
    NormalOperation,
    Resetting,
    Standby
} MM_State_t; 

typedef enum {
    NoError,
    InvalidSampleRate,
    MathError,
    MagnetInitFail,
    AccelInitFail,
    GyroInitFail,
    MagnetRateFail,
    AccelRateFail,
    GyroRateFail,
} MM_ErrorType_t;

/*---------------------------- Module Functions ---------------------------*/
static uint16_t decodeReading(void);
static uint16_t convertToDegrees(float f);
static bool takeInitStep(void);
static void getRegisterValue(uint8_t reg, uint8_t responseBytes);
static void writeRegisterValue(uint8_t reg, uint8_t value);
static void initInterruptPin(void);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;              // Framework priority
static MM_State_t CurrentState;         // Keep track of our state
static uint8_t I2CResponse[4];          // Response from compass
static uint8_t I2CCommand[4];           // Command to compass
static CompassReading_t LastReading;    // Last complete reading from compass

static uint8_t initIndex = 0;           // Index to step through set up registers
static uint8_t SetUpRegisters[7] = {    // Registers we need to set after power up
    MM_REG_MAG_RATE,
    MM_REG_ACCEL_RATE,
    MM_REG_GYRO_RATE,
    MM_REG_QRATE_DIVISOR,
    MM_REG_ALGORITHM_CONTROL,
    MM_REG_ENABLE_EVENTS,
    MM_REG_HOST_CONTROL
};
static uint8_t SetUpValues[7] = {       // Values to place in the those registers
    0x0A,   // Mag rate of 10Hz
    0x01,   // Accel rate of 10Hz (register requires /10 value)
    0x01,   // Gyro rate of 10Hz (register requires /10 value)
    0x0A,   // Sets data output to 1Hz (Gyro rate/Reg value)
    0x06,   // Enable heading, pitch, roll reporting (not quaternions)
    0x07,   // Enable interrupts (Error, Measurement, Reset)
    0x01    // Enable compass to run normally
};

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitCompassCom
 Parameters
     uint8_t : the priorty of this service
 Returns
     bool, false if error in initialization, true otherwise
 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
****************************************************************************/
bool InitCompassCom ( uint8_t Priority ) {
    MyPriority = Priority;
    CurrentState = PowerUp;
    
    // Init GPIO pin (D0) for interrupts from MM
    //initInterruptPin();
    
    printf("CompassCom initialized\r\n");
    // Post the initial transition event
    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     PostCompassCom
 Parameters
     EF_Event ThisEvent , the event to post to the queue
 Returns
     boolean False if the Enqueue operation failed, True otherwise
 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostCompassCom(ES_Event ThisEvent) {
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunCompassCom
 Parameters
   ES_Event : the event to process
 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description
   Implements CompassCom state machine
 Notes
****************************************************************************/
ES_Event RunCompassCom( ES_Event ThisEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT, 0}; // assume no errors
    uint8_t lastCommand = I2CCommand[0];
    
    // TODO: If we have a transfer failure from the I2C Module 
    if(ThisEvent.EventType == I2C_TransferFailure) {
        printf("I2C Transfer failure in state: %d\r\n", CurrentState);
    }
    
    switch(CurrentState) {
        case PowerUp :
            // If we just came out of initialization
            if(ThisEvent.EventType == ES_INIT) {
                // Check for a successful power up/EEPROM upload
                setI2CPostFunction(COMPASS_I2C_MODULE, PostCompassCom);
                getRegisterValue(MM_REG_SENTRAL_STATUS, 1);
            }
            // If we have a response from the MM to read
            else if(ThisEvent.EventType == I2C_TransferComplete) {
                uint8_t StatusReg = I2CResponse[0];
                // If the EEPROM was detected and uploaded
                if((StatusReg & MM_EEPROM_DETECTED) && (StatusReg & MM_EEPROM_UPLOAD_DONE)) {
                    // If we have an upload error, send reset command
                    if(StatusReg & MM_EEPROM_UPLOAD_ERROR) {
                        CurrentState = Resetting;
                        writeRegisterValue(MM_REG_RESET_REQUEST, MM_REQUEST_RESET);
                    } else {
                        // Upload complete with no errors, move to register set up
                        printf("Power up complete!\r\n");
                        CurrentState = RegisterSetUp;
                        takeInitStep();
                    }
                // MM Power up not complete, poll status register again
                } else {
                    printf ("Get Sentral Status register again: 0x%X\r\n", StatusReg);
                    //getRegisterValue(MM_REG_SENTRAL_STATUS, 1);
                }
            }
            break;
            
        case RegisterSetUp : 
            // If we have a response from the MM to read
            if(ThisEvent.EventType == I2C_TransferComplete) {
                // Take init step returns false if no more steps to take
                if(!takeInitStep()) { 
                    printf("Initialization complete! Normal operation\r\n");
                    CurrentState = NormalOperation;
                    getRegisterValue(MM_REG_EVENT_STATUS, 1);
                }
            }
            break;   
            
        case NormalOperation : 
            // If it's a compass interrupt read the ESR
            if(ThisEvent.EventType == GetHeading) {
                getRegisterValue(MM_REG_EVENT_STATUS, 1);
            }
            // If it's a transfer complete
            else if(ThisEvent.EventType == I2C_TransferComplete) {
                // Last command was to poll the ESR
                if(lastCommand == MM_REG_EVENT_STATUS) {
                    uint8_t ESRValue = I2CResponse[0];
                    // Check the error bit, and send a reset command if true
                    if(ESRValue & MM_ERROR_BIT) {
                        CurrentState = Resetting;
                        writeRegisterValue(MM_REG_RESET_REQUEST, MM_REQUEST_RESET);
                    // Check new result bit, start reading if true
                    } else if (ESRValue & MM_NEW_RESULT_BIT) {
                        // Start grabbing new data
                        getRegisterValue(MM_REG_HEADING, 4);
                    }
                } else if(lastCommand == MM_REG_HEADING) {
                    // Save heading and read pitch
                    LastReading.heading = decodeReading();
                    printf("Heading: %d\r\n", LastReading.heading);
                    getRegisterValue(MM_REG_PITCH, 4);
                    
                } else if(lastCommand == MM_REG_PITCH) {
                    // Save pitch and read roll
                    LastReading.pitch = decodeReading();
                    printf("Pitch: %d\r\n", LastReading.pitch);
                    getRegisterValue(MM_REG_ROLL, 4);
                    
                } else if(lastCommand == MM_REG_ROLL) {
                    // Save roll and post new compass data available (save to LastReading)
                    LastReading.roll = decodeReading();
                    printf("Roll: %d\r\n", LastReading.roll);
                }
            // If we timed out, restart communication...
            } else if(ThisEvent.EventType == ES_TIMEOUT) {
                
            }
            break;
            
        case Standby :
            break;
            
        case Resetting : 
            // Reset command processed, move to power up routine and reset init index
            if(ThisEvent.EventType == I2C_TransferComplete) {
                CurrentState = PowerUp;
                initIndex = 0;
                getRegisterValue(MM_REG_SENTRAL_STATUS, 1);
            }
            break;
    }
    return ReturnEvent;
}

/*
Take the 4 bytes we get back from the compass, form into float,
then convert to a uint16_t of degrees 0-359
*/
static uint16_t decodeReading(void) {
    // Make a temporary float and pointer
    float f;
    unsigned char *pc;
    pc = (unsigned char*)&f;

    // Insert the bytes (little endian format)
    pc[0] = I2CResponse[0];
    pc[1] = I2CResponse[1];
    pc[2] = I2CResponse[2];
    pc[3] = I2CResponse[3]; 
    
    // Convert the float to a 16bit unsigned integer (0-359 degrees)
    if(f < 0.0) {
        return 180 - convertToDegrees(-1*f);
    } else {
        return 180 + convertToDegrees(f);
    }
}

static uint16_t convertToDegrees(float f) {
    float degrees = f * RAD_PER_REV;
    return (uint16_t) degrees;
}

void PortDInterruptHandler(void) {
    printf("Port D Interrupt");
    //Read the masked interrupt status to see which pin triggered the interrput
    uint32_t status = GPIOIntStatus(GPIO_PORTD_BASE, true);
    if(status & MM_INTERRUPT_PIN_HIGH) {
        printf("... MM interrupt\r\n");
        MM_HostInterruptResponse();
    }
    else { // For protection.. but we shouldn't get here ever.
        printf("... but not MM: %X\r\n", status);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    }
}

void MM_HostInterruptResponse(void) {
    //Clear the interrupt flag
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);
    // Post an event to read the compass ESR
    ES_Event NewEvent = {GetHeading, 0};
    PostCompassCom(NewEvent);
}

CompassReading_t getCompassData(void) {
    printf("Last heading: %d\r\n", LastReading.heading);
    //printf("Getting compass\r\n");
    ES_Event NewEvent = {GetHeading, 0};
    PostCompassCom(NewEvent);
    return LastReading;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static bool takeInitStep(void) {
    // First check if we are done with initialization
    if(initIndex >= NUM_INIT_STEPS) return false; 
    writeRegisterValue(SetUpRegisters[initIndex], SetUpValues[initIndex]);
    initIndex++;
    return true;
}

static void getRegisterValue(uint8_t reg, uint8_t responseBytes) {
    I2CCommand[0] = reg;
    I2CMessage_t newMessage = {COMPASS_I2C_MODULE, COMPASS_ADDRESS, I2CCommand, 1, I2CResponse, responseBytes};
    I2CStart(&newMessage);
    //start a timeout
}

static void writeRegisterValue(uint8_t reg, uint8_t value) {
    I2CCommand[0] = reg;
    I2CCommand[1] = value;
    I2CMessage_t newMessage = {COMPASS_I2C_MODULE, COMPASS_ADDRESS, I2CCommand, 2, I2CResponse, 0};
    I2CStart(&newMessage);
    //start a timeout
}

static void initInterruptPin(void) {
    // Initialize GPIO pin D0 as the input interrupt from the MM compass
    // Enable GPIO peripheral (Port D)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Set pin D0 as an input
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
    // Set pin D0 interrupt to be on a rising edge
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
    // Register the interrupt handler (this is for all of port D)
    GPIOIntRegister(GPIO_PORTD_BASE, PortDInterruptHandler);
    // Enable GPIO interrupts on pin 0
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_0);
    // Enable interrupts globally
    IntMasterEnable();
}
