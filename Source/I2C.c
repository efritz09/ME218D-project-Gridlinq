/****************************************************************************
 Module
   I2C.c

Revision            Revised by: 
    1.0.0               Alex
    2.0.0               Denny

 Description
   This service handles master I2C Communication. 

 Notes
    I2C Module 0: PB2 - SCL, PB3 - SDA
    I2C Module 1: PA6 - SCL, PA7 - SDA
 
 Edits:
    1.0.0       Burst Reads
    2.0.0       Coded for multiple I2C modules. 
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

#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "bitdefs.h"
#include "termio.h"

#include "I2C.h"

/*----------------------------- Module Defines ----------------------------*/
#define I2C0    0
#define I2C1    1

typedef enum {
    Idle,
    Sending,
    Receiving
} I2CState_t;

typedef struct {
    uint8_t num;
    uint32_t base;
    I2CState_t state;
    bool error;
    uint8_t slaveAddress;
    uint8_t index;
    uint8_t commandSize;    // number of elements in command
    uint8_t* command;       // pointer to first element in command array
    uint8_t responseSize;   // number of elements in response
    uint8_t* response;      // pointer to first element in response array
    postFuncDef postFunc;   // pointer to post function of service calling this one
} I2CModuleInfo_t;

/*------------------------ Private Module Functions -----------------------*/
static void InitI2CHardware(void);
static void interruptHelper(I2CModuleInfo_t* thisModule);
static void beginTx(I2CModuleInfo_t* thisModule);
static void beginRx(I2CModuleInfo_t* thisModule);
static void printModule(I2CModuleInfo_t* thisModule);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static I2CModuleInfo_t I2CModule0;
static I2CModuleInfo_t I2CModule1;

/***************************************************************************
 Module API Definitions - Descriptions on use in header
 ***************************************************************************/
bool I2CBusy(uint8_t module) {
    if(module == I2C0) return (I2CModule0.state != Idle);
    else if(module == I2C1) return (I2CModule1.state != Idle);
    return false;
}

void setI2CPostFunction(uint8_t module, postFuncDef postFunc) {
    if(module == I2C0) I2CModule0.postFunc = postFunc;
    if(module == I2C1) I2CModule1.postFunc = postFunc;
}

bool I2CStart(I2CMessage_t* message) {
    //printf("I2CStart:\r\n");
    //printf("\tModule: %d\r\n", message->module);
    //printf("\tSlaveAddress: 0x%X\r\n", message->slaveAddress);
    
    I2CModuleInfo_t* thisModule;
    if(message->module == I2C0) thisModule = &I2CModule0;
    else if (message->module == I2C1) thisModule = &I2CModule1;
    else return false; // module specified not used
    if(thisModule->state != Idle) {
        printf("I2C Module %d is busy\r\n", message->module);
        printModule(thisModule);
        return false; // module specified is busy
    }

    thisModule->commandSize = message->commandSize;
    thisModule->command = message->command;
    thisModule->responseSize = message->responseSize;
    thisModule->response = message->response;
    thisModule->slaveAddress = message->slaveAddress;
    
    ES_Event newEvent = {I2C_BeginTransfer, message->module};
    return PostI2C(newEvent);
}

static void printModule(I2CModuleInfo_t* thisModule) {
    printf("Module%d:\r\n", thisModule->num);
    printf("\tIndex: %X\r\n", thisModule->index);
    printf("\tSlaveAddress: 0x%X\r\n", thisModule->slaveAddress);
    printf("\tCommandSize: %d\r\n", thisModule->commandSize);
    if(thisModule->commandSize > 0) {
        printf("\tCommand: ");
        for(int i = 0; i < thisModule->commandSize; i++) {
            printf("0x%X ", thisModule->command[i]);
        }
        printf("\r\n");
    }
    printf("\tResponseSize: %X\r\n", thisModule->responseSize);
    if(thisModule->responseSize > 0) {
        printf("\tResponse: ");
        for(int i = 0; i < thisModule->responseSize; i++) {
            printf("0x%X ", thisModule->response[i]);
        }
        printf("\r\n");
    }
}

/***************************************************************************
 Module Service Code  - Excessive comments ahead...
 ***************************************************************************/

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
bool InitI2C ( uint8_t Priority ) {
    MyPriority = Priority;
    I2CModule0.num = 0;
    I2CModule1.num = 1;
    I2CModule0.base = I2C0_BASE;
    I2CModule1.base = I2C1_BASE;
    I2CModule0.error = false;
    I2CModule1.error = false;

    InitI2CHardware();
    printf("I2C Initialized\r\n"); 

    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService(MyPriority, ThisEvent);
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
bool PostI2C(ES_Event ThisEvent) {
  return ES_PostToService(MyPriority, ThisEvent);
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
ES_Event RunI2C( ES_Event ThisEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT, 0}; // assume no errors

    I2CModuleInfo_t* thisModule;
    
    if(ThisEvent.EventParam == I2C0) { 
        thisModule = &I2CModule0;
    } else if (ThisEvent.EventParam == I2C1) { 
        thisModule = &I2CModule1;
    }
    
    if(ThisEvent.EventType == I2C_Error) {
        //printf("I2C%d Failure: Aborting\r\n", ThisEvent.EventParam);
        thisModule->state = Idle;
        thisModule->error = false;
        ES_Event NewEvent = {I2C_TransferFailure, thisModule->num};
        thisModule->postFunc(NewEvent);
    }
    
    switch(thisModule->state) {
        case Idle :
            if(ThisEvent.EventType == I2C_BeginTransfer) {
                if(thisModule->commandSize != 0) {
                    thisModule->state = Sending;
                    beginTx(thisModule);
                } else {
                    thisModule->state = Receiving;
                    beginRx(thisModule);
                } 
            }
            break;
            
        case Sending :
            if(ThisEvent.EventType == I2C_TxComplete) {
                if(thisModule->responseSize != 0) {
                    thisModule->state = Receiving;
                    beginRx(thisModule);
                } else {
                    ES_Event NewEvent = {I2C_TransferComplete, thisModule->num};
                    thisModule->postFunc(NewEvent);
                    thisModule->state = Idle;
                }
            }
            break;
            
        case Receiving :
            if(ThisEvent.EventType == I2C_RxComplete) {
                //printModule(thisModule);
                ES_Event NewEvent = {I2C_TransferComplete, thisModule->num};
                thisModule->postFunc(NewEvent);
                thisModule->state = Idle;
            }
            break;
    } 
    return ReturnEvent;
}

/****************************************************************************
 Function
    I2C0InterruptResponse
 Parameters
   none
 Returns
   none
 Description
   Interrupt response for I2C Module 0
 Notes
****************************************************************************/
void I2C0InterruptResponse(void) {
    // Clear the Interrupt flag
    I2CMasterIntClear(I2C0_BASE);
    //printf("I2C0 Interrupt:");
    // Call the interrupt helper with I2C module 0 to execute next step
    interruptHelper(&I2CModule0);
}

/****************************************************************************
 Function
    I2C1InterruptResponse
 Parameters
   none
 Returns
   none
 Description
   Interrupt response for I2C Module 1
 Notes
****************************************************************************/
void I2C1InterruptResponse(void) {
    // Clear the Interrupt flag
    I2CMasterIntClear(I2C1_BASE);
    //printf("I2C1 Interrupt:");
    // Call the interrupt helper  with I2C module 1 to execute next step
    interruptHelper(&I2CModule1);
}

/****************************************************************************
 Function
    interruptHelper
 Parameters
   Base of I2C module, Pointer to the module info
 Returns
   none
 Description
   Should implement the interrupt helper for the specified module. If an
   error is detected the module will reset
 Notes
****************************************************************************/
static void interruptHelper(I2CModuleInfo_t* thisModule) {
    // Check for errors. Kill transmission and reset if true.
    if(I2C_MASTER_ERR_NONE != I2CMasterErr(thisModule->base)) {
        //printf("I2C%d Error: %d \r\n", thisModule->num, thisModule->index);
        if(!thisModule->error) {
            thisModule->error = true;
            ES_Event newEvent = {I2C_Error, thisModule->num};
            PostI2C(newEvent);
            if(thisModule->state == Sending) {
                //printf("Stop Tx\r\n");
                I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            } else if(thisModule->state == Receiving) {
                //printf("Stop Rx\r\n");
                I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
            }
        }
    }
    // If we are sending data
    else if(thisModule->state == Sending) {
        //printf("\tTx\r\n");
        if(thisModule->index < thisModule->commandSize-1) {
            I2CMasterDataPut(thisModule->base, thisModule->command[thisModule->index]);
            I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_SEND_CONT);
        } else if(thisModule->index == thisModule->commandSize-1) {
            I2CMasterDataPut(thisModule->base, thisModule->command[thisModule->index]);
            I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        } else {
            ES_Event newEvent = {I2C_TxComplete, thisModule->num};
            PostI2C(newEvent);
            //printf("Tx Complete \r\n");
        }
    }
    // If we are receiving data
    else if(thisModule->state == Receiving) {
        //printf("\tRx\r\n");
        if(thisModule->index < thisModule->responseSize-1) {
            thisModule->response[thisModule->index] = I2CMasterDataGet(thisModule->base);
            I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        } else if(thisModule->index == thisModule->responseSize-1) {
            thisModule->response[thisModule->index] = I2CMasterDataGet(thisModule->base);
            I2CMasterControl(thisModule->base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            ES_Event newEvent = {I2C_RxComplete, thisModule->num};
            PostI2C(newEvent);
            //printf("Rx Complete \r\n");
        }
    }
    thisModule->index++;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
    beginTx
****************************************************************************/
static void beginTx(I2CModuleInfo_t* thisModule) {
    //printf("Starting Tx\r\n");
    //printModule(thisModule);
    
    // Place the first command byte
    I2CMasterDataPut(thisModule->base, thisModule->command[0]);
    
    // Set index to 1 since we've already loaded byte 0
    thisModule->index = 1; 
    
    // Modifiy the data direction to false, I2C Master initiating a write
    I2CMasterSlaveAddrSet(thisModule->base, thisModule->slaveAddress, false);
    
    // Start the transfer
    uint32_t sendType;
    if(thisModule->commandSize == 1) {
        sendType = I2C_MASTER_CMD_SINGLE_SEND;
    }else { 
        sendType = I2C_MASTER_CMD_BURST_SEND_START;
    }
    I2CMasterControl(thisModule->base, sendType);
}

/****************************************************************************
 Function
    beginRx
****************************************************************************/
static void beginRx(I2CModuleInfo_t* thisModule) {
    //printf("Starting Rx\r\n");
    //printModule(thisModule);
    
    // Reset index to 0 so first byte will be saved there
    thisModule->index = 0; 
    
    // Modifiy the data direction to false, I2C Master initiating a read
    I2CMasterSlaveAddrSet(thisModule->base, thisModule->slaveAddress, true);
    
    // Start the transfer
    uint32_t responseType;
    if(thisModule->responseSize == 1) {
        responseType = I2C_MASTER_CMD_SINGLE_RECEIVE;
    }
    else {
        responseType = I2C_MASTER_CMD_BURST_RECEIVE_START;
    }
    I2CMasterControl(thisModule->base, responseType);
}

/****************************************************************************
 Function
    InitI2CHardware
****************************************************************************/
static void InitI2CHardware(void) {    
    // Enable I2C module 0 and 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
 
    // Reset I2C module 0 and 1
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
     
    // Enable GPIO peripheral that contains I2C0 (Port B) and I2C1 (Port A)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
 
    // Configure the pin muxing for I2C0 functions on port B2, B3, A6, A7.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
 
    // Enable and initialize the I2C0 and I2C1 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
    
    // Set a timeout for transmission. Timer is 12-bit (20ms timeout = 0x7D TivaWare p.338)
    I2CMasterTimeoutSet(I2C0_BASE, 0x7D ); 
    I2CMasterTimeoutSet(I2C1_BASE, 0x7D ); 
    
    // Clear I2C FIFOs
    I2CTxFIFOFlush(I2C0_BASE);
    I2CRxFIFOFlush(I2C0_BASE);
    I2CTxFIFOFlush(I2C1_BASE);
    I2CRxFIFOFlush(I2C1_BASE);
    
    // Enable interrupts for I2C0
    // First register interrupt response
    I2CIntRegister(I2C0_BASE, I2C0InterruptResponse);
    I2CIntRegister(I2C1_BASE, I2C1InterruptResponse);
    
    // Enable local interrupts for I2C0 and the master
    IntEnable(INT_I2C0);
    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C1);
    I2CMasterIntEnable(I2C1_BASE);
    
    // Finally enable interrupts globaly
    IntMasterEnable();
    __enable_irq( );
}
