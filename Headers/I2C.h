/****************************************************************************
 Module
    I2C.h
 Description
    Header file for master I2C communication
 Notes
    To use this master communication module all a service needs to do is 
    set a callback function for the module to call when it is done (the 
    service's post function or any function with the same prototype will do). 
    Then call I2C Start passing the address of an I2C Message (defined below).
 ****************************************************************************/
#ifndef I2C_H
#define I2C_H
/*----------------------------- Include Files -----------------------------*/

/*----------------------------- Module Defines ----------------------------*/
typedef struct {
    uint8_t module;
    uint8_t slaveAddress;
    uint8_t* command; // Pointer to front of array
    uint8_t commandSize;
    uint8_t* response; // Pointer to front of array
    uint8_t responseSize;
} I2CMessage_t;

typedef bool (*postFuncDef)(ES_Event);

/*----------------------- Public Function Prototypes ----------------------*/
/*Public functions for the framework*/
bool InitI2C(uint8_t Priority);
ES_Event RunI2C(ES_Event ThisEvent);
bool PostI2C(ES_Event ThisEvent);

/*Public functions for other services to use*/
// Bool if the specified module is busy
bool I2CBusy(uint8_t module);

// Set a pointer to the post function for the module controller
// This must be done first before I2C Start is ever called!
void setI2CPostFunction(uint8_t module, postFuncDef postFunc);

// Write a message to the specified slave on the specifies module
// To only write data to slave: set responseSize = 0
// To only read data from slave: set commandSize = 0
// Write data to slave and then read response: Set appropriate sizes
bool I2CStart(I2CMessage_t* message);

#endif /* I2C_H */
