/****************************************************************************
 Module
     I2C.h
 Description
     header file for the I2C functions
*****************************************************************************/

#ifndef I2C_H
#define I2C_H

// Function Prototypes
bool InitI2C ( uint8_t Priority );
ES_Event RunI2C( ES_Event ThisEvent );
bool PostI2C( ES_Event ThisEvent );

double getYaw( void );
double getHeading( void );

//bool Check4Keystroke(void);


#endif /* I2C_H */
