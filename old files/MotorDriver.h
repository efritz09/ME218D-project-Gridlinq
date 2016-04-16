/****************************************************************************
 
  Header file for MotorDriver.c

 ****************************************************************************/

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

/*----------------------------- Module Defines ----------------------------*/
typedef enum {UncontrolledState, ControlledState} MotorState;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitMotorDriver ( uint8_t Priority );
bool PostMotorDriver ( ES_Event ThisEvent );
ES_Event RunMotorDriver ( ES_Event CurrentEvent );

#endif /* MOTORDRIVER.H */
