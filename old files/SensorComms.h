/****************************************************************************
 
  Header file for SensorComms.c

 ****************************************************************************/

#ifndef SensorComms_H
#define SensorComms_H

// Function Prototypes
bool InitSensorComms ( uint8_t Priority );
ES_Event RunSensorComms( ES_Event ThisEvent );
bool PostSensorComms( ES_Event ThisEvent );

double getYaw( void );
double getHeading( void );

#endif /* SensorComms_H */
