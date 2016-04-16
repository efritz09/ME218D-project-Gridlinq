/****************************************************************************
 Module
     CompassCom.h
 Description
     Header file for interface with the MM Blue compass over I2C
 ****************************************************************************/
#ifndef COMPASS_COM_H
#define COMPASS_COM_H
/*----------------------------- Include Files -----------------------------*/

/*----------------------------- Module Defines ----------------------------*/
// Define CompassReading type for storing/reading data
typedef struct {
    int16_t heading;  // Heading, pitch and roll are reported as +/- pi
    int16_t pitch;
    int16_t roll;
} CompassReading_t;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitCompassCom (uint8_t Priority);
ES_Event RunCompassCom(ES_Event ThisEvent);
bool PostCompassCom(ES_Event ThisEvent);

void MM_HostInterruptResponse(void);

CompassReading_t getCompassData(void);

#endif /* COMPASS_COM_H */
