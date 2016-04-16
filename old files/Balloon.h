/****************************************************************************
 
  Header file for Balloon.c

 ****************************************************************************/

#ifndef BALLOON_H
#define BALLOON_H

/*----------------------------- Include Files -----------------------------*/
// set up a global header for the project
#include <stdint.h>
#include <stdbool.h>

// ES_framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"
#include "ES_Events.h"
/*----------------------------- Module Defines ----------------------------*/
typedef enum {IdleState, HeatingWireState, ActuateState} PopState;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitBalloon	( uint8_t Priority );
bool PostBalloon ( ES_Event ThisEvent );
ES_Event RunBalloon ( ES_Event CurrentEvent );

#endif /* BALLOON.H */
