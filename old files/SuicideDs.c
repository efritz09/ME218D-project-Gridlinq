/****************************************************************************
 Module
  SuicideDs.c

 Revision     Revised by: 
  0.1.1       Eric
  0.1.2       Vikram

 Description
  State machine for suicide doors

 Edits:
  0.1.1 - Preliminary setup
  0.1.2 - Added second set of sevos

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
#include "SuicideDs.h"
#include "SpectreXbee.h"
#include "PWM.h"

// Module Defines
#define PWM_WIDTH_CLOSE_L 600
#define PWM_WIDTH_OPEN_L 1000
#define PWM_WIDTH_OUT_L 1200
#define PWM_WIDTH_IN_L 1650

#define PWM_WIDTH_IN_R 2100 //(in)
#define PWM_WIDTH_OUT_R 2400 //(out)
#define PWM_WIDTH_OPEN_R 1900 // open
#define PWM_WIDTH_CLOSE_R 2350 // close

#define SUICIDE_PWM_CHL_IO 2
#define SUICIDE_PWM_CHL_OC 3
#define SUICIDE_PWM_CHR_IO 0
#define SUICIDE_PWM_CHR_OC 1

void closeSuicideDoors(void){
  SetPWMWidth(PWM_WIDTH_CLOSE_L, SUICIDE_PWM_CHL_OC);
  SetPWMWidth(PWM_WIDTH_IN_L, SUICIDE_PWM_CHL_IO);
  
  SetPWMWidth(PWM_WIDTH_CLOSE_R, SUICIDE_PWM_CHR_OC);
  SetPWMWidth(PWM_WIDTH_IN_R, SUICIDE_PWM_CHR_IO);
}

void openSuicideDoors(void){
  //SetPWMWidth(PWM_WIDTH_OUT_L, SUICIDE_PWM_CHL_IO);
  SetPWMWidth(PWM_WIDTH_OPEN_L, SUICIDE_PWM_CHL_OC);
    
  //SetPWMWidth(PWM_WIDTH_OUT_R, SUICIDE_PWM_CHR_IO);
  SetPWMWidth(PWM_WIDTH_OPEN_R, SUICIDE_PWM_CHR_OC);
}

/*------------------------------ End of file ------------------------------*/
