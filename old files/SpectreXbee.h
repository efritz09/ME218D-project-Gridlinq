/****************************************************************************
 
  Header file for SpectreXbee.c

 ****************************************************************************/
 
#ifndef SPECTREXBEE_H
#define SPECTREXBEE_H

/*----------------------------- Include Files -----------------------------*/
// set up a global header for the project
// Standard headers
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

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
#include "inc/hw_uart.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "bitdefs.h"

/*----------------------------- Module Defines ----------------------------*/
typedef enum {UnpairedState, PairedState} SpectreState;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitSpectreXbee ( uint8_t Priority );
bool PostSpectreXbee ( ES_Event ThisEvent );
ES_Event RunSpectreXbee ( ES_Event CurrentEvent );
void UART1InterruptResponse( void );
void InputCaptureResponse( void );
void SoftwareReset(void);
uint8_t * GetCommands(void);

#endif /* SPECTREXBEE_H */
