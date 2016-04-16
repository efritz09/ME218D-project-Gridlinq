/****************************************************************************
 
  Header file for SpectreUART.c

 ****************************************************************************/
 
 
#ifndef SPECTREUART_H
#define SPECTREUART_H

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
typedef enum {WaitForPairingState, PairedState} SpectreState;

/*----------------------- Public Function Prototypes ----------------------*/

bool InitSpectre ( uint8_t Priority );
bool PostSpectre ( ES_Event ThisEvent );
ES_Event RunSpectre ( ES_Event CurrentEvent );
void UART1InterruptResponse( void );
void InputCaptureResponse( void );
void sendXbeeData( void ) ;
void printData(void);
uint8_t checkSum(uint8_t chkArray[], uint8_t size);

#endif /* SPECTREUART_H */
