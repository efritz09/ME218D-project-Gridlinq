/****************************************************************************
  Header file for PWM.c
 ****************************************************************************/
#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"

#define CLAMP_MOTOR 0
#define IR_BEACON 	2
#define LED	        1
#define BEACON_HACK 3
void InitPWM(void);
void SetPWMDuty(uint8_t duty, int channel);
void SetPWMWidth(uint32_t width, int channel);

#endif
