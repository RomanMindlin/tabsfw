// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _tabs_h_
#define _tabs_h_

#include <stdint.h>

#include "lmic.h"

// ------------------------------------------------
// Standard payloads

enum {
    TABS_PL_STATUS_BTN0         = (1 << 0),
    TABS_PL_STATUS_BTN1         = (1 << 1),

    TABS_PL_STATUS_FAULT2       = (1 << 2),
    TABS_PL_STATUS_FAULT1       = (1 << 3),
    TABS_PL_STATUS_FAULT0       = (1 << 4),
};

enum {
    TABS_PORT_CONFIG            = 204,  /* 0xcc */

    TABS_PORT_DOORWINDOW        = 100,
    TABS_PORT_MOTION            = 102,
    TABS_PORT_AIRQ              = 103,
    TABS_PORT_POSITION          = 136,
    TABS_PORT_BUTTON            = 147,
};

// encode battery voltage (0.125dV) and capacity level (0-15)
unsigned char tabs_enc_batt (uint32_t vbat, uint32_t cap, bool powered);
// encode temperature (°C)
unsigned char tabs_enc_temp (int32_t temp);


// ------------------------------------------------
// Shared convenience functions

uint32_t tabs_minutes (void);

// convert volts to 0.125dV
#define TABS_DV8(v) ((uint32_t) ((v) * 80.0 + 0.5))


#if defined(SVC_tabs_bvt)
// ------------------------------------------------
// Battery/Temperature sensor (service tabs_bvt)

// start measurement, cb is run on job when done
void tabs_bvt_read (osjob_t* job, osjobcb_t cb);
// get last measurement -- vbat unit: 0.125dV, temp unit: °C
void tabs_bvt_get (uint32_t* vbat, int32_t* temp);

#endif


#if defined(SVC_tabs_pwrman)
// ------------------------------------------------
// Power manager (service tabs_pwrman)
//  - periodic battery/temperature measurements
//  - remaining capacity, based on consumption accounted by service pwrman
//  - low battery and battery replacement detection

enum {
    TABS_PWR_BATTERY,
    TABS_PWR_POWERED,
};

typedef struct {
    unsigned int state; // TABS_PWR state
    uint32_t vbat_8dv;  // voltage in 0.125dV
    int32_t temp_c;     // temp in C
    uint32_t cap_uah;   // remaining capacity in uAh
    uint32_t cap_lvl;   // remaining capacity level (0-15)
} tabs_pwrman_info;

#define TABS_PWRMAN_POWERED(state) ((state) >= TABS_PWR_POWERED)

// set measurement interval (normal, low battery); 0 disables measurements
void tabs_pwrman_bvt_interval (ostime_t intv, ostime_t intv_low);
// get current power info (and update battery level reported to network)
void tabs_pwrman_get (tabs_pwrman_info* info);

#endif


#if defined(SVC_tabs_led)
// ------------------------------------------------
// LED blink job (service tabs_led)

enum {
#if defined(BRD_LED_BLUE)
    TABS_LED_BLUE,
#endif
    TABS_LED_COUNT
};

typedef void (*tabs_led_done) (unsigned int led);

// blink LED count times (0: switch on if t_on is non-zero, off otherwise)
void tabs_led (unsigned int led, ostime_t t_on, ostime_t t_off, unsigned int count, tabs_led_done done);

#endif

#endif
