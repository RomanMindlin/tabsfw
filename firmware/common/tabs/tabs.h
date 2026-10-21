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
// Configuration downlinks (tag-length-value, little endian)

// NOTE: do not change tag values (append only!)
enum {
    TABS_CFGTAG_PORT            = 128,  // len=1
    TABS_CFGTAG_DEFAULTS        = 129,  // len=0
    TABS_CFGTAG_FIXED_DR        = 130,  // len=1  7:4 reserved, 3:0 DR 0-14, 15=ADR
    TABS_CFGTAG_SETMAH          = 131,  // len=2  set mAh counter value
};

// append tag and len bytes of value to buffer
void tabs_cfg_put_tagvalue (unsigned char tag, int len, uint32_t val, unsigned char** buf);
// read len bytes value from buffer, return false if buffer is too short
bool tabs_cfg_getlsbf (uint32_t* pval, int len, unsigned char** buf, unsigned char* end);


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


#if defined(SVC_tabs_pir)
// ------------------------------------------------
// PIR motion sensor (service tabs_pir)

// initialize I/O lines
void tabs_pir_init (void);
// write 25-bit configuration word to sensor, cb is run on job when done
void tabs_pir_config (uint32_t config, osjob_t* job, osjobcb_t cb);

#endif


#if defined(SVC_tabs_sht20)
// ------------------------------------------------
// SHT20 temperature/humidity sensor (service tabs_sht20)

enum {
    TABS_SHT20_OK               = 0,
    TABS_SHT20_ERROR            = -1,
};

typedef struct {
    int8_t temp;        // temperature (°C)
    uint8_t rh;         // relative humidity (%)
} tabs_sht20_data;

// measure temperature and humidity, cb is run on job when done (TABS_SHT20 status in *pstatus)
void tabs_sht20_read (tabs_sht20_data* data, int* pstatus, osjob_t* job, osjobcb_t cb);

#endif


#if defined(SVC_tabs_ccs811)
// ------------------------------------------------
// CCS811 air quality sensor (service tabs_ccs811)

enum {
    TABS_CCS811_OK              = 0,
    TABS_CCS811_ERROR           = -1,
    TABS_CCS811_NOTREADY        = -2,   // no new measurement available
};

typedef struct {
    uint16_t co2;       // equivalent CO2 (ppm)
    uint16_t voc;       // total VOC (ppb)
} tabs_ccs811_data;

// start application and measurement (every 60s), cb is run on job when done (TABS_CCS811 status in *pstatus)
void tabs_ccs811_init (int* pstatus, osjob_t* job, osjobcb_t cb);
// write compensation values (°C, %RH; rh < 0: none) and read measurement, cb is run on job when done
void tabs_ccs811_read (int temp, int rh, tabs_ccs811_data* data, int* pstatus, osjob_t* job, osjobcb_t cb);

#endif

#endif
