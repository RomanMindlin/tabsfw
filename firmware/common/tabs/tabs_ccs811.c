// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// CCS811 air quality sensor (service tabs_ccs811)

#include "lmic.h"
#include "peripherals.h"
#include "tabs/tabs.h"

#if !defined(BRD_I2C) || !defined(GPIO_CCS811_WAKE)
#error "Tabs CCS811 service requires BRD_I2C, GPIO_I2C_SCL, GPIO_I2C_SDA and GPIO_CCS811_WAKE to be set in board file"
#endif

#define CCS811_I2C_ADDR         (0x5b << 1)
#define CCS811_REG_STATUS       0x00
#define CCS811_REG_MEAS_MODE    0x01
#define CCS811_REG_ALG_RESULT   0x02
#define CCS811_REG_ENV_DATA     0x05
#define CCS811_REG_HW_ID        0x20
#define CCS811_REG_APP_START    0xf4

#define CCS811_HW_ID            0x81

#define CCS811_ST_ERROR         (1 << 0)
#define CCS811_ST_DATA_READY    (1 << 3)
#define CCS811_ST_APP_VALID     (1 << 4)
#define CCS811_ST_FW_MODE       (1 << 7)

// status bits that must be set (and error clear) when application is running
#define CCS811_ST_MASK          (CCS811_ST_FW_MODE | CCS811_ST_APP_VALID | CCS811_ST_ERROR)
#define CCS811_ST_RUNNING       (CCS811_ST_FW_MODE | CCS811_ST_APP_VALID)

#define CCS811_MODE_DRIVE3      (3 << 4)    // pulse heating mode, measurement every 60s

#define I2C_TIMEOUT             ms2osticks(500)

static struct {
    unsigned char buf[5];
    int i2c;                    // I2C status
    int temp;                   // compensation values
    int rh;
    tabs_ccs811_data* data;
    int* pstatus;
    osjob_t* job;
    osjobcb_t cb;
} ccs;

static void begin (int* pstatus, osjob_t* job, osjobcb_t cb) {
    ccs.pstatus = pstatus;
    ccs.job = job;
    ccs.cb = cb;
    // drive nWAKE low
    pio_set(GPIO_CCS811_WAKE, 0);
}

static void done (int status) {
    // release nWAKE (external pull-up)
    pio_default(GPIO_CCS811_WAKE);
    *ccs.pstatus = status;
    os_setCallback(ccs.job, ccs.cb);
}

static void xfer (unsigned int wlen, unsigned int rlen, osjobcb_t next) {
    i2c_xfer_ex(CCS811_I2C_ADDR, ccs.buf, wlen, rlen, I2C_TIMEOUT, ccs.job, next, &ccs.i2c);
}

static void read_reg (unsigned char reg, osjobcb_t next) {
    ccs.buf[0] = reg;
    xfer(1, 1, next);
}

static bool running (void) {
    return ccs.i2c == I2C_OK && (ccs.buf[0] & CCS811_ST_MASK) == CCS811_ST_RUNNING;
}


// ------------------------------------------------
// Initialization

static void init_check (osjob_t* job) {
    done((ccs.i2c == I2C_OK && ccs.buf[0] == CCS811_MODE_DRIVE3) ? TABS_CCS811_OK : TABS_CCS811_ERROR);
}

static void init_verify (osjob_t* job) {
    read_reg(CCS811_REG_MEAS_MODE, init_check);
}

static void init_wait (osjob_t* job) {
    if( ccs.i2c != I2C_OK ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    os_setTimedCallback(job, os_getTime() + ms2osticks(500), init_verify);
}

static void init_mode (osjob_t* job) {
    if( !running() ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    ccs.buf[0] = CCS811_REG_MEAS_MODE;
    ccs.buf[1] = CCS811_MODE_DRIVE3;
    xfer(2, 0, init_wait);
}

static void init_status (osjob_t* job) {
    if( ccs.i2c != I2C_OK ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    read_reg(CCS811_REG_STATUS, init_mode);
}

static void init_start (osjob_t* job) {
    if( ccs.i2c != I2C_OK || ccs.buf[0] != CCS811_HW_ID ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    // switch from boot to application mode
    ccs.buf[0] = CCS811_REG_APP_START;
    xfer(1, 0, init_status);
}

void tabs_ccs811_init (int* pstatus, osjob_t* job, osjobcb_t cb) {
    begin(pstatus, job, cb);
    read_reg(CCS811_REG_HW_ID, init_start);
}


// ------------------------------------------------
// Measurement

static void read_done (osjob_t* job) {
    if( ccs.i2c != I2C_OK ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    unsigned int co2 = (ccs.buf[0] << 8) | ccs.buf[1];
    unsigned int voc = (ccs.buf[2] << 8) | ccs.buf[3];
    if( co2 == 0 ) {
        done(TABS_CCS811_NOTREADY);
        return;
    }
    ccs.data->co2 = co2;
    ccs.data->voc = voc;
    done(TABS_CCS811_OK);
}

static void read_result (osjob_t* job) {
    if( ccs.i2c != I2C_OK ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    ccs.buf[0] = CCS811_REG_ALG_RESULT;
    xfer(1, 4, read_done);
}

static void read_env (osjob_t* job) {
    if( !running() ) {
        done(TABS_CCS811_ERROR);
        return;
    }
    if( (ccs.buf[0] & CCS811_ST_DATA_READY) == 0 ) {
        done(TABS_CCS811_NOTREADY);
        return;
    }
    // write compensation values (used for next measurement)
    int t = ccs.temp, h = ccs.rh;
    if( h < 0 ) {
        // defaults
        t = 25;
        h = 50;
    } else {
        if( t < -25 ) {
            t = -25;
        } else if( t > 102 ) {
            t = 102;
        }
        if( h > 100 ) {
            h = 100;
        }
    }
    // humidity: 1/512 %RH, temperature: 1/512 °C with 25 °C offset
    ccs.buf[0] = CCS811_REG_ENV_DATA;
    ccs.buf[1] = h << 1;
    ccs.buf[2] = 0;
    ccs.buf[3] = (t + 25) << 1;
    ccs.buf[4] = 0;
    xfer(5, 0, read_result);
}

void tabs_ccs811_read (int temp, int rh, tabs_ccs811_data* data, int* pstatus, osjob_t* job, osjobcb_t cb) {
    ccs.temp = temp;
    ccs.rh = rh;
    ccs.data = data;
    begin(pstatus, job, cb);
    read_reg(CCS811_REG_STATUS, read_env);
}
