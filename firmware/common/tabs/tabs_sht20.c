// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// SHT20 temperature/humidity sensor (service tabs_sht20)

#include "lmic.h"
#include "peripherals.h"
#include "tabs/tabs.h"

#if !defined(BRD_I2C)
#error "Tabs SHT20 service requires BRD_I2C, GPIO_I2C_SCL and GPIO_I2C_SDA to be set in board file"
#endif

#define SHT20_I2C_ADDR          (0x40 << 1)
#define SHT20_CMD_MEAS_TEMP     0xe3    // hold master mode
#define SHT20_CMD_MEAS_RH       0xe5    // hold master mode

#define I2C_TIMEOUT             ms2osticks(500)

// logical shift right with rounding
#define LSR_ROUND(v,a) (((v) + (1 << ((a) - 1))) >> (a))

static struct {
    unsigned char buf[3];       // msb, lsb (2 status bits), crc
    int i2c;                    // I2C status
    tabs_sht20_data* data;
    int* pstatus;
    osjob_t* job;
    osjobcb_t cb;
} sht;

static void done (int status) {
    *sht.pstatus = status;
    os_setCallback(sht.job, sht.cb);
}

static void measure (unsigned char cmd, osjobcb_t next) {
    sht.buf[0] = cmd;
    i2c_xfer_ex(SHT20_I2C_ADDR, sht.buf, 1, 3, I2C_TIMEOUT, sht.job, next, &sht.i2c);
}

// 14-bit raw measurement value
static int raw_value (void) {
    return (sht.buf[0] << 6) | (sht.buf[1] >> 2);
}

static void rh_done (osjob_t* job) {
    if( sht.i2c != I2C_OK ) {
        done(TABS_SHT20_ERROR);
        return;
    }
    // RH = -6 + 125 * s / 2^14
    int h = LSR_ROUND(raw_value() * 125, 14) - 6;
    if( h < 0 ) {
        h = 0;
    } else if( h > 100 ) {
        h = 100;
    }
    sht.data->rh = h;
    done(TABS_SHT20_OK);
}

static void temp_done (osjob_t* job) {
    if( sht.i2c != I2C_OK ) {
        done(TABS_SHT20_ERROR);
        return;
    }
    // T = -46.85 + 175.72 * s / 2^14
    //   ~ -47 + 44 * s / 2^12
    sht.data->temp = LSR_ROUND(raw_value() * 44, 12) - 47;
    measure(SHT20_CMD_MEAS_RH, rh_done);
}

void tabs_sht20_read (tabs_sht20_data* data, int* pstatus, osjob_t* job, osjobcb_t cb) {
    sht.data = data;
    sht.pstatus = pstatus;
    sht.job = job;
    sht.cb = cb;
    measure(SHT20_CMD_MEAS_TEMP, temp_done);
}
