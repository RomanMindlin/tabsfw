// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// Battery/temperature sensor (service tabs_bvt)

#include "lmic.h"
#include "tabs/tabs.h"

#if !defined(CFG_wsms130_board)
#error "Battery/temperature sensor not implemented for this board"
#endif

// logical shift right with rounding
#define LSR_ROUND(v,a) (((v) + (1 << ((a) - 1))) >> (a))

static struct {
    int samples;
    uint32_t vbat;
    uint32_t temp;
    osjobcb_t cb;
} bvt;

// calculate battery voltage -- unit: 0.125dV
static uint32_t bvt_calc_vbat (uint32_t a_vbat) {
    // Vin = (a * ((3 * (cal / 10) / 2^9) * 100RR / ref     (0.125dV)
    //
    // RR = (R1 + R2) / R2, with R1 = 107k, R2 = 100k
    const uint32_t _100RR = 207;

    // read reference voltage and calibration value
    uint32_t a_vref = adc_read(VREFINT_ADC_CH, 7);
    uint32_t cal10 = *VREFINT_CAL_ADDR / 10;

    return (LSR_ROUND(a_vbat * 3 * cal10, 9) * _100RR) / a_vref;
}

// NTC look-up table
//   - Rntc = 20700*(Vbatt/Vtemp)-10000
//   - scl = 20700.000000
//   - off = -10000.000000
//   - sft = 12
//   - NTC = (100000, 4250)
static const uint32_t NTC_LOOKUP[127] = {
    /*  94.5C */ 0x00000cf4, /*  93.5C */ 0x00000d1f, /*  92.5C */ 0x00000d4b, /*  91.5C */ 0x00000d7a,
    /*  90.5C */ 0x00000da9, /*  89.5C */ 0x00000ddb, /*  88.5C */ 0x00000e0f, /*  87.5C */ 0x00000e44,
    /*  86.5C */ 0x00000e7c, /*  85.5C */ 0x00000eb6, /*  84.5C */ 0x00000ef2, /*  83.5C */ 0x00000f31,
    /*  82.5C */ 0x00000f72, /*  81.5C */ 0x00000fb6, /*  80.5C */ 0x00000ffc, /*  79.5C */ 0x00001045,
    /*  78.5C */ 0x00001091, /*  77.5C */ 0x000010e1, /*  76.5C */ 0x00001133, /*  75.5C */ 0x00001189,
    /*  74.5C */ 0x000011e3, /*  73.5C */ 0x00001240, /*  72.5C */ 0x000012a2, /*  71.5C */ 0x00001307,
    /*  70.5C */ 0x00001371, /*  69.5C */ 0x000013df, /*  68.5C */ 0x00001452, /*  67.5C */ 0x000014c9,
    /*  66.5C */ 0x00001546, /*  65.5C */ 0x000015c9, /*  64.5C */ 0x00001651, /*  63.5C */ 0x000016df,
    /*  62.5C */ 0x00001774, /*  61.5C */ 0x0000180f, /*  60.5C */ 0x000018b1, /*  59.5C */ 0x0000195b,
    /*  58.5C */ 0x00001a0c, /*  57.5C */ 0x00001ac5, /*  56.5C */ 0x00001b87, /*  55.5C */ 0x00001c52,
    /*  54.5C */ 0x00001d26, /*  53.5C */ 0x00001e04, /*  52.5C */ 0x00001eed, /*  51.5C */ 0x00001fe0,
    /*  50.5C */ 0x000020df, /*  49.5C */ 0x000021eb, /*  48.5C */ 0x00002303, /*  47.5C */ 0x00002429,
    /*  46.5C */ 0x0000255d, /*  45.5C */ 0x000026a0, /*  44.5C */ 0x000027f3, /*  43.5C */ 0x00002957,
    /*  42.5C */ 0x00002acd, /*  41.5C */ 0x00002c56, /*  40.5C */ 0x00002df2, /*  39.5C */ 0x00002fa3,
    /*  38.5C */ 0x0000316b, /*  37.5C */ 0x0000334a, /*  36.5C */ 0x00003541, /*  35.5C */ 0x00003753,
    /*  34.5C */ 0x00003981, /*  33.5C */ 0x00003bcc, /*  32.5C */ 0x00003e36, /*  31.5C */ 0x000040c2,
    /*  30.5C */ 0x00004370, /*  29.5C */ 0x00004643, /*  28.5C */ 0x0000493e, /*  27.5C */ 0x00004c63,
    /*  26.5C */ 0x00004fb4, /*  25.5C */ 0x00005334, /*  24.5C */ 0x000056e6, /*  23.5C */ 0x00005ace,
    /*  22.5C */ 0x00005eed, /*  21.5C */ 0x0000634a, /*  20.5C */ 0x000067e6, /*  19.5C */ 0x00006cc6,
    /*  18.5C */ 0x000071ee, /*  17.5C */ 0x00007764, /*  16.5C */ 0x00007d2c, /*  15.5C */ 0x0000834c,
    /*  14.5C */ 0x000089c9, /*  13.5C */ 0x000090aa, /*  12.5C */ 0x000097f6, /*  11.5C */ 0x00009fb3,
    /*  10.5C */ 0x0000a7ea, /*   9.5C */ 0x0000b0a2, /*   8.5C */ 0x0000b9e5, /*   7.5C */ 0x0000c3bd,
    /*   6.5C */ 0x0000ce33, /*   5.5C */ 0x0000d953, /*   4.5C */ 0x0000e529, /*   3.5C */ 0x0000f1c2,
    /*   2.5C */ 0x0000ff2b, /*   1.5C */ 0x00010d75, /*   0.5C */ 0x00011caf, /*  -0.5C */ 0x00012ceb,
    /*  -1.5C */ 0x00013e3b, /*  -2.5C */ 0x000150b6, /*  -3.5C */ 0x00016470, /*  -4.5C */ 0x00017982,
    /*  -5.5C */ 0x00019006, /*  -6.5C */ 0x0001a818, /*  -7.5C */ 0x0001c1d6, /*  -8.5C */ 0x0001dd63,
    /*  -9.5C */ 0x0001fae2, /* -10.5C */ 0x00021a7a, /* -11.5C */ 0x00023c56, /* -12.5C */ 0x000260a4,
    /* -13.5C */ 0x00028796, /* -14.5C */ 0x0002b163, /* -15.5C */ 0x0002de47, /* -16.5C */ 0x00030e82,
    /* -17.5C */ 0x0003425a, /* -18.5C */ 0x00037a1e, /* -19.5C */ 0x0003b621, /* -20.5C */ 0x0003f6bf,
    /* -21.5C */ 0x00043c5d, /* -22.5C */ 0x00048767, /* -23.5C */ 0x0004d857, /* -24.5C */ 0x00052fb0,
    /* -25.5C */ 0x00058e01, /* -26.5C */ 0x0005f3e9, /* -27.5C */ 0x00066215, /* -28.5C */ 0x0006d942,
    /* -29.5C */ 0x00075a41, /* -30.5C */ 0x0007e5f7, /* -31.5C */ 0x00087d61, };

// calculate temperature -- unit: °C
static int32_t bvt_calc_temp (uint32_t a_temp, uint32_t a_vbat) {
    if( a_temp == 0 ) {
        return 95; // NTC shorted -- report maximum
    }
    uint32_t ratio = (a_vbat << 12) / a_temp;
    int i;
    for( i = 0; i < 127 && NTC_LOOKUP[i] < ratio; i++ ); // binary search would be faster
    return 95 - i;
}

// sample vbat and temp (50 times every 10ms)
static void bvt_sample (osjob_t* job) {
    if( bvt.samples < 50 ) {
        // read ADC values (12 bit)
        bvt.vbat += adc_read(BRD_ADC_VBAT, 7);
        bvt.temp += adc_read(BRD_ADC_TEMP, 7);
        bvt.samples += 1;
        // rewind sampling job
        os_setTimedCallback(job, os_getTime() + ms2osticks(10), bvt_sample);
    } else {
        // turn off voltage dividers
        SET_PIN(GPIO_PSW, 0);
        CFG_PIN_DEFAULT(GPIO_PSW);
        // calculate values
        uint32_t vbat = bvt.vbat / bvt.samples;
        bvt.vbat = bvt_calc_vbat(vbat);
        uint32_t vtmp = bvt.temp / bvt.samples;
        bvt.temp = bvt_calc_temp(vtmp, vbat);
        // invoke application callback
        os_setCallback(job, bvt.cb);
    }
}

void tabs_bvt_read (osjob_t* job, osjobcb_t cb) {
    // turn on voltage dividers
    CFG_PIN_VAL(GPIO_PSW, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE, 1);
    // initialize state
    bvt.samples = 0;
    bvt.vbat = 0;
    bvt.temp = 0;
    bvt.cb = cb;
    // wait for voltage to stabilize, then start sampling
    os_setTimedCallback(job, os_getTime() + ms2osticks(20), bvt_sample);
}

void tabs_bvt_get (uint32_t* vbat, int32_t* temp) {
    if( vbat ) {
        *vbat = bvt.vbat;
    }
    if( temp ) {
        *temp = bvt.temp;
    }
}
