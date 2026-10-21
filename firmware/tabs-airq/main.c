// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// Tabs air quality sensor -- port of the TrackNet tabs-airq application
//
// The sensor board carries an SHT20 (temperature/humidity) and, on even
// hardware revisions, a CCS811 (eCO2/TVOC). Both are sampled every minute; an
// uplink is sent when a value changed by more than its threshold, or when the
// minimum send interval has elapsed.

#include <stdlib.h>

#include "lmic.h"
#include "lwmux/lwmux.h"
#include "pwrman/pwrman.h"

#include "tabs/tabs.h"
#include "eefs/eefs.h"
#include "svcdefs.h"


// ------------------------------------------------
// Persistent configuration

typedef struct {
    uint32_t port;      // LoRa port
    uint32_t i_minsend; // minimum send interval (sec, 0 for off)
    uint32_t t_dtemp;   // temperature delta threshold (abs. °C, 0 for off)
    uint32_t t_dhumi;   // humidity delta threshold (abs. %RH, 0 for off)
    uint32_t t_dco2;    // eCO2 delta threshold (rel. %, 0 for off)
} airq_cfg;

// 1a15b01840e647e0-53e7dbb7
static const uint8_t UFID_AIRQ_CONFIG[12] = { 0xe0, 0x47, 0xe6, 0x40, 0x18, 0xb0, 0x15, 0x1a, 0xb7, 0xdb, 0xe7, 0x53 };

static airq_cfg cfg;

const char* airq_eefs_fn (const uint8_t* ufid) {
    if( memcmp(ufid, UFID_AIRQ_CONFIG, sizeof(UFID_AIRQ_CONFIG)) == 0 ) {
        return "io.tabs.airqcfg";
    }
    return NULL;
}

static void cfg_defaults (airq_cfg* c) {
    c->port      = TABS_PORT_AIRQ;
    c->i_minsend = 60 * 60;     // 1 hr
    c->t_dtemp   = 2;           // 2 °C
    c->t_dhumi   = 5;           // 5 %RH
    c->t_dco2    = 10;          // 10 %
}

static void cfg_print (void) {
    debug_printf("airq: port=%d i_minsend=%d t_dtemp=%d t_dhumi=%d t_dco2=%d\r\n",
            cfg.port, cfg.i_minsend, cfg.t_dtemp, cfg.t_dhumi, cfg.t_dco2);
}

void airq_cfg_load (void) {
    if( eefs_read(UFID_AIRQ_CONFIG, &cfg, sizeof(airq_cfg)) != sizeof(airq_cfg) ) {
        cfg_defaults(&cfg);
    }
    eefs_save(UFID_AIRQ_CONFIG, &cfg, sizeof(airq_cfg));
    cfg_print();
}


// ------------------------------------------------
// Sensors

// odd hardware revisions do not have a VOC sensor
#define HAVE_VOC()      ((hal_hwid() & 1) == 0)

#define SENSE_INTV      sec2osticks(60)     // sense cycle (matches CCS811 drive mode)
#define VOC_MAX_TRIES   5                   // read attempts (1s apart) until CCS811 data is ready
#define VOC_MAX_ERRORS  5                   // power-cycle sensor board after this many consecutive errors

#define VOC_NONE        0xffff              // CO2/VOC value if sensor is absent or failed

enum {
    FLAG_VOC_NOINIT     = (1 << 0),     // VOC sensor initialization failed
    FLAG_VOC_ERROR      = (1 << 1),     // last VOC measurement failed
    FLAG_TEMP_ERROR     = (1 << 2),     // last temperature/humidity measurement failed
};

typedef struct {
    tabs_sht20_data th;
    tabs_ccs811_data aq;
} airq_data;

static struct {
    uint32_t flags;
    int status;
    int voc_tries;
    int voc_errors;
    ostime_t next_sense;
    ostime_t last_accu;         // VOC sensor consumption accounted up to here
    airq_data now;              // current values
    airq_data last;             // values of last uplink
    osjob_t job;
} airq;

static void check_send (void);

static void sensor_power_off (osjob_t* job);
static void sense_start (osjob_t* job);

static void sense_done (osjob_t* job) {
    if( airq.voc_errors > VOC_MAX_ERRORS ) {
        debug_printf("ccs811: too many errors, resetting sensor board\r\n");
        os_setApproxTimedCallback(job, os_getTime() + sec2osticks(10), sensor_power_off);
    } else {
        os_setApproxTimedCallback(job, airq.next_sense, sense_start);
    }
    check_send();
}

static void voc_fail (osjob_t* job) {
    airq.now.aq.co2 = VOC_NONE;
    airq.now.aq.voc = VOC_NONE;
    airq.flags |= FLAG_VOC_ERROR;
    sense_done(job);
}

static void voc_read (osjob_t* job);

static void voc_read_done (osjob_t* job) {
    switch( airq.status ) {
        case TABS_CCS811_OK: {
            int adj = VOC_MAX_TRIES - airq.voc_tries;
            debug_printf("ccs811: %d ppm CO2, %d ppb VOC\r\n", airq.now.aq.co2, airq.now.aq.voc);
            if( adj ) {
                // shift sense cycle so that data is ready on first attempt
                debug_printf("ccs811: adjusting sense cycle by %d s\r\n", adj);
                airq.next_sense += sec2osticks(adj);
            }
            airq.flags &= ~FLAG_VOC_ERROR;
            airq.voc_errors = 0;
            sense_done(job);
            break;
        }
        case TABS_CCS811_NOTREADY:
            airq.voc_tries -= 1;
            debug_printf("ccs811: not ready (%d tries remaining)\r\n", airq.voc_tries);
            if( airq.voc_tries ) {
                os_setApproxTimedCallback(job, os_getTime() + sec2osticks(1), voc_read);
            } else {
                airq.now.aq.co2 = 0;
                airq.now.aq.voc = 0;
                airq.flags |= FLAG_VOC_ERROR;
                sense_done(job);
            }
            break;
        default:
            debug_printf("ccs811: error\r\n");
            airq.voc_errors += 1;
            voc_fail(job);
            break;
    }
}

static void voc_read (osjob_t* job) {
    int rh = (airq.flags & FLAG_TEMP_ERROR) ? -1 : airq.now.th.rh;
    tabs_ccs811_read(airq.now.th.temp, rh, &airq.now.aq, &airq.status, job, voc_read_done);
}

static void temp_done (osjob_t* job) {
    if( airq.status == TABS_SHT20_OK ) {
        airq.flags &= ~FLAG_TEMP_ERROR;
        debug_printf("sht20: %d C, %d %%RH\r\n", airq.now.th.temp, airq.now.th.rh);
    } else {
        airq.flags |= FLAG_TEMP_ERROR;
        debug_printf("sht20: error\r\n");
    }

    if( !HAVE_VOC() ) {
        voc_fail(job);
        return;
    }

    // account VOC sensor consumption
    ostime_t now = os_getTime();
    pwrman_consume(PWRMAN_C_APP1, now - airq.last_accu, BRD_PWR_CCS811_UA);
    airq.last_accu = now;

    if( airq.flags & FLAG_VOC_NOINIT ) {
        airq.voc_errors += 1;
        voc_fail(job);
        return;
    }
    airq.voc_tries = VOC_MAX_TRIES;
    voc_read(job);
}

static void sense_start (osjob_t* job) {
    airq.next_sense += SENSE_INTV;
    tabs_sht20_read(&airq.now.th, &airq.status, job, temp_done);
}

static void voc_init_done (osjob_t* job) {
    if( airq.status != TABS_CCS811_OK ) {
        debug_printf("ccs811: init failed\r\n");
        airq.flags |= FLAG_VOC_NOINIT;
    }
    airq.voc_errors = 0;
    airq.next_sense = airq.last_accu = os_getTime();
    sense_start(job);
}

static void voc_init (osjob_t* job) {
    tabs_ccs811_init(&airq.status, job, voc_init_done);
}

static void sensor_power_on (osjob_t* job) {
    pio_set(GPIO_SSW, 1);
    airq.flags &= ~FLAG_VOC_NOINIT;
    if( HAVE_VOC() ) {
        // wait 100ms until sensors are ready
        os_setTimedCallback(job, os_getTime() + ms2osticks(100), voc_init);
    } else {
        debug_printf("ccs811: not present\r\n");
        airq.status = TABS_CCS811_OK;
        os_setTimedCallback(job, os_getTime() + ms2osticks(100), voc_init_done);
    }
}

static void sensor_power_off (osjob_t* job) {
    // turn off sensor board -- ensure clean slate
    pio_set(GPIO_SSW, 0);
    os_setTimedCallback(job, os_getTime() + ms2osticks(500), sensor_power_on);
}


// ------------------------------------------------
// Configuration downlink

// NOTE: do not change tag values (append only!)
enum {
    TAG_I_MINSEND_SEC   = 0,    // len=2 (sec, up to ~18hr)
    TAG_I_MINSEND_HR    = 1,    // len=1 (hr, up to ~10days)
    TAG_T_DTEMP         = 2,    // len=1 (°C)
    TAG_T_DHUMI         = 3,    // len=1 (%RH)
    TAG_T_DCO2          = 4,    // len=1 (%)
};

static bool cfg_tx (lwm_txinfo* txinfo) {
    unsigned char* buf = txinfo->data;
    if( cfg.i_minsend > 0xffff ) {
        tabs_cfg_put_tagvalue(TAG_I_MINSEND_HR, 1, cfg.i_minsend / (60 * 60), &buf);
    } else {
        tabs_cfg_put_tagvalue(TAG_I_MINSEND_SEC, 2, cfg.i_minsend, &buf);
    }
    tabs_cfg_put_tagvalue(TAG_T_DTEMP, 1, cfg.t_dtemp, &buf);
    tabs_cfg_put_tagvalue(TAG_T_DHUMI, 1, cfg.t_dhumi, &buf);
    tabs_cfg_put_tagvalue(TAG_T_DCO2, 1, cfg.t_dco2, &buf);
    tabs_cfg_put_tagvalue(TABS_CFGTAG_PORT, 1, cfg.port, &buf);
    int dlen = buf - txinfo->data;
    if( dlen > txinfo->dlen ) {
        debug_printf("airq: config report too long for current data rate (%d > %d)\r\n", dlen, txinfo->dlen);
        return false;
    }
    txinfo->dlen = dlen;
    txinfo->port = TABS_PORT_CONFIG;
    return true;
}

static void cfg_process (unsigned char* buf, int n) {
    static lwm_job lj;
    airq_cfg ncfg = cfg;
    unsigned char* end = buf + n;
    uint32_t v;

    while( buf < end ) {
        switch( *buf++ ) {
            case TAG_I_MINSEND_SEC:
                if( !tabs_cfg_getlsbf(&v, 2, &buf, end) || (v && v < 60) ) {
                    goto error;
                }
                ncfg.i_minsend = v;
                continue;
            case TAG_I_MINSEND_HR:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                ncfg.i_minsend = (60 * 60) * v;
                continue;
            case TAG_T_DTEMP:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                ncfg.t_dtemp = v;
                continue;
            case TAG_T_DHUMI:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                ncfg.t_dhumi = v;
                continue;
            case TAG_T_DCO2:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                ncfg.t_dco2 = v;
                continue;
            case TABS_CFGTAG_PORT:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) || v < 1 || v > 223 ) {
                    goto error;
                }
                ncfg.port = v;
                continue;
            case TABS_CFGTAG_DEFAULTS:
                cfg_defaults(&ncfg);
                continue;
            default:
                goto error;
        }
    }
    if( ncfg.i_minsend == 0 && ncfg.t_dtemp == 0 && ncfg.t_dhumi == 0 && ncfg.t_dco2 == 0 ) {
        goto error;
    }

    cfg = ncfg;
    eefs_save(UFID_AIRQ_CONFIG, &cfg, sizeof(airq_cfg));
    cfg_print();

error:
    // report current configuration
    lwm_request_send(&lj, 0, cfg_tx);
}

void app_dl (int port, unsigned char* data, int dlen, unsigned int flags) {
    debug_printf("DL[%d]: %h\r\n", port, data, dlen);
    if( port == TABS_PORT_CONFIG ) {
        cfg_process(data, dlen);
    }
}


// ------------------------------------------------
// Uplink

static struct {
    lwm_job lj;
    osxtime_t xtime;    // time of last uplink request (0: none yet)
} uplink;

static void uplink_done (void) {
#if defined(SVC_tabs_led)
    tabs_led(TABS_LED_BLUE, ms2osticks(100), ms2osticks(100), 2, NULL);
#endif
}

static bool uplink_tx (lwm_txinfo* txinfo) {
    tabs_pwrman_info pwrinfo;
    tabs_pwrman_get(&pwrinfo);

    unsigned char status = 0;
    if( airq.flags & FLAG_TEMP_ERROR ) {
        status |= TABS_PL_STATUS_FAULT0;
    }
    if( airq.flags & (FLAG_VOC_NOINIT | FLAG_VOC_ERROR) ) {
        status |= TABS_PL_STATUS_FAULT1;
    }

    txinfo->data[0] = status;
    txinfo->data[1] = tabs_enc_batt(pwrinfo.vbat_8dv, pwrinfo.cap_lvl, TABS_PWRMAN_POWERED(pwrinfo.state));
    txinfo->data[2] = tabs_enc_temp(airq.now.th.temp);
    txinfo->data[3] = airq.now.th.rh;
    os_wlsbf2(txinfo->data + 4, airq.now.aq.co2);
    os_wlsbf2(txinfo->data + 6, airq.now.aq.voc);
    txinfo->dlen = 8;
    txinfo->port = cfg.port;
    txinfo->txcomplete = uplink_done;

    airq.last = airq.now;
    return true;
}

static bool check_co2 (unsigned int now, unsigned int last) {
    if( now == 0 || last == 0 ) {
        return now != last;
    }
    unsigned int delta = (now > last) ? (now - last) : (last - now);
    return (delta * 100) / last > cfg.t_dco2;
}

static void check_send (void) {
    osxtime_t now = os_getXTime();
    if( uplink.xtime == 0
            || (cfg.i_minsend && now - uplink.xtime >= sec2osxticks(cfg.i_minsend))
            || (cfg.t_dtemp && abs(airq.now.th.temp - airq.last.th.temp) >= cfg.t_dtemp)
            || (cfg.t_dhumi && abs(airq.now.th.rh - airq.last.th.rh) >= cfg.t_dhumi)
            || (cfg.t_dco2 && check_co2(airq.now.aq.co2, airq.last.aq.co2)) ) {
        uplink.xtime = now;
        lwm_request_send(&uplink.lj, 0, uplink_tx);
    }
}


// ------------------------------------------------
// Application entry point

bool app_main (osjob_t* job) {
    // refresh battery/temperature measurements every 2 hours; 10 min if battery is low
    tabs_pwrman_bvt_interval(sec2osticks(2 * 60 * 60), sec2osticks(10 * 60));

    // power up sensor board and start sense cycle
    os_setCallback(&airq.job, sensor_power_off);

    // start joining
    lwm_setmode(LWM_MODE_NORMAL);

    // indicate we are running
    return true;
}
