// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// Tabs PIR motion sensor -- port of the TrackNet tabs-pir application

#include "lmic.h"
#include "lwmux/lwmux.h"

#include "tabs/tabs.h"
#include "eefs/eefs.h"
#include "uexti/uexti.h"
#include "svcdefs.h"


// ------------------------------------------------
// Sensor parameters

// _______s sssssssb bbbppww_ __f_____
#define PIR_PARAMS(s,b,p,w,f)   ((((s) & 0xff) << 17) | (((b) & 0xf) << 13) | (((p) & 0x3) << 11) | (((w) & 0x3) << 9) | (((f) & 0x1) << 5))
#define PIR_PARAMS_MASK         PIR_PARAMS(255, 15, 3, 3, 1)

#define PIR_PARAM_SENSE(p)      (((p) >> 17) & 0xff)
#define PIR_PARAM_BLINDTIME(p)  (((p) >> 13) & 0xf)
#define PIR_PARAM_PULSECNTR(p)  (((p) >> 11) & 0x3)
#define PIR_PARAM_WINDOWTIME(p) (((p) >> 9) & 0x3)
#define PIR_PARAM_FILTER(p)     (((p) >> 5) & 0x1)

#define PIR_MODE_WAKEUP         (2 << 7)    // operation mode (wake-up)
#define PIR_RESERVED            (2 << 3)    // reserved bits, must be 2


// ------------------------------------------------
// Persistent configuration

typedef struct {
    uint32_t port;      // LoRa port
    uint32_t i_minsend; // minimum send interval (sec, 0 for off)
    uint32_t i_occsend; // send interval during occupancy (sec, 0 for off)
    uint32_t t_free;    // "free" detection threshold (min, 0 for off)
    uint32_t t_cntsend; // counter send threshold (0 for off)
    uint32_t params;    // sensor parameters (see PIR_PARAMS)
} pir_cfg;

// c17acc81a38856b0-e7fa6a66
static const uint8_t UFID_PIR_CONFIG[12] = { 0xc1, 0x7a, 0xcc, 0x81, 0xa3, 0x88, 0x56, 0xb0, 0xe7, 0xfa, 0x6a, 0x66 };

static pir_cfg cfg;

const char* pir_eefs_fn (const uint8_t* ufid) {
    if( memcmp(ufid, UFID_PIR_CONFIG, sizeof(UFID_PIR_CONFIG)) == 0 ) {
        return "io.tabs.pircfg";
    }
    return NULL;
}

static void cfg_defaults (pir_cfg* c) {
    c->port      = TABS_PORT_MOTION;
    c->i_minsend = 60 * 60;                     // 1 hr
    c->i_occsend = 10 * 60;                     // 10 min
    c->t_free    = 5;                           // 5 min
    c->t_cntsend = 0;                           // disable counter-based uplink trigger
    c->params    = PIR_PARAMS(16, 15, 0, 0, 0); // 16 sensitivity (medium), 8s blind time, 1 pulse, 4s window time, BPF
}

static void cfg_print (void) {
    debug_printf("pir: port=%d i_minsend=%d i_occsend=%d t_free=%d t_cntsend=%d\r\n",
            cfg.port, cfg.i_minsend, cfg.i_occsend, cfg.t_free, cfg.t_cntsend);
    debug_printf("pir: params=%08x (sensitivity=%d, blindtime=%d.%cs, pulsecounter=%d, windowtime=%ds, filter=%s)\r\n",
            cfg.params,
            PIR_PARAM_SENSE(cfg.params),
            (PIR_PARAM_BLINDTIME(cfg.params) + 1) / 2, (PIR_PARAM_BLINDTIME(cfg.params) & 1) ? '0' : '5',
            PIR_PARAM_PULSECNTR(cfg.params) + 1,
            (PIR_PARAM_WINDOWTIME(cfg.params) + 1) * 4,
            PIR_PARAM_FILTER(cfg.params) ? "lpf" : "bpf");
}

void pir_cfg_load (void) {
    if( eefs_read(UFID_PIR_CONFIG, &cfg, sizeof(pir_cfg)) != sizeof(pir_cfg) ) {
        cfg_defaults(&cfg);
    }
    eefs_save(UFID_PIR_CONFIG, &cfg, sizeof(pir_cfg));
    cfg_print();
}


// ------------------------------------------------
// Battery / temperature

static void bvt_encode (unsigned char* buf) {
    tabs_pwrman_info pwrinfo;
    tabs_pwrman_get(&pwrinfo);
    buf[0] = tabs_enc_batt(pwrinfo.vbat_8dv, pwrinfo.cap_lvl, TABS_PWRMAN_POWERED(pwrinfo.state));
    buf[1] = tabs_enc_temp(pwrinfo.temp_c);
}


// ------------------------------------------------
// PIR sensor

enum {
    FLAG_OCCUPIED       = (1 << 0),
    FLAG_CHANGED        = (1 << 1), // state has changed (free <-> occupied)
    FLAG_LEDTEST        = (1 << 2), // show activity detection using LED
    FLAG_RECONFIG       = (1 << 3), // reconfigure sensor
    FLAG_CONFIG         = (1 << 4), // sensor configuration in progress
};

static struct {
    uint32_t flags;
    // updated by interrupt handler
    volatile bool motion;       // motion detected since last check
    volatile uint32_t total;    // total events counted since last reboot
    volatile uint32_t last;     // time of last event (in minutes)
    osxtime_t last_xtime;       // time of last event (access with IRQs disabled)
    osjob_t job;
} pir;

static void uplink_rewind (void);

static void pir_func (osjob_t* job);

static void pir_confdone (osjob_t* job) {
    pir.flags &= ~FLAG_CONFIG;
    // arm the interrupt
    pio_set(GPIO_PIR_DLINK, PIO_INP_HIZ);
    uexti_config(GPIO_PIR_DLINK, true, false);
    uexti_enable(GPIO_PIR_DLINK, true);
    pir_func(job);
}

static void pir_func (osjob_t* job) {
    if( pir.flags & FLAG_CONFIG ) {
        // will be called again when done
        return;
    }
    if( pir.flags & FLAG_RECONFIG ) {
        uexti_enable(GPIO_PIR_DLINK, false);
        pir.flags = (pir.flags & ~FLAG_RECONFIG) | FLAG_CONFIG;
        tabs_pir_config((cfg.params & PIR_PARAMS_MASK) | PIR_MODE_WAKEUP | PIR_RESERVED,
                &pir.job, pir_confdone);
        return;
    }

    if( pir.motion ) {
        pir.motion = false;
#if defined(SVC_tabs_led)
        if( pir.flags & FLAG_LEDTEST ) {
            // LED indication for duration of blind time
            tabs_led(TABS_LED_BLUE, ms2osticks((PIR_PARAM_BLINDTIME(cfg.params) + 1) * 500), 0, 1, NULL);
        }
#endif
    }

    unsigned int occupied = 0;
    if( cfg.t_free && pir.total ) { // occupied/free detection on
        hal_disableIRQs();
        osxtime_t t_free = pir.last_xtime + sec2osxticks(60 * cfg.t_free);
        hal_enableIRQs();
        if( os_getXTime() < t_free ) {
            occupied = FLAG_OCCUPIED;
            // reschedule ourselves for free timeout
            os_setApproxTimedCallback(&pir.job, (ostime_t) t_free, pir_func);
        }
    }

    if( (pir.flags & FLAG_OCCUPIED) ^ occupied ) {
        pir.flags = (pir.flags & ~FLAG_OCCUPIED) | FLAG_CHANGED | occupied;
        debug_printf("pir: %s\r\n", occupied ? "occupied" : "free");
    }

    uplink_rewind();
}

static void pir_init (void) {
    // initialize I/O lines
    tabs_pir_init();
    // turn on sensor board
    pio_set(GPIO_SSW, 1);
    // wait a little for the sensor to power up, then configure it
    pir.flags |= FLAG_RECONFIG;
    os_setTimedCallback(&pir.job, os_getTime() + ms2osticks(200), pir_func);
}

void pir_irq (unsigned int* mask) {
    unsigned int line = 1 << PIO_IRQ_LINE(GPIO_PIR_DLINK);
    if( *mask & line ) {
        // drive low to reset
        pio_set(GPIO_PIR_DLINK, 0);
        pio_set(GPIO_PIR_DLINK, PIO_INP_HIZ);

        // save time stamp and increment counter
        pir.last = tabs_minutes();
        pir.last_xtime = os_getXTime();
        pir.total += 1;
        pir.motion = true;

        os_setCallback(&pir.job, pir_func);
        *mask &= ~line;
    }
}


// ------------------------------------------------
// Configuration downlink

// NOTE: do not change tag values (append only!)
enum {
    TAG_I_MINSEND_SEC   = 0,    // len=2 (sec, up to ~18hr)
    TAG_I_MINSEND_HR    = 1,    // len=1 (hr, up to ~10days)
    TAG_I_OCCSEND       = 2,    // len=2 (sec, up to ~18hr)
    TAG_T_FREE          = 3,    // len=1 (min, up to ~4hr)
    TAG_T_CNTSEND       = 4,    // len=2 (0..65535)
    TAG_PIRPARAMS       = 5,    // len=4 (encoded according to PIR_PARAMS() macro)
    TAG_TESTMODE        = 6,    // len=1 (boolean)
};

static bool cfg_tx (lwm_txinfo* txinfo) {
    unsigned char* buf = txinfo->data;
    if( cfg.i_minsend > 0xffff ) {
        tabs_cfg_put_tagvalue(TAG_I_MINSEND_HR, 1, cfg.i_minsend / (60 * 60), &buf);
    } else {
        tabs_cfg_put_tagvalue(TAG_I_MINSEND_SEC, 2, cfg.i_minsend, &buf);
    }
    tabs_cfg_put_tagvalue(TAG_I_OCCSEND, 2, cfg.i_occsend, &buf);
    tabs_cfg_put_tagvalue(TAG_T_FREE, 1, cfg.t_free, &buf);
    tabs_cfg_put_tagvalue(TAG_T_CNTSEND, 2, cfg.t_cntsend, &buf);
    tabs_cfg_put_tagvalue(TAG_PIRPARAMS, 4, cfg.params, &buf);
    tabs_cfg_put_tagvalue(TABS_CFGTAG_PORT, 1, cfg.port, &buf);
    int dlen = buf - txinfo->data;
    if( dlen > txinfo->dlen ) {
        debug_printf("pir: config report too long for current data rate (%d > %d)\r\n", dlen, txinfo->dlen);
        return false;
    }
    txinfo->dlen = dlen;
    txinfo->port = TABS_PORT_CONFIG;
    return true;
}

static void cfg_process (unsigned char* buf, int n) {
    static lwm_job lj;
    pir_cfg ncfg = cfg;
    unsigned char* end = buf + n;
    uint32_t v;

    while( buf < end ) {
        switch( *buf++ ) {
            case TAG_I_MINSEND_SEC:
                if( !tabs_cfg_getlsbf(&v, 2, &buf, end) || v < 15 ) {
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
            case TAG_I_OCCSEND:
                if( !tabs_cfg_getlsbf(&v, 2, &buf, end) ) {
                    goto error;
                }
                ncfg.i_occsend = v;
                continue;
            case TAG_T_FREE:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                ncfg.t_free = v;
                continue;
            case TAG_T_CNTSEND:
                if( !tabs_cfg_getlsbf(&v, 2, &buf, end) ) {
                    goto error;
                }
                ncfg.t_cntsend = v;
                continue;
            case TAG_PIRPARAMS:
                if( !tabs_cfg_getlsbf(&v, 4, &buf, end) ) {
                    goto error;
                }
                ncfg.params = (v & PIR_PARAMS_MASK);
                continue;
            case TAG_TESTMODE:
                if( !tabs_cfg_getlsbf(&v, 1, &buf, end) ) {
                    goto error;
                }
                if( v ) {
                    pir.flags |= FLAG_LEDTEST;
                } else {
                    pir.flags &= ~FLAG_LEDTEST;
                }
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
    if( ncfg.i_minsend == 0 && ncfg.i_occsend == 0 && ncfg.t_cntsend == 0 && ncfg.t_free == 0 ) {
        goto error;
    }

    if( ((cfg.params ^ ncfg.params) & PIR_PARAMS_MASK) != 0 ) {
        pir.flags |= FLAG_RECONFIG;
    }

    cfg = ncfg;
    eefs_save(UFID_PIR_CONFIG, &cfg, sizeof(pir_cfg));
    cfg_print();

    os_setCallback(&pir.job, pir_func);

error:
    // report current configuration
    lwm_request_send(&lj, 0, cfg_tx);
}

void app_dl (int port, unsigned char* data, int dlen, unsigned int flags) {
    debug_printf("DL[%d]: %h\r\n", port, data, dlen);
    switch( port ) {
        case TABS_PORT_MOTION:
            // legacy sensor parameter downlink (0x01 + params)
            if( dlen == 5 && data[0] == 1 ) {
                unsigned char buf[5];
                buf[0] = TAG_PIRPARAMS;
                memcpy(buf + 1, data + 1, 4);
                cfg_process(buf, sizeof(buf));
            }
            break;
        case TABS_PORT_CONFIG:
            cfg_process(data, dlen);
            break;
    }
}


// ------------------------------------------------
// Uplink

static struct {
    osxjob_t xjob;
    uint32_t total;     // event count at last counter-triggered uplink
    osxtime_t xtime;    // time of last uplink (0: none yet)
} uplink;

static void uplink_schedule (osjob_t* job);

static void uplink_done (void) {
#if defined(SVC_tabs_led)
    tabs_led(TABS_LED_BLUE, ms2osticks(100), ms2osticks(100), 3, NULL);
#endif
    uplink_rewind();
}

static bool uplink_tx (lwm_txinfo* txinfo) {
    debug_printf("reporting %s, total=%d\r\n", (pir.flags & FLAG_OCCUPIED) ? "OCCUPIED" : "FREE", pir.total);

    txinfo->data[0] = (pir.flags & FLAG_OCCUPIED) ? TABS_PL_STATUS_BTN0 : 0;
    bvt_encode(txinfo->data + 1);
    os_wlsbf2(txinfo->data + 3, tabs_minutes() - pir.last);
    os_wlsbf3(txinfo->data + 5, pir.total);
    txinfo->dlen = 8;
    txinfo->port = cfg.port;
    txinfo->txcomplete = uplink_done;

    pir.flags &= ~FLAG_CHANGED;
    uplink.xtime = os_getXTime();
    return true;
}

static void uplink_schedule (osjob_t* job) {
    static lwm_job lj;
    lwm_request_send(&lj, 0, uplink_tx);
}

// based on current rules, check when next uplink should occur
static void uplink_rewind (void) {
    osxtime_t next;
    if( cfg.t_cntsend && (pir.total - uplink.total) > cfg.t_cntsend ) {
        uplink.total = pir.total; // only update total if we triggered on that
        goto now;
    } else if( uplink.xtime == 0 || (cfg.t_free && (pir.flags & FLAG_CHANGED)) ) {
        goto now;
    } else if( cfg.i_occsend && (pir.flags & FLAG_OCCUPIED) ) {
        next = uplink.xtime + sec2osxticks(cfg.i_occsend);
    } else if( cfg.i_minsend ) {
        next = uplink.xtime + sec2osxticks(cfg.i_minsend);
    } else {
        os_clearCallback(&uplink.xjob.job);
        return;
    }
    if( next <= os_getXTime() ) {
now:
        os_clearCallback(&uplink.xjob.job);
        uplink_schedule(NULL);
    } else {
        os_setExtendedTimedCallback(&uplink.xjob, next, uplink_schedule);
    }
}


// ------------------------------------------------
// Application entry point

bool app_main (osjob_t* job) {
    // refresh battery/temperature measurements every 2 hours; 10 min if battery is low
    tabs_pwrman_bvt_interval(sec2osticks(2 * 60 * 60), sec2osticks(10 * 60));

    // initialize sensor
    pir_init();

    // start joining
    lwm_setmode(LWM_MODE_NORMAL);

    // indicate we are running
    return true;
}
