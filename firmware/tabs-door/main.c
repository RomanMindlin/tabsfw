// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// Tabs door/window sensor -- port of the TrackNet tabs-door application

#include "lmic.h"
#include "lwmux/lwmux.h"

#include "tabs/tabs.h"
#include "eefs/eefs.h"
#include "uexti/uexti.h"
#include "svcdefs.h"


// ------------------------------------------------
// Persistent configuration

typedef struct {
    uint32_t interval;  // minimum send interval (sec)
} door_cfg;

// 2f45b720ebf8f03b-69ae81cf
static const uint8_t UFID_DOOR_CONFIG[12] = { 0x3b, 0xf0, 0xf8, 0xeb, 0x20, 0xb7, 0x45, 0x2f, 0xcf, 0x81, 0xae, 0x69 };

static door_cfg cfg;

const char* door_eefs_fn (const uint8_t* ufid) {
    if( memcmp(ufid, UFID_DOOR_CONFIG, sizeof(UFID_DOOR_CONFIG)) == 0 ) {
        return "io.tabs.doorcfg";
    }
    return NULL;
}

void door_cfg_load (void) {
    if( eefs_read(UFID_DOOR_CONFIG, &cfg, sizeof(door_cfg)) != sizeof(door_cfg) ) {
        cfg.interval = 360 * 60;
    }
    eefs_save(UFID_DOOR_CONFIG, &cfg, sizeof(door_cfg));
}


// ------------------------------------------------
// Battery / temperature

#if defined(SVC_tabs_pwrman)

// use tabs power manager (capacity from consumption accounting)

static void bvt_init (void) {
    // refresh measurements every 2 hours; 10 min if battery is low
    tabs_pwrman_bvt_interval(sec2osticks(2 * 60 * 60), sec2osticks(10 * 60));
}

static void bvt_encode (unsigned char* buf) {
    tabs_pwrman_info pwrinfo;
    tabs_pwrman_get(&pwrinfo);
    buf[0] = tabs_enc_batt(pwrinfo.vbat_8dv, pwrinfo.cap_lvl, TABS_PWRMAN_POWERED(pwrinfo.state));
    buf[1] = tabs_enc_temp(pwrinfo.temp_c);
}

#else

// standalone measurements (capacity estimated from voltage)

static struct {
    uint32_t vbat;      // battery voltage (0.125dV)
    int32_t temp;       // temperature (°C)
    bool low;           // battery voltage is low
    ostime_t last;      // time of last measurement
    osjob_t job;
} bvt;

static void bvt_start (osjob_t* job);

static void bvt_done (osjob_t* job) {
    tabs_bvt_get(&bvt.vbat, &bvt.temp);
    bvt.low = (bvt.vbat <= TABS_DV8(BRD_BATT_LOW));
    debug_printf("batt: %F V (%s)\r\ntemp: %d C\r\n",
            (bvt.vbat * 100) / 8, 3, bvt.low ? "low" : "ok", bvt.temp);
    // refresh every 2 hours; 10 min if battery is low
    os_setApproxTimedCallback(&bvt.job,
            bvt.last + (bvt.low ? sec2osticks(10 * 60) : sec2osticks(2 * 60 * 60)),
            bvt_start);
}

static void bvt_start (osjob_t* job) {
    bvt.last = os_getTime();
    tabs_bvt_read(job, bvt_done);
}

// Estimate remaining capacity level (0-15) from battery voltage. The TrackNet
// firmware used a charge counter here; lacking that, scale the voltage over the
// same 2.6-3.9V range used for the voltage field.
static uint32_t bvt_level (void) {
    if( bvt.low ) {
        return 0;
    }
    uint32_t v = tabs_enc_batt(bvt.vbat, 0, false); // 1..14
    return ((v - 1) * 15) / 13;
}

static void bvt_init (void) {
    bvt_start(&bvt.job);
}

static void bvt_encode (unsigned char* buf) {
    buf[0] = tabs_enc_batt(bvt.vbat, bvt_level(), false);
    buf[1] = tabs_enc_temp(bvt.temp);
}

#endif


// ------------------------------------------------
// Door/window switch

static struct {
    bool open;          // current sensor state (open/closed)
    uint32_t total;     // total events counted since last reboot
    uint32_t last;      // time of last event (in minutes)
    unsigned int gpio;  // sensor input (reed switch or hall sensor)
    osjob_t* job;       // pointer to job (reusing init job!)
} state;

static void uplink_schedule (osjob_t* job);

static void door_update (void) {
    bool open = pio_get(state.gpio);
    if( state.open == open ) {
        // state didn't change -- missed an event
        state.total += 2;
    } else {
        state.open = open;
        state.total += 1;
    }
    state.last = tabs_minutes();
    debug_printf("%s total=%d\r\n", state.open ? "OPEN" : "CLOSED", state.total);
    uplink_schedule(NULL);
}

static void door_debounce (osjob_t* job) {
    door_update();
}

// initialize the sensor
static void door_init (osjob_t* job) {
    static int step;
    switch( step ) {
        case 0:
            if( hal_pd_valid() && (hal_hwid() & 0xff) == 0 ) {
                // rev 0 has reed switch
                state.gpio = GPIO_REED;
            } else {
                // rev 1+ has hall sensor, which needs power
                state.gpio = GPIO_HALL;
                pio_set(GPIO_SSW, 1);
            }
            // configure the sensor input pin
            pio_set(state.gpio, PIO_INP_HIZ);
            // wait a little for the sensor to settle
            os_setTimedCallback(job, os_getTime() + ms2osticks(500), door_init);
            break;
        case 1:
            // read current state
            state.open = pio_get(state.gpio);
            state.last = tabs_minutes();
            debug_printf("%s (%s)\r\n", state.open ? "OPEN" : "CLOSED",
                    (state.gpio == GPIO_REED) ? "reed" : "hall");
            // arm the interrupt
            uexti_config(state.gpio, true, true);
            uexti_enable(state.gpio, true);
            // trigger first uplink
            uplink_schedule(NULL);
            break;
        default:
            ASSERT(0);
    }
    step += 1;
}

void door_irq (unsigned int* mask) {
    static osjob_t job;
    unsigned int line = 1 << PIO_IRQ_LINE(state.gpio);
    if( state.gpio != 0 && (*mask & line) ) {
        os_setTimedCallback(&job, os_getTime() + ms2osticks(5), door_debounce);
        *mask &= ~line;
    }
}


// ------------------------------------------------
// Uplink

static void uplink_done (void) {
#if defined(SVC_tabs_led)
    tabs_led(TABS_LED_BLUE, ms2osticks(100), ms2osticks(100), 3, NULL);
#endif
    // send heartbeat if nothing happens until then
    os_setApproxTimedCallback(state.job, os_getTime() + sec2osticks(cfg.interval), uplink_schedule);
}

static bool uplink_tx (lwm_txinfo* txinfo) {
    debug_printf("reporting %s, total=%d\r\n", state.open ? "OPEN" : "CLOSED", state.total);

    txinfo->data[0] = state.open ? TABS_PL_STATUS_BTN0 : 0;
    bvt_encode(txinfo->data + 1);
    os_wlsbf2(txinfo->data + 3, tabs_minutes() - state.last);
    os_wlsbf3(txinfo->data + 5, state.total);
    txinfo->dlen = 8;
    txinfo->port = TABS_PORT_DOORWINDOW;
    txinfo->txcomplete = uplink_done;
    return true;
}

static void uplink_schedule (osjob_t* job) {
    static lwm_job lj;
    lwm_request_send(&lj, 0, uplink_tx);
}

void app_dl (int port, unsigned char* data, int dlen, unsigned int flags) {
    debug_printf("DL[%d]: %h\r\n", port, data, dlen);
}


// ------------------------------------------------
// Application entry point

bool app_main (osjob_t* job) {
    // reuse job for heartbeat uplinks
    state.job = job;

    // measure battery and temperature
    bvt_init();

    // initialize sensor
    door_init(job);

    // start joining
    lwm_setmode(LWM_MODE_NORMAL);

    // indicate we are running
    return true;
}
