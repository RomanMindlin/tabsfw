// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// Tabs power manager -- port of the TrackNet tabs power manager
//
// Consumption is accounted by service pwrman (MCU run/sleep via CFG_rtstats,
// radio via antenna switch). The remaining capacity is the battery capacity
// minus what was consumed since the battery was last known to be full (first
// boot or detected battery replacement).

#include "lmic.h"

#include "tabs/tabs.h"
#include "eefs/eefs.h"
#include "pwrman/pwrman.h"

#include "svcdefs.h" // for type-checking hook functions

#ifndef BRD_BATT_MAH
#error "Tabs power manager requires battery capacity (BRD_BATT_MAH) to be set in board file"
#endif

#define CC_BATT_MAX     (BRD_BATT_MAH * 1000)   // uAh

enum {
    FLAG_BVT_ACTIVE     = (1 << 0),     // measurement in progress
    FLAG_BVT_VALID      = (1 << 1),     // measurement available
    FLAG_LOWVBATT       = (1 << 2),     // battery voltage is low
    FLAG_NOBASE         = (1 << 3),     // consumption base not yet set
};

static struct {
    unsigned int flags;
    struct {
        ostime_t last;
        ostime_t intv;
        ostime_t intv_low;
        uint32_t vbat;
        int32_t temp;
        osjob_t job;
    } bvt;
    struct {
        uint32_t base;          // pwrman accumulator value when battery was full
        uint32_t cap_uah;       // remaining capacity
        int lvl;                // remaining capacity level (0-15)
    } cc;
} pm;

// Persistent state (eefs)
typedef struct {
    uint32_t base;      // pwrman accumulator value when battery was full
    uint32_t vbat;      // last battery voltage (0.125dV)
} tabs_pwrman_pstate;

// 05be58bc62339583-bc5616fa
static const uint8_t UFID_TABS_PWRMAN[12] = { 0x83, 0x95, 0x33, 0x62, 0xbc, 0x58, 0xbe, 0x05, 0xfa, 0x16, 0x56, 0xbc };

const char* _tabs_pwrman_eefs_fn (const uint8_t* ufid) {
    if( memcmp(ufid, UFID_TABS_PWRMAN, sizeof(UFID_TABS_PWRMAN)) == 0 ) {
        return "io.tabs.pwrman";
    }
    return NULL;
}

void _tabs_pwrman_init (void) {
    tabs_pwrman_pstate ps;
    if( eefs_read(UFID_TABS_PWRMAN, &ps, sizeof(ps)) == sizeof(ps) ) {
        pm.cc.base = ps.base;
        pm.bvt.vbat = ps.vbat;
        debug_printf("pm: restoring base (%d uAh) and voltage (%F V)\r\n",
                ps.base, (ps.vbat * 100) / 8, 3);
    } else {
        // set on first update (pwrman state might not be loaded yet)
        pm.flags |= FLAG_NOBASE;
    }
    pm.cc.lvl = -1;
}

static void pm_commit (void) {
    tabs_pwrman_pstate ps;
    ps.base = pm.cc.base;
    ps.vbat = pm.bvt.vbat;
    eefs_save(UFID_TABS_PWRMAN, &ps, sizeof(ps));
    pwrman_commit(); // persist consumption accounted by service pwrman
}

// battery is (assumed to be) full now
static void pm_set_full (void) {
    pm.cc.base = pwrman_accu_uah();
    pm.flags &= ~FLAG_NOBASE;
    debug_printf("pm: battery full (base %d uAh)\r\n", pm.cc.base);
}

static void pm_update (void) {
    if( pm.flags & FLAG_NOBASE ) {
        pm_set_full();
        pm_commit();
    }
    uint32_t used = pwrman_accu_uah() - pm.cc.base;
    pm.cc.cap_uah = (used < CC_BATT_MAX) ? (CC_BATT_MAX - used) : 0;
    int lvl;
    if( pm.flags & FLAG_LOWVBATT ) {
        // always report 0 if battery voltage is deemed low
        lvl = 0;
    } else {
        lvl = (pm.cc.cap_uah * 16) / CC_BATT_MAX;
        if( lvl > 15 ) {
            lvl = 15;
        }
    }
    if( pm.cc.lvl != lvl ) {
        pm.cc.lvl = lvl;
        pm_commit();
    }
}

static void pm_bvt_start (osjob_t* job);

static void pm_bvt_schedule (void) {
    ostime_t interval = (pm.flags & FLAG_LOWVBATT) ? pm.bvt.intv_low : pm.bvt.intv;
    if( interval ) {
        os_setApproxTimedCallback(&pm.bvt.job, pm.bvt.last + interval, pm_bvt_start);
    } else {
        os_clearCallback(&pm.bvt.job);
    }
}

static void pm_bvt_done (osjob_t* job) {
    uint32_t vbat;
    int32_t temp;
    tabs_bvt_get(&vbat, &temp);

#ifdef BRD_BATT_REPL
    // battery replacement test
    if( pm.bvt.vbat && ((int32_t) vbat - (int32_t) pm.bvt.vbat) > (int32_t) TABS_DV8(BRD_BATT_REPL) ) {
        debug_printf("pm: battery replacement detected\r\n");
        pm_set_full();
    }
#endif

#ifdef BRD_BATT_LOW
    if( vbat <= TABS_DV8(BRD_BATT_LOW) ) {
        pm.flags |= FLAG_LOWVBATT;
    } else {
        pm.flags &= ~FLAG_LOWVBATT;
    }
#endif

    pm.bvt.vbat = vbat;
    pm.bvt.temp = temp;
    pm.flags = (pm.flags & ~FLAG_BVT_ACTIVE) | FLAG_BVT_VALID;

    debug_printf("batt: %F V (%s)\r\ntemp: %d C\r\n",
            (vbat * 100) / 8, 3, (pm.flags & FLAG_LOWVBATT) ? "low" : "ok", temp);

    // persist voltage (for replacement detection after reset) and consumption
    pm_commit();

    pm_bvt_schedule();
}

static void pm_bvt_start (osjob_t* job) {
    pm.flags |= FLAG_BVT_ACTIVE;
    pm.bvt.last = os_getTime();
    tabs_bvt_read(job, pm_bvt_done);
}

void tabs_pwrman_bvt_interval (ostime_t intv, ostime_t intv_low) {
    pm.bvt.intv = intv;
    pm.bvt.intv_low = intv_low;
    if( (pm.flags & FLAG_BVT_ACTIVE) == 0 ) {
        if( (pm.flags & FLAG_BVT_VALID) == 0 && (intv || intv_low) ) {
            // no measurement yet -- start right away
            pm_bvt_start(&pm.bvt.job);
        } else {
            pm_bvt_schedule();
        }
    }
}

void tabs_pwrman_get (tabs_pwrman_info* info) {
    pm_update();
    if( info ) {
        info->state = TABS_PWR_BATTERY;
        info->vbat_8dv = pm.bvt.vbat;
        info->temp_c = pm.bvt.temp;
        info->cap_uah = pm.cc.cap_uah;
        info->cap_lvl = pm.cc.lvl;
    }
    debug_printf("cap: %d uAh, lvl: %d\r\n", pm.cc.cap_uah, pm.cc.lvl);
    hal_setBattLevel(tabs_enc_batt(pm.bvt.vbat, pm.cc.lvl, false));
}
