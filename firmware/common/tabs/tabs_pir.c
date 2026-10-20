// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// PIR motion sensor (service tabs_pir)

#include "lmic.h"
#include "peripherals.h"
#include "tabs/tabs.h"

#if !defined(GPIO_PIR_SERIN) || !defined(GPIO_PIR_DLINK) || !defined(BRD_PIR_TMR)
#error "Tabs PIR service requires GPIO_PIR_SERIN, GPIO_PIR_DLINK and BRD_PIR_TMR to be set in board file"
#endif

// The PIR has an internal clock frequency (Fclk) at 32kHz
// typical. Often, the datasheet says to not exceed 1/Fclk,
// and recommends 25us (40kHz). 1/Fclk may be as long as
// 35.71us (28kHz).
// A timer is used to drive the communication with the PIR,
// with a freqency of 400kHz, i.e. a period of 2.5us.

#define TMR_PSC         TMR_PSC_MHZ(0.4)
#define MAX_PERIOD      15      // 37.5 us

static struct {
    int n;                      // remaining bits (-1: end of transfer)
    uint32_t config;            // configuration word
    pio_direct serin;           // direct access to SERIN line
    osjob_t* job;
    osjobcb_t cb;
} state;

static void config_tmr (void) {
    if( state.n > 0 ) {
        // L-H
        pio_direct_set0(&state.serin);
        pio_direct_set1(&state.serin);
        // data bit
        pio_direct_set(&state.serin, (state.config >> --state.n) & 1);
        // hold for at least 2 clock cycles
        tmr_run(BRD_PIR_TMR, 2 * MAX_PERIOD, config_tmr, true);
    } else if( state.n == 0 ) {
        // drive line low
        pio_direct_set0(&state.serin);
        // hold for at least 16 clock cycles
        tmr_run(BRD_PIR_TMR, 16 * MAX_PERIOD, config_tmr, true);
        state.n = -1;
    } else {
        tmr_stop(BRD_PIR_TMR);
        pio_direct_stop(&state.serin);
        pio_set(GPIO_PIR_SERIN, PIO_INP_PDN);
        // reset direct link
        pio_set(GPIO_PIR_DLINK, 0);
        pio_default(GPIO_PIR_DLINK);
        os_setCallback(state.job, state.cb);
        state.job = NULL;
    }
}

void tabs_pir_config (uint32_t config, osjob_t* job, osjobcb_t cb) {
    ASSERT(state.job == NULL && job != NULL);
    state.config = config;
    state.job = job;
    state.cb = cb;
    state.n = 25; // 25 bits to transfer

    pio_set(GPIO_PIR_SERIN, 0);
    pio_direct_start(GPIO_PIR_SERIN, &state.serin);
    tmr_start(BRD_PIR_TMR, TMR_PSC);
    tmr_run(BRD_PIR_TMR, 1, config_tmr, true);
}

void tabs_pir_init (void) {
    pio_set(GPIO_PIR_SERIN, PIO_INP_PDN);
}
