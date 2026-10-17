// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// LED blink job (service tabs_led)

#include "lmic.h"
#include "tabs/tabs.h"

#if !defined(BRD_LED_BLUE)
#error "Tabs LED service requires an LED (BRD_LED_BLUE) to be set in board file"
#endif

static const unsigned int led_gpio[TABS_LED_COUNT] = {
    [TABS_LED_BLUE] = BRD_LED_BLUE,
};

static struct {
    struct {
        ostime_t t_on;
        ostime_t t_off;
        ostime_t t_next;
        int count;
        tabs_led_done done;
    } led[TABS_LED_COUNT];
    osjob_t job;
} led_state;

static void led_func (osjob_t* job) {
    ostime_t now = os_getTime();
    ostime_t next = 0;
    bool again = false;
    for( int i = 0; i < TABS_LED_COUNT; i++ ) {
        if( led_state.led[i].count == 0 ) {
            continue;
        }
        if( (now - led_state.led[i].t_next) >= 0 ) {
            if( led_state.led[i].count < 0 ) {
                // static on/off
                leds_set(led_gpio[i], led_state.led[i].t_on ? 1 : 0);
                led_state.led[i].count = 0;
                goto done;
            }
            int ct = --(led_state.led[i].count);
            leds_set(led_gpio[i], ct & 1);
            if( ct & 1 ) {
                led_state.led[i].t_next = now + led_state.led[i].t_on;
            } else if( ct != 0 ) {
                led_state.led[i].t_next = now + led_state.led[i].t_off;
            } else {
done:
                if( led_state.led[i].done ) {
                    led_state.led[i].done(i);
                }
                continue;
            }
        }
        if( !again || (led_state.led[i].t_next - next) < 0 ) {
            next = led_state.led[i].t_next;
            again = true;
        }
    }
    if( again ) {
        os_setTimedCallback(&led_state.job, next, led_func);
    }
}

void tabs_led (unsigned int led, ostime_t t_on, ostime_t t_off, unsigned int count, tabs_led_done done) {
    ASSERT(led < TABS_LED_COUNT);

    led_state.led[led].t_on = t_on;
    led_state.led[led].t_off = t_off;
    led_state.led[led].t_next = os_getTime();
    led_state.led[led].count = count ? (count * 2) : -1;
    led_state.led[led].done = done;

    os_setCallback(&led_state.job, led_func);
}
