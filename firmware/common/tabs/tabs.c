// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"
#include "tabs/tabs.h"

// logical shift right with rounding
#define LSR_ROUND(v,a) (((v) + (1 << ((a) - 1))) >> (a))

uint32_t tabs_minutes (void) {
    return osticks2sec(os_getXTime()) / 60;
}

// encode battery voltage / level according to Tabs payload specification
unsigned char tabs_enc_batt (uint32_t vbat, uint32_t cap, bool powered) {
    if( powered ) {
        return MCMD_DEVS_EXT_POWER;
    }
    // encode battery voltage, range is 1-14  =>  2.6-3.9
    int v = LSR_ROUND(vbat, 3) - 26 + 1; // 2.6V = 26dV
    if( v < 1 ) {
        v = 1; // 0 is reserved for line powered
    } else if( v > 14 ) {
        v = 14; // 15 is reserved for fault
    }
    return (cap << 4) | v;
}

unsigned char tabs_enc_temp (int32_t temp) {
    if( temp < -32 ) {
        temp = -32;
    } else if( temp > 95 ) {
        temp = 95;
    }
    return temp + 32;
}

void tabs_cfg_put_tagvalue (unsigned char tag, int len, uint32_t val, unsigned char** buf) {
    *(*buf)++ = tag;
    while( len-- > 0 ) {
        *(*buf)++ = val;
        val >>= 8;
    }
}

bool tabs_cfg_getlsbf (uint32_t* pval, int len, unsigned char** buf, unsigned char* end) {
    uint32_t v = 0;
    for( int i = 0; i < len; i++ ) {
        if( *buf >= end ) {
            return false;
        }
        v |= (*(*buf)++ << (8 * i));
    }
    *pval = v;
    return true;
}
