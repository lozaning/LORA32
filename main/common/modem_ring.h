#pragma once
#include "modem_dsp.h"
typedef struct {
    uint32_t units,pairs,late_max,slice_max,overruns,status,detail,event_drops,bb_overruns,bb_depth_max,decode_max;
} modem_ring_stats_t;
typedef struct {uint32_t bank,first,count,index;} modem_ring_unit_t;
bool modem_ring_init(void);
bool modem_ring_start(uint8_t node,int mix_sign,bool conjugate);
bool modem_ring_running(void);
bool modem_ring_stop(void);
bool modem_ring_decoder_sink(modem_block_fn);
bool modem_ring_decoder_reset(void (*)(void));
bool modem_ring_event(modem_message_t *);
modem_ring_stats_t modem_ring_stats(void);
bool modem_ring_capture(unsigned rate,unsigned units);
modem_ring_unit_t modem_ring_capture_unit(unsigned unit);
const uint32_t *modem_ring_bank(unsigned bank);
