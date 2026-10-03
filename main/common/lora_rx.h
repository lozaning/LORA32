#pragma once
#include "modem_dsp.h"
#include "lora_codec.h"
typedef void (*lora_packet_fn)(const uint8_t *,const lora7_header_t *);
typedef struct {
    uint32_t input,samples,ffts,syncs,headers,packets,header_fail,crc_fail,discontinuities;
    uint32_t fft_max,block_max,fft_cycles,block_cycles;
    uint16_t quality;
    int16_t timing,cfo;
} lora_rx_stats_t;
/* Experimental SF7/BW1.25MHz receiver. Input is 4MS/s complex baseband
 * from the existing native mixer, with remaining +500kHz carrier. */
void lora_rx_start(lora_packet_fn);
void lora_rx_block(const modem_iq_t *,unsigned);
void lora_rx_discontinuity(void);
lora_rx_stats_t lora_rx_stats(void);
