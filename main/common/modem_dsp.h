#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* CSS1 is a diagnostic wire format, not a Semtech LoRa frame. */
#define MODEM_SF 5u
#define MODEM_N 32u
#define MODEM_FRAGMENT 16u
#define MODEM_TEXT_MAX 256u
#define MODEM_FRAME_MAX 26u
#define MODEM_SYMBOLS_MAX 50u
#define MODEM_VERSION 0xc1u
typedef struct { int16_t i,q; } modem_iq_t;
typedef struct {
    uint8_t source,destination,fragment,fragments,length;
    uint16_t sequence;
    uint8_t payload[MODEM_FRAGMENT];
} modem_frame_t;
typedef struct {
    uint8_t source,destination;
    uint16_t sequence,length;
    uint8_t text[MODEM_TEXT_MAX];
} modem_message_t;
typedef void (*modem_message_fn)(const modem_message_t *,void *);
typedef void (*modem_block_fn)(const modem_iq_t *,unsigned);
typedef struct {
    uint32_t samples,symbols,frames,crc_fail,header_fail,syncs,discontinuities;
    uint16_t quality;
    uint32_t fft_cycles;
} modem_dsp_stats_t;

void modem_dsp_init(void);
uint16_t modem_crc16(const uint8_t *,size_t);
size_t modem_frame_encode(const modem_frame_t *,uint8_t symbols[MODEM_SYMBOLS_MAX]);
bool modem_frame_decode(const uint8_t *,size_t,modem_frame_t *);
unsigned modem_symbol_detect(const modem_iq_t input[MODEM_N],uint16_t *quality);
void modem_decoder_start(uint8_t node,modem_message_fn,void *);
void modem_decoder_discontinuity(void);
void modem_front_discontinuity(void);
void modem_native_sink(modem_block_fn);
void modem_decoder_block(const modem_iq_t *,unsigned);
void modem_decoder_sample(int16_t i,int16_t q);
void modem_decoder_native(const uint32_t *words,unsigned first,unsigned count,int mix_sign,bool conjugate);
modem_dsp_stats_t modem_decoder_stats(void);
unsigned modem_front_bench(const uint32_t *,unsigned,unsigned);
void modem_waveform_init(unsigned clock);
unsigned modem_waveform(const uint8_t *symbols,unsigned count,uint32_t *out);
