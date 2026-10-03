#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* SF7 explicit headers, standard interleaver, no LDRO. Coding only. */
#define LORA7_MAX_SYMBOLS 600u
void lora7_init(void);
typedef struct {uint8_t length,cr;bool crc;unsigned corrected;} lora7_header_t;
size_t lora7_encode(const uint8_t *,size_t,unsigned,bool,uint8_t *,size_t);
bool lora7_header(const uint8_t *,size_t,lora7_header_t *);
bool lora7_decode(const uint8_t *,size_t,uint8_t *,size_t,lora7_header_t *);
