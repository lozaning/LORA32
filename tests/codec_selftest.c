/* Portable SF7 codec smoke test. GNU GPL-3.0-or-later.
 * The deadbeef symbols are the saved gr-lora reference vector also used by
 * the firmware LORAKAT command; no radio is used by this test. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lora_codec.h"
int main(void) {
    const uint8_t expected[]={29,1,97,125,37,109,1,97,119,51,20,1,22,82,37,58,2,48,12,126,120,100,110,58};
    const uint8_t deadbeef[]={0xde,0xad,0xbe,0xef};
    uint8_t data[255],symbols[LORA7_MAX_SYMBOLS],decoded[255];lora7_header_t h;
    size_t n=lora7_encode(deadbeef,4,4,true,symbols,sizeof(symbols));
    /* The reference has different unused trailing pad nibbles. Both decode
     * to the same CRC-valid bytes; only the header is padding-independent. */
    assert(n==sizeof(expected) && !memcmp(symbols,expected,8));
    assert(lora7_decode(symbols,n,decoded,sizeof(decoded),&h));
    assert(h.length==4 && h.cr==4 && h.crc && !memcmp(decoded,deadbeef,4));
    assert(lora7_decode(expected,sizeof(expected),decoded,sizeof(decoded),&h));
    assert(h.length==4 && h.cr==4 && h.crc && !memcmp(decoded,deadbeef,4));
    assert(!lora7_decode(expected,sizeof(expected)-1,decoded,sizeof(decoded),&h));
    assert(!lora7_decode(expected,sizeof(expected),decoded,3,&h));
    unsigned cases=0;const unsigned lengths[]={1,2,4,17,64,72,254,255};
    for(unsigned i=0;i<sizeof(lengths)/sizeof(lengths[0]);i++) {
        unsigned length=lengths[i];for(unsigned j=0;j<length;j++)data[j]=(j*19+length*7)&255;
        for(unsigned cr=1;cr<=4;cr++)for(unsigned crc=0;crc<=1;crc++) {
            if(crc && length<2)continue;
            n=lora7_encode(data,length,cr,crc,symbols,sizeof(symbols));assert(n);
            assert(lora7_decode(symbols,n,decoded,sizeof(decoded),&h));
            assert(h.length==length && h.cr==cr && h.crc==crc && !memcmp(data,decoded,length));
            assert(!lora7_decode(symbols,n-1,decoded,sizeof(decoded),&h));cases++;
        }
    }
    assert(!lora7_encode(data,4,0,true,symbols,sizeof(symbols)));
    assert(!lora7_encode(data,4,5,true,symbols,sizeof(symbols)));
    printf("PASS: known vector, %u round trips, truncation/capacity/invalid-CR controls\n",cases);
    return 0;
}
