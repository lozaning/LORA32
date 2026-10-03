/* SF7 LoRa coding port, following gr-lora_sdr algorithms (GPL-3.0-or-later).
 * The reduced first interleaver block includes parity AND trailing zero.
 */
#include "lora_codec.h"
#include <string.h>
static unsigned bit(unsigned x,unsigned k){return (x>>k)&1u;}
static uint8_t white_next(uint8_t x){return (uint8_t)((x<<1)|((x>>7^x>>5^x>>4^x>>3)&1));}
static uint16_t payload_crc(const uint8_t *p,unsigned n){
    uint16_t crc=0;
    for(unsigned j=0;j+2<n;j++){
        crc^=(uint16_t)p[j]<<8;
        for(unsigned b=0;b<8;b++)crc=(crc<<1)^((crc&0x8000)?0x1021:0);
    }
    return crc^p[n-1]^((uint16_t)p[n-2]<<8);
}
static void header_nibbles(unsigned n,unsigned cr,bool crc,uint8_t h[5]){
    unsigned a=n>>4,b=n&15,c=cr*2+crc;
    h[0]=a;h[1]=b;h[2]=c;
    h[3]=bit(a,3)^bit(a,2)^bit(a,1)^bit(a,0);
    unsigned c3=bit(a,3)^bit(b,3)^bit(b,2)^bit(b,1)^bit(c,0);
    unsigned c2=bit(a,2)^bit(b,3)^bit(b,0)^bit(c,3)^bit(c,1);
    unsigned c1=bit(a,1)^bit(b,2)^bit(b,0)^bit(c,2)^bit(c,1)^bit(c,0);
    unsigned c0=bit(a,0)^bit(b,1)^bit(c,3)^bit(c,2)^bit(c,1)^bit(c,0);
    h[4]=c3*8+c2*4+c1*2+c0;
}
static uint8_t hamming(unsigned n,unsigned cr){
    unsigned a=bit(n,0),b=bit(n,1),c=bit(n,2),d=bit(n,3);
    if(cr==1)return a*16+b*8+c*4+d*2+(a^b^c^d);
    return (a*128+b*64+c*32+d*16+(a^b^c)*8+(b^c^d)*4+(a^b^d)*2+(a^c^d))>>(4-cr);
}
static uint8_t decode_lut[4][256];
static bool decode_ready;
static uint8_t build_hamming_entry(unsigned cw,unsigned cr){
    unsigned best=99,count=0,value=0;
    for(unsigned j=0;j<16;j++){
        unsigned distance=__builtin_popcount(cw^hamming(j,cr));
        if(distance<best){best=distance;count=1;value=j;}else if(distance==best)count++;
    }
    if(count!=1 || best>(cr>=3?1u:0u))return 255;
    return value|(best<<4);
}
void lora7_init(void){
    if(decode_ready)return;
    memset(decode_lut,255,sizeof(decode_lut));
    for(unsigned cr=1;cr<=4;cr++)for(unsigned cw=0;cw<(1u<<(4+cr));cw++)decode_lut[cr-1][cw]=build_hamming_entry(cw,cr);
    decode_ready=true;
}
static bool unhamming(unsigned cw,unsigned cr,uint8_t *n,unsigned *corrected){
    if(!decode_ready)lora7_init();
    unsigned value=decode_lut[cr-1][cw];if(value==255)return false;
    *n=value&15;*corrected+=value>>4;return true;
}
static uint8_t gray_out(unsigned x){unsigned b=x;for(unsigned k=1;k<7;k++)b^=x>>k;return (b+1)&127;}
static unsigned gray_in(unsigned s){unsigned x=(s+127)&127;return x^(x>>1);}
static void encode_block(const uint8_t *n,unsigned rows,unsigned cr,bool first,uint8_t *out){
    uint8_t c[7];for(unsigned j=0;j<rows;j++)c[j]=hamming(n[j],cr);
    for(unsigned i=0;i<4+cr;i++){
        unsigned s=0,parity=0;
        for(unsigned j=0;j<rows;j++){
            unsigned k=(i+rows*2-j-1)%rows,b=bit(c[k],3+cr-i);
            s=s*2+b;parity^=b;
        }
        if(first)s=(s*2+parity)*2;
        out[i]=gray_out(s);
    }
}
static bool decode_block(const uint8_t *s,unsigned rows,unsigned cr,uint8_t *n,unsigned *corrected){
    unsigned mapped[8];for(unsigned i=0;i<4+cr;i++){if(s[i]>127)return false;mapped[i]=gray_in(s[i]);}
    for(unsigned k=0;k<rows;k++){
        unsigned cw=0;
        for(unsigned i=0;i<4+cr;i++){
            unsigned j=(i+2*rows-k-1)%rows;cw=cw*2+bit(mapped[i],6-j);
        }
        if(!unhamming(cw,cr,n+k,corrected))return false;
    }
    return true;
}
size_t lora7_encode(const uint8_t *p,size_t n,unsigned cr,bool crc,uint8_t *s,size_t capacity){
    if(!p || !s || !n || n>255 || cr<1 || cr>4 || (crc&&n<2))return 0;
    uint8_t nibs[5+510+4+6]={0};header_nibbles(n,cr,crc,nibs);unsigned used=5;uint8_t white=255;
    for(unsigned j=0;j<n;j++){uint8_t x=p[j]^white;white=white_next(white);nibs[used++]=x&15;nibs[used++]=x>>4;}
    if(crc){unsigned check=payload_crc(p,n);for(unsigned j=0;j<4;j++)nibs[used++]=(check>>(j*4))&15;}
    unsigned blocks=(used-5+6)/7,count=8+blocks*(4+cr);if(count>capacity)return 0;
    encode_block(nibs,5,4,true,s);
    for(unsigned j=0;j<blocks;j++)encode_block(nibs+5+j*7,7,cr,false,s+8+j*(4+cr));
    return count;
}
bool lora7_header(const uint8_t *s,size_t count,lora7_header_t *h){
    if(!s || !h || count<8)return false;
    uint8_t n[5],expected[5];unsigned corrected=0;
    if(!decode_block(s,5,4,n,&corrected))return false;
    unsigned length=n[0]*16+n[1],cr=n[2]>>1;bool crc=n[2]&1;
    if(!length || cr<1 || cr>4 || (crc&&length<2))return false;
    header_nibbles(length,cr,crc,expected);if(memcmp(n,expected,5))return false;
    *h=(lora7_header_t){.length=length,.cr=cr,.crc=crc,.corrected=corrected};return true;
}
bool lora7_decode(const uint8_t *s,size_t count,uint8_t *out,size_t capacity,lora7_header_t *h){
    lora7_header_t header;if(!out || !lora7_header(s,count,&header) || capacity<header.length)return false;
    unsigned need=header.length*2+(header.crc?4:0),blocks=(need+6)/7;
    if(count!=8+blocks*(4+header.cr))return false;
    uint8_t n[510+4+6];
    for(unsigned j=0;j<blocks;j++)if(!decode_block(s+8+j*(4+header.cr),7,header.cr,n+j*7,&header.corrected))return false;
    uint8_t white=255;
    for(unsigned j=0;j<header.length;j++){out[j]=(n[j*2]+n[j*2+1]*16)^white;white=white_next(white);}
    if(header.crc){unsigned check=0;for(unsigned j=0;j<4;j++)check|=(unsigned)n[header.length*2+j]<<(4*j);if(check!=payload_crc(out,header.length))return false;}
    if(h)*h=header;
    return true;
}
