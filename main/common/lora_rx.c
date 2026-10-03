/* SPDX-License-Identifier: GPL-3.0-or-later
 * Streaming experimental SF7 receiver, validated against saved RF captures.
 * Modem bandwidth differs from SX1280; this is not an interop claim. */
#include "lora_rx.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define N 128u
#define PI 3.14159265358979323846
#ifdef LORA_HOST_TEST
#define HOT
#define CYCLES() 0u
#else
#include "esp_attr.h"
#include "esp_cpu.h"
#define HOT IRAM_ATTR
#define CYCLES() esp_cpu_get_cycle_count()
extern int modem_fft_sc16(int16_t *,int,int16_t *);
extern void modem_dechirp_sc16(const modem_iq_t *,modem_iq_t *,const modem_iq_t *,unsigned);
static inline float inverse(float x){float r,t;__asm__("recip0.s %0,%2\nconst.s %1,1\nmsub.s %1,%2,%0\nmadd.s %0,%0,%1\nconst.s %1,1\nmsub.s %1,%2,%0\nmaddn.s %0,%0,%1":"=&f"(r),"=&f"(t):"f"(x));return r;}
#endif
static modem_iq_t up[N] __attribute__((aligned(16))),down[N] __attribute__((aligned(16)));
static modem_iq_t table[N/2] __attribute__((aligned(16))),window[N] __attribute__((aligned(16))),vector[N] __attribute__((aligned(16)));
static uint8_t reverse[N];
static bool initialized;
static struct {
    modem_iq_t ring[N],previous;
    uint32_t end,next_fft,sync2_start,payload_start;
    int next_fifth;
    unsigned phase,decimation,state,hits,last,up_bin,symbol_count,expected,filled;
    int cfo;
    uint8_t symbols[LORA7_MAX_SYMBOLS],payload[255];
    lora7_header_t header;
    lora_packet_fn emit;
    lora_rx_stats_t stats;
} rx;
HOT static int signed_bin(unsigned b){return (int)((b+64u)&127u)-64;}
HOT static bool near(unsigned a,unsigned b){int d=signed_bin(a-b);return d>=-1 && d<=1;}
static void init(void){
    if(initialized)return;
    for(unsigned k=0;k<N;k++){
        double p=PI*((double)k*k/N-k);up[k]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};down[k]=(modem_iq_t){up[k].i,-up[k].q};
        unsigned r=0;for(unsigned j=0;j<7;j++)r=(r<<1)|((k>>j)&1);reverse[k]=r;
    }
    for(unsigned k=0;k<N/2;k++){double p=2*PI*k/N;table[reverse[k]>>1]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};}
    initialized=true;
}
HOT static unsigned detect(bool is_down,uint16_t *quality){
    unsigned start=CYCLES();const modem_iq_t *chirp=is_down?down:up;
    unsigned first=rx.end&127u;
    for(unsigned k=0;k<N;k++)window[k]=rx.ring[(first+k)&127u];
#ifdef LORA_HOST_TEST
    int32_t vi[N],vq[N];
    for(unsigned k=0;k<N;k++){
        int32_t i=window[k].i,q=window[k].q,c=chirp[k].i,s=chirp[k].q;
        vi[reverse[k]]=(i*c+q*s)>>15;vq[reverse[k]]=(q*c-i*s)>>15;
    }
    for(unsigned length=2;length<=N;length*=2){unsigned half=length/2,step=N/length;
        for(unsigned at=0;at<N;at+=length)for(unsigned j=0;j<half;j++){
            double p=-2*PI*j*step/N;int32_t c=lrint(32767*cos(p)),s=lrint(32767*sin(p));
            unsigned a=at+j,b=a+half;int32_t i=(vi[b]*c-vq[b]*s)>>15,q=(vi[b]*s+vq[b]*c)>>15,ai=vi[a],aq=vq[a];
            vi[a]=(ai+i)>>1;vq[a]=(aq+q)>>1;vi[b]=(ai-i)>>1;vq[b]=(aq-q)>>1;
        }
    }
#else
    modem_dechirp_sc16(window,vector,chirp,N/4);
    modem_fft_sc16((int16_t *)vector,N,(int16_t *)table);
#endif
    unsigned best=0;uint32_t total=0,peak=0;
    for(unsigned k=0;k<N;k++){
#ifdef LORA_HOST_TEST
        int32_t i=vi[k],q=vq[k];
#else
        uint32_t p=((const uint32_t *)vector)[k];int32_t i=(int32_t)(p<<16)>>16,q=(int32_t)p>>16;
#endif
        uint32_t power=i*i+q*q;total+=power;if(power>peak){peak=power;best=k;}
    }
#ifdef LORA_HOST_TEST
    *quality=total?(uint16_t)(((uint64_t)peak*1000u)/total):0;
#else
    *quality=total?(uint16_t)((float)peak*inverse((float)total)*1000.0f):0;best=reverse[best];
#endif
    unsigned cost=CYCLES()-start;if(cost>rx.stats.fft_max)rx.stats.fft_max=cost;
    rx.stats.fft_cycles+=cost;
    rx.stats.ffts++;rx.stats.quality=*quality;return best;
}
HOT static void search(void){rx.state=rx.hits=rx.symbol_count=rx.expected=0;rx.next_fft=rx.end+N;}
HOT static void tick(void){
    uint16_t quality;unsigned bin=detect(rx.state==3,&quality),start=rx.end-N;
    if(rx.state==0){
        unsigned normalized=(bin-start)&127u;
        if(quality>=300){if(rx.hits && near(normalized,rx.last))rx.hits++;else rx.hits=1;rx.last=normalized;}
        else rx.hits=0;
        rx.next_fft=rx.end+N;
        if(rx.hits>=3){rx.state=1;rx.up_bin=bin;rx.next_fft=rx.end+N;}
        return;
    }
    rx.next_fft=rx.end+N;
    if(rx.state==1){
        if(quality>=250 && near(bin,rx.up_bin))return;
        if(quality>=250 && near(bin,(rx.up_bin+8)&127u)){rx.state=2;return;}
        search();return;
    }
    if(rx.state==2){
        if(quality>=250 && near(bin,(rx.up_bin+16)&127u)){rx.state=3;rx.sync2_start=start;return;}
        search();return;
    }
    if(rx.state==3){
        if(quality<250){search();return;}
        int best=100000,delta=0,cfo=0;
        for(int d=-64;d<64;d++){
            int u=signed_bin(rx.up_bin-d),v=signed_bin(bin+d),difference=abs(u-v),sum=abs(u+v);
            if(difference>1 || sum>32)continue;
            int score=difference*64+sum;if(score<best){best=score;delta=d;cfo=(u+v)/2;}
        }
        if(best==100000){search();return;}
        rx.stats.timing=delta;rx.stats.cfo=cfo;rx.stats.syncs++;rx.cfo=cfo;
        rx.payload_start=rx.sync2_start-delta+416u;rx.next_fft=rx.payload_start+N;rx.state=4;return;
    }
    if(rx.symbol_count>=LORA7_MAX_SYMBOLS){search();return;}
    rx.symbols[rx.symbol_count++]=(bin-rx.cfo)&127u;
    if(rx.symbol_count==8){
        bool valid=false;uint8_t adjusted[8];
        for(int correction=-1;correction<=1 && !valid;correction++){
            bool reduced=true;for(unsigned j=0;j<8;j++){adjusted[j]=(rx.symbols[j]-correction)&127u;if((adjusted[j]&3)!=1)reduced=false;}
            if(reduced && lora7_header(adjusted,8,&rx.header) && rx.header.crc){
                memcpy(rx.symbols,adjusted,8);rx.cfo+=correction;rx.stats.cfo=rx.cfo;valid=true;
            }
        }
        if(!valid){rx.stats.header_fail++;search();return;}
        rx.stats.headers++;rx.expected=8+((2u*rx.header.length+4u+6u)/7u)*(4u+rx.header.cr);
    }
    if(rx.expected && rx.symbol_count==rx.expected){
        if(lora7_decode(rx.symbols,rx.symbol_count,rx.payload,sizeof(rx.payload),&rx.header)){
            rx.stats.packets++;if(rx.emit)rx.emit(rx.payload,&rx.header);
        }else rx.stats.crc_fail++;
        rx.expected=0;search();
    }
}
HOT static void sample(modem_iq_t v){
    rx.ring[rx.end&127u]=v;rx.end++;rx.stats.samples++;
    if(rx.filled<N)rx.filled++;
    if(rx.filled==N && rx.end==rx.next_fft)tick();
}
void lora_rx_start(lora_packet_fn emit){init();lora7_init();memset(&rx,0,sizeof(rx));rx.emit=emit;rx.next_fft=N;}
HOT void lora_rx_discontinuity(void){
    lora_packet_fn emit=rx.emit;lora_rx_stats_t stats=rx.stats;
    memset(&rx,0,sizeof(rx));rx.emit=emit;rx.stats=stats;rx.stats.discontinuities++;rx.next_fft=N;
}
HOT void lora_rx_block(const modem_iq_t *input,unsigned count){
    unsigned start=CYCLES();
    rx.stats.input+=count;
    unsigned first=rx.decimation&1u;rx.decimation=(rx.decimation+count)&1u;
    for(unsigned k=first;k<count;k+=2){
        modem_iq_t v=input[k];
        /* Exact -500kHz mixer at 2MS/s: 1,-j,-1,+j. */
        switch(rx.phase&3u){case 1:v=(modem_iq_t){v.q,-v.i};break;case 2:v=(modem_iq_t){-v.i,-v.q};break;case 3:v=(modem_iq_t){-v.q,v.i};break;}
        rx.phase++;
        while(rx.next_fifth<=0){
            int fraction=rx.next_fifth+5;
            /* Q15 weights remove two integer divisions per output sample.
             * Maximum interpolation error versus exact /5 is <1 IQ unit. */
            static const int weight[6]={0,6554,13107,19661,26214,32768};
            int w=weight[fraction];
            modem_iq_t result={
                rx.previous.i+(((int32_t)v.i-rx.previous.i)*w>>15),
                rx.previous.q+(((int32_t)v.q-rx.previous.q)*w>>15)};
            sample(result);rx.next_fifth+=8;
        }
        rx.next_fifth-=5;rx.previous=v;
    }
    unsigned cost=CYCLES()-start;if(cost>rx.stats.block_max)rx.stats.block_max=cost;
    rx.stats.block_cycles+=cost;
}
lora_rx_stats_t lora_rx_stats(void){return rx.stats;}
