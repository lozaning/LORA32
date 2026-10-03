#include "modem_dsp.h"
#include <math.h>
#include <string.h>
#ifdef MODEM_HOST_TEST
#define HOT
#define CYCLES() 0u
#else
#include "esp_attr.h"
#include "esp_cpu.h"
#define HOT IRAM_ATTR
#define CYCLES() esp_cpu_get_cycle_count()
/* Positive finite denominator only. Two Newton corrections use the Xtensa
 * hardware reciprocal seed instead of the ROM software divide routine.
 * Canonical sequence: Cadence Xtensa ISA Summary, RECIP0.S. */
static inline float reciprocal(float x) {
    float r,t;
    __asm__("recip0.s %0,%2\nconst.s %1,1\nmsub.s %1,%2,%0\n"
            "madd.s %0,%0,%1\nconst.s %1,1\nmsub.s %1,%2,%0\nmaddn.s %0,%0,%1"
            :"=&f"(r),"=&f"(t):"f"(x));
    return r;
}
#endif
#define PI 3.14159265358979323846
static modem_iq_t chirp[MODEM_N] __attribute__((aligned(16)));
static modem_iq_t twiddle[MODEM_N/2],mixer[8];
static unsigned reverse_bits[MODEM_N];
#ifdef MODEM_HOST_TEST
static int32_t fft_i[MODEM_N],fft_q[MODEM_N];
#else
static modem_iq_t fft_vector[MODEM_N] __attribute__((aligned(16)));
static modem_iq_t fft_table[MODEM_N/2] __attribute__((aligned(16)));
static modem_iq_t mix_table[2][8] __attribute__((aligned(16)));
static modem_iq_t mixed[32] __attribute__((aligned(16)));
extern int modem_fft_sc16(int16_t *,int,int16_t *);
extern void modem_dechirp_sc16(const modem_iq_t *,modem_iq_t *,const modem_iq_t *,unsigned);
extern void modem_mix_iq10(const uint32_t *,modem_iq_t *,const modem_iq_t *,unsigned,unsigned);
#endif
static modem_iq_t tx_chirp[640],tx_osc[40];
static unsigned tx_os=10,tx_period=20;
static modem_block_fn block_sink;
static struct {unsigned phase,count;int32_t i,q;} front;
static struct {
    modem_iq_t window[MODEM_N];
    unsigned used,skip,hits,last_bin,state,bit_count,byte_count,expected;
    uint32_t bits,assembly_sample,last_sample;
    uint8_t bytes[MODEM_FRAME_MAX],node;
    modem_frame_t current;
    modem_message_t assembling;
    uint16_t bitmap,last_sequence;
    uint8_t last_source,fragments;
    bool assembly_valid,last_valid;
    modem_message_fn emit;
    void *context;
    modem_dsp_stats_t stats;
} rx __attribute__((aligned(16)));

void modem_dsp_init(void) {
    for(unsigned k=0;k<MODEM_N;k++) {
        double p=PI*((double)k*k/MODEM_N-k);
        chirp[k]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};
        unsigned r=0;
        for(unsigned j=0;j<MODEM_SF;j++) r=(r<<1)|((k>>j)&1);
        reverse_bits[k]=r;
    }
    for(unsigned k=0;k<MODEM_N/2;k++) {
        double p=-2*PI*k/MODEM_N;
        twiddle[k]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};
#ifndef MODEM_HOST_TEST
        fft_table[reverse_bits[k]>>1]=(modem_iq_t){twiddle[k].i,-twiddle[k].q};
#endif
    }
    for(unsigned k=0;k<8;k++) {
        double p=-2*PI*k/8;
        mixer[k]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};
#ifndef MODEM_HOST_TEST
        mix_table[0][k]=(modem_iq_t){mixer[k].i,-mixer[k].q};
        mix_table[1][k]=mixer[k];
#endif
    }
}
HOT uint16_t modem_crc16(const uint8_t *p,size_t n) {
    uint16_t crc=0xffff;
    while(n--) {
        crc^=(uint16_t)*p++<<8;
        for(unsigned k=0;k<8;k++) crc=(crc<<1)^((crc&0x8000)?0x1021:0);
    }
    return crc;
}
HOT static bool valid_header(const uint8_t *b) {
    return b[0]==MODEM_VERSION && b[1]!=0 && b[1]!=255 && b[2]!=0 &&
        b[6]>=1 && b[6]<=16 && b[5]<b[6] && b[7]>=1 && b[7]<=MODEM_FRAGMENT &&
        (b[5]+1==b[6] || b[7]==MODEM_FRAGMENT);
}
size_t modem_frame_encode(const modem_frame_t *f,uint8_t out[MODEM_SYMBOLS_MAX]) {
    uint8_t b[MODEM_FRAME_MAX]={MODEM_VERSION,f->source,f->destination,
        f->sequence>>8,f->sequence,f->fragment,f->fragments,f->length};
    if(!valid_header(b)) return 0;
    memcpy(b+8,f->payload,f->length);
    unsigned n=8+f->length;
    uint16_t crc=modem_crc16(b,n); b[n++]=crc>>8;b[n++]=crc;
    unsigned k=0;
    for(unsigned j=0;j<6;j++)out[k++]=0;
    out[k++]=13;out[k++]=26;
    uint32_t bits=0;unsigned used=0;
    for(unsigned j=0;j<n;j++) {
        bits=(bits<<8)|b[j];used+=8;
        while(used>=5){used-=5;out[k++]=(bits>>used)&31;}
    }
    if(used)out[k++]=(bits<<(5-used))&31;
    return k;
}
HOT bool modem_frame_decode(const uint8_t *b,size_t n,modem_frame_t *f) {
    if(n<11 || !valid_header(b) || n!=10u+b[7]) return false;
    unsigned end=n-2;
    if(modem_crc16(b,end)!=((uint16_t)b[end]<<8|b[end+1])) return false;
    *f=(modem_frame_t){.source=b[1],.destination=b[2],.sequence=(uint16_t)b[3]<<8|b[4],
        .fragment=b[5],.fragments=b[6],.length=b[7]};
    memcpy(f->payload,b+8,f->length);
    return true;
}
HOT unsigned modem_symbol_detect(const modem_iq_t input[MODEM_N],uint16_t *quality) {
#ifdef MODEM_HOST_TEST
    for(unsigned k=0;k<MODEM_N;k++) {
        int32_t i=input[k].i,q=input[k].q,c=chirp[k].i,s=chirp[k].q;
        unsigned r=reverse_bits[k];
        fft_i[r]=(i*c+q*s)>>15;fft_q[r]=(q*c-i*s)>>15;
    }
    for(unsigned length=2;length<=MODEM_N;length*=2) {
        unsigned half=length/2,step=MODEM_N/length;
        for(unsigned start=0;start<MODEM_N;start+=length) {
            for(unsigned j=0;j<half;j++) {
                unsigned a=start+j,b=a+half;
                int32_t c=twiddle[j*step].i,s=twiddle[j*step].q;
                int32_t vi=(fft_i[b]*c-fft_q[b]*s)>>15,vq=(fft_i[b]*s+fft_q[b]*c)>>15;
                int32_t ai=fft_i[a],aq=fft_q[a];
                fft_i[a]=(ai+vi)>>1;fft_q[a]=(aq+vq)>>1;
                fft_i[b]=(ai-vi)>>1;fft_q[b]=(aq-vq)>>1;
            }
        }
    }
#else
    modem_dechirp_sc16(input,fft_vector,chirp,MODEM_N/4);
    modem_fft_sc16((int16_t *)fft_vector,MODEM_N,(int16_t *)fft_table);
#endif
    /* Input comes from signed IQ10. Stage scaling keeps |components| below
     * 1024, so 32-bin total power fits uint32. Avoid software int64 multiplies
     * and uint64-to-float conversion in the real-time detector. */
    unsigned best=0;uint32_t total=0,peak=0;
    for(unsigned k=0;k<MODEM_N;k++) {
#ifdef MODEM_HOST_TEST
        int32_t i=fft_i[k],q=fft_q[k];uint32_t p=i*i+q*q;
#else
        uint32_t packed=((const uint32_t *)fft_vector)[k];
        int32_t i=(int32_t)(packed<<16)>>16,q=(int32_t)packed>>16;
        uint32_t p=i*i+q*q;
#endif
        total+=p;if(p>peak){peak=p;best=k;}
    }
#ifdef MODEM_HOST_TEST
    *quality=total?(uint16_t)((peak*1000u)/total):0;
#else
    *quality=total?(uint16_t)((float)peak*reciprocal((float)total)*1000.0f):0;
#endif
#ifdef MODEM_HOST_TEST
    return best;
#else
    return reverse_bits[best];
#endif
}
void modem_decoder_start(uint8_t node,modem_message_fn emit,void *context) {
    memset(&rx,0,sizeof(rx));rx.node=node;rx.emit=emit;rx.context=context;
    memset(&front,0,sizeof(front));block_sink=NULL;
}
HOT void modem_decoder_discontinuity(void) {
    rx.state=rx.used=rx.skip=rx.hits=0;
    /* Erase the interrupted waveform, retaining independently CRC-verified
     * fragments. A missing RF sample cannot invalidate an earlier frame. */
    rx.stats.discontinuities++;
}
HOT void modem_front_discontinuity(void){memset(&front,0,sizeof(front));}
void modem_native_sink(modem_block_fn sink){block_sink=sink;}
HOT static void accept_frame(const modem_frame_t *f) {
    if(f->destination!=rx.node && f->destination!=255) return;
    if(rx.last_valid && f->source==rx.last_source && f->sequence==rx.last_sequence &&
       rx.stats.samples-rx.last_sample<8000000u) return;
    if(rx.assembly_valid && rx.stats.samples-rx.assembly_sample>=8000000u)rx.assembly_valid=false;
    if(!rx.assembly_valid || f->source!=rx.assembling.source ||
       f->sequence!=rx.assembling.sequence || f->destination!=rx.assembling.destination) {
        rx.assembling=(modem_message_t){.source=f->source,.destination=f->destination,.sequence=f->sequence};
        rx.fragments=f->fragments;rx.bitmap=0;rx.assembly_valid=true;
    }
    rx.assembly_sample=rx.stats.samples;
    if(f->fragments!=rx.fragments){rx.assembly_valid=false;rx.stats.header_fail++;return;}
    unsigned offset=f->fragment*MODEM_FRAGMENT;
    if(rx.bitmap&(1u<<f->fragment)) {
        if(memcmp(rx.assembling.text+offset,f->payload,f->length)) {
            rx.assembly_valid=false;rx.stats.header_fail++;
        }
        return;
    }
    memcpy(rx.assembling.text+offset,f->payload,f->length);
    rx.bitmap|=1u<<f->fragment;
    if(f->fragment+1==f->fragments) rx.assembling.length=offset+f->length;
    if(rx.bitmap==(uint16_t)((1u<<f->fragments)-1u)) {
        rx.last_source=f->source;rx.last_sequence=f->sequence;rx.last_valid=true;
        rx.last_sample=rx.stats.samples;
        if(rx.emit)rx.emit(&rx.assembling,rx.context);
        rx.assembly_valid=false;
    }
}
HOT static void decode_symbol(unsigned bin,uint16_t quality) {
    rx.stats.symbols++;rx.stats.quality=quality;
    if(quality<350){rx.state=rx.hits=0;return;}
    if(rx.state==0) {
        if(rx.hits && bin==rx.last_bin)rx.hits++;else {rx.last_bin=bin;rx.hits=1;}
        if(rx.hits>=3) {
            rx.skip=(MODEM_N-bin)&31;rx.state=1;rx.hits=0;
        }
        return;
    }
    if(rx.state==1) {
        if(bin==0)return;
        if(bin==13){rx.state=2;return;}
        rx.state=rx.hits=0;return;
    }
    if(rx.state==2) {
        if(bin==26){rx.state=3;rx.bits=rx.bit_count=rx.byte_count=rx.expected=0;rx.stats.syncs++;return;}
        rx.state=rx.hits=0;return;
    }
    rx.bits=(rx.bits<<5)|bin;rx.bit_count+=5;
    while(rx.bit_count>=8) {
        rx.bit_count-=8;
        if(rx.byte_count>=MODEM_FRAME_MAX){rx.state=0;rx.stats.header_fail++;return;}
        rx.bytes[rx.byte_count++]=(rx.bits>>rx.bit_count)&255;
        if(rx.byte_count==8) {
            if(!valid_header(rx.bytes)){rx.state=0;rx.stats.header_fail++;return;}
            rx.expected=10+rx.bytes[7];
        }
        if(rx.expected && rx.byte_count==rx.expected) {
            if((rx.bits&((1u<<rx.bit_count)-1u))==0 &&
               modem_frame_decode(rx.bytes,rx.expected,&rx.current)) {
                rx.stats.frames++;accept_frame(&rx.current);
            } else rx.stats.crc_fail++;
            rx.state=rx.hits=0;return;
        }
    }
}
HOT void modem_decoder_sample(int16_t i,int16_t q) {
    rx.stats.samples++;
    if(rx.skip){rx.skip--;return;}
    rx.window[rx.used++]=(modem_iq_t){i,q};
    if(rx.used==MODEM_N) {
        uint16_t quality;unsigned start=CYCLES(),bin=modem_symbol_detect(rx.window,&quality);
        rx.stats.fft_cycles+=CYCLES()-start;
        rx.used=0;decode_symbol(bin,quality);
    }
}
HOT void modem_decoder_block(const modem_iq_t *input,unsigned bb) {
    rx.stats.samples+=bb;
    unsigned consumed=0;
    while(consumed<bb) {
        unsigned left=bb-consumed;
        if(rx.skip) {
            unsigned skip=rx.skip<left?rx.skip:left;
            rx.skip-=skip;consumed+=skip;continue;
        }
        unsigned part=MODEM_N-rx.used;if(part>left)part=left;
        memcpy(rx.window+rx.used,input+consumed,part*sizeof(modem_iq_t));
        consumed+=part;rx.used+=part;
        if(rx.used==MODEM_N) {
            uint16_t quality;unsigned start=CYCLES();
            unsigned bin=modem_symbol_detect(rx.window,&quality);
            rx.stats.fft_cycles+=CYCLES()-start;
            rx.used=0;decode_symbol(bin,quality);
        }
    }
}
HOT void modem_decoder_native(const uint32_t *words,unsigned first,unsigned count,int mix_sign,bool conjugate) {
    unsigned phase=front.phase,used=front.count;
    int32_t sum_i=front.i,sum_q=front.q;
    for(unsigned j=0;j<count;) {
#ifndef MODEM_HOST_TEST
        unsigned at=(first+j)&16383;
        if(!(phase&7) && !used && at<=16372 && count-j>=8) {
            unsigned n=count-j;if(n>128)n=128;
            if(n>16380-at)n=16380-at;
            n&=~7u;
            modem_mix_iq10(words+at,mixed,mix_table[(mix_sign<0)^conjugate],n/8,conjugate);
            if(block_sink)block_sink(mixed,n/4);else modem_decoder_block(mixed,n/4);
            j+=n;phase+=n;continue;
        }
#endif
        uint32_t w=words[(first+j)&16383];
        int32_t i=(int32_t)(w<<22)>>22,q=(int32_t)(w<<12)>>22;
        if(conjugate)q=-q;
        unsigned k=phase++&7;
        int32_t c=mixer[k].i,s=mixer[k].q*mix_sign;
        sum_i+=(i*c-q*s)>>15;sum_q+=(i*s+q*c)>>15;
        if(++used==4) {
            modem_iq_t one={sum_i>>2,sum_q>>2};
            if(block_sink)block_sink(&one,1);else modem_decoder_sample(one.i,one.q);
            used=0;sum_i=sum_q=0;
        }
        j++;
    }
    front.phase=phase;front.count=used;front.i=sum_i;front.q=sum_q;
}
modem_dsp_stats_t modem_decoder_stats(void){return rx.stats;}
HOT unsigned modem_front_bench(const uint32_t *words,unsigned n,unsigned iterations) {
    unsigned start=CYCLES();
#ifndef MODEM_HOST_TEST
    for(unsigned j=0;j<iterations;j++)for(unsigned k=0;k+128<=n;k+=128)
        modem_mix_iq10(words+k,mixed,mix_table[0],16,0);
#endif
    return CYCLES()-start;
}
void modem_waveform_init(unsigned clock) {
    tx_os=clock?10:20;tx_period=clock?20:40;
    for(unsigned k=0;k<MODEM_N*tx_os;k++) {
        double t=(double)k/tx_os,p=PI*(t*t/MODEM_N-t);
        tx_chirp[k]=(modem_iq_t){lrint(200*cos(p)),lrint(200*sin(p))};
    }
    for(unsigned k=0;k<tx_period;k++) {
        double p=2*PI*k/tx_period;
        tx_osc[k]=(modem_iq_t){lrint(32767*cos(p)),lrint(32767*sin(p))};
    }
}
unsigned modem_waveform(const uint8_t *symbols,unsigned count,uint32_t *out) {
    unsigned span=MODEM_N*tx_os,total=count*span;
    if(total>16380)return 0;
    for(unsigned a=0;a<count;a++) {
        for(unsigned k=0;k<span;k++) {
            modem_iq_t z=tx_chirp[(k+symbols[a]*tx_os)%span],w=tx_osc[k%tx_period];
            int32_t i=(z.i*w.i-z.q*w.q)>>15,q=(z.i*w.q+z.q*w.i)>>15;
            out[a*span+k]=((uint32_t)i&1023)|(((uint32_t)q&1023)<<10);
        }
    }
    return total;
}
