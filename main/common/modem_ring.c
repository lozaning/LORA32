/* ADC bank rotation derived from ESPARGOS esp-sdr 550fade (continuous RX).
 * Every completed unit is boundary-checked. Overruns reset the decoder;
 * missing samples never masquerade as an uninterrupted chirp stream. */
#include "modem_ring.h"
#include <string.h>
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#define HOT IRAM_ATTR
#define CTRL 0x60033d5cu
#define INDEX 0x60033d60u
#define OWNER 0x600c101cu
#define MASK 16383u
#define THRESHOLD 12288u
#define START_GUARD 512u
#define END_GUARD 1024u
#define SENTINEL 0xa5c33c5au
#define NONE 0xffffffffu
#define BB_SLOTS 128u
#define MEMW() __asm__ volatile("memw" ::: "memory")
static volatile unsigned requested,busy;
static volatile unsigned generation;
static unsigned node_id;
static unsigned rate=6,capture_units;
static modem_ring_unit_t captured[3];
static int sign;
static bool invert;
static TaskHandle_t worker;
static TaskHandle_t decoder;
static modem_ring_stats_t stats;
static modem_message_t events[8];
static volatile uint32_t event_head,event_tail;
static struct {unsigned first,count,used;bool pending;} units[3];
static unsigned fifo[3],fifo_count;
/* One millisecond of baseband slack covers core0 interrupts and command
 * handling. The producer still never waits for DSP or overwrites unread data.
 * Reserve room for a full four-block native slice before starting that slice. */
static struct {modem_iq_t iq[32];unsigned count,epoch;} baseband[BB_SLOTS];
static volatile uint32_t bb_head,bb_tail;
static unsigned stream_epoch;
static unsigned decode_epoch;
static modem_block_fn decoder_sink;
static void (*decoder_reset)(void);
static inline uint32_t *bank(unsigned b){return (uint32_t *)(0x3fcb0000u+b*65536u);}
HOT static void select_bank(uint32_t saved,unsigned b) {
    REG_WRITE(OWNER,(saved&~15u)|(1u<<b));MEMW();
}
HOT static void fill(unsigned b,unsigned at,unsigned count) {
    at&=MASK;
    while(count) {
        unsigned n=16384-at;if(n>count)n=count;
        uint32_t *p=bank(b)+at;
        unsigned head=(-at)&3u;if(head>n)head=n;
        for(unsigned i=0;i<head;i++)*p++=SENTINEL;
        unsigned vectors=(n-head)/4;uint32_t value=SENTINEL;
        if(vectors) {
            __asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
                "ee.movi.32.q q6, %[v], 1\n"
                "ee.movi.32.q q6, %[v], 2\n"
                "ee.movi.32.q q6, %[v], 3\n"
                "loopnez %[n], 1f\n"
                "ee.vst.128.ip q6, %[p], 16\n1:"
                :[p]"+a"(p):[v]"a"(value),[n]"a"(vectors):"memory");
        }
        for(unsigned i=head+vectors*4;i<n;i++)*p++=SENTINEL;
        count-=n;at=0;
    }
    MEMW();
}
HOT static unsigned first_written(unsigned b,unsigned origin,unsigned guard) {
    const volatile uint32_t *p=bank(b);
    if(p[origin&MASK]!=SENTINEL || p[(origin+guard-1)&MASK]==SENTINEL)return NONE;
    unsigned lo=0,hi=guard-1;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(p[(origin+mid)&MASK]==SENTINEL)lo=mid+1;else hi=mid;}
    return (origin+lo)&MASK;
}
HOT static unsigned end_written(unsigned b,unsigned origin,unsigned observed,unsigned guard) {
    const volatile uint32_t *p=bank(b);unsigned lo=(observed-origin)&MASK,hi=guard-1;
    if(lo>=guard || p[(origin+hi)&MASK]!=SENTINEL)return NONE;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(p[(origin+mid)&MASK]==SENTINEL)hi=mid;else lo=mid+1;}
    return (origin+lo)&MASK;
}
HOT static void queue_event(const modem_message_t *message,void *unused) {
    (void)unused;
    if(event_head-event_tail>=8){stats.event_drops++;return;}
    events[event_head&7]=*message;MEMW();event_head++;MEMW();
}
HOT static void queue_baseband(const modem_iq_t *iq,unsigned count) {
    MEMW();unsigned depth=bb_head-bb_tail;
    if(depth>=BB_SLOTS){stats.bb_overruns++;stream_epoch++;return;}
    unsigned slot=bb_head&(BB_SLOTS-1);
    memcpy(baseband[slot].iq,iq,count*sizeof(modem_iq_t));
    baseband[slot].count=count;baseband[slot].epoch=stream_epoch;
    if(depth+1>stats.bb_depth_max)stats.bb_depth_max=depth+1;
    MEMW();bb_head++;MEMW();
}
static void decode_task(void *unused) {
    (void)unused;
    for(;;) {
        MEMW();
        if(bb_head==bb_tail) {
            if(!busy && !requested)vTaskDelay(1);else taskYIELD();
            continue;
        }
        unsigned slot=bb_tail&(BB_SLOTS-1);
        if(baseband[slot].epoch!=decode_epoch){decode_epoch=baseband[slot].epoch;modem_decoder_discontinuity();if(decoder_reset)decoder_reset();}
        unsigned start=esp_cpu_get_cycle_count();
        if(decoder_sink)decoder_sink(baseband[slot].iq,baseband[slot].count);
        else modem_decoder_block(baseband[slot].iq,baseband[slot].count);
        unsigned cost=esp_cpu_get_cycle_count()-start;
        if(cost>stats.decode_max)stats.decode_max=cost;
        MEMW();bb_tail++;MEMW();
    }
}
HOT static void retire(unsigned b) {
    if(!units[b].pending)return;
    stats.overruns++;modem_front_discontinuity();stream_epoch++;units[b].pending=false;
    for(unsigned k=0;k<fifo_count;k++)if(fifo[k]==b) {
        for(unsigned j=k+1;j<fifo_count;j++){fifo[j-1]=fifo[j];}
        fifo_count--;break;
    }
}
HOT static void slice(void) {
    if(!fifo_count)return;
    MEMW();if(bb_head-bb_tail>=BB_SLOTS-8)return; /* bounded backpressure; ADC poll continues */
    unsigned b=fifo[0],left=units[b].count-units[b].used,n=left>512?512:left;
    unsigned start=esp_cpu_get_cycle_count();
    modem_decoder_native(bank(b),(units[b].first+units[b].used)&MASK,n,sign,invert);
    unsigned cost=esp_cpu_get_cycle_count()-start;
    if(cost>stats.slice_max)stats.slice_max=cost;
    units[b].used+=n;
    if(units[b].used==units[b].count) {
        units[b].pending=false;
        for(unsigned k=1;k<fifo_count;k++){fifo[k-1]=fifo[k];}
        fifo_count--;
    }
}
HOT static void run(void) {
    memset(&stats,0,sizeof(stats));memset(units,0,sizeof(units));fifo_count=0;
    modem_decoder_start(node_id,queue_event,NULL);
    /* Stop waits for an empty queue. Keep its monotonic SPSC counters across
     * launches: resetting tail before head lets the other core observe stale
     * slots as newly queued work, then increment a reset tail past its head. */
    stream_epoch++;decode_epoch=stream_epoch;
    if(!capture_units)modem_native_sink(queue_baseband);
    unsigned start_probe[3]={0},end_probe[3]={0};
    /* Snapshot diagnostics tolerate the initial ADC index/tap bias at the
     * faster clocks. Steady 16 MS/s DSP keeps the cheaper short probes. */
    unsigned start_guard=capture_units?3072:START_GUARD;
    unsigned end_guard=capture_units?4096:END_GUARD;
    for(unsigned b=0;b<3;b++)fill(b,0,16384);
    unsigned cpp=rate==0?3:rate==1?6:15;
    uint32_t saved=REG_READ(OWNER),ctrl=0x00024000u|(rate==1?BIT(15):rate==6?BIT(16):0);
    unsigned irq=portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(CTRL,0);REG_WRITE(0x60033d90u,0x000c2040u);
    REG_WRITE(CTRL,ctrl);select_bank(saved,0);REG_WRITE(CTRL,ctrl|BIT(31));
    unsigned epoch=esp_cpu_get_cycle_count(),w0=REG_READ(INDEX)&MASK;
    unsigned expected=(w0-256u)&MASK,b=0;bool prepared=false;
    for(unsigned unit=0;;unit++) {
        unsigned next=(b+1)%3,written,observed;
        bool need_next=!capture_units || unit+1<capture_units;
        bool stop=!need_next;
        for(;;) {
            observed=REG_READ(INDEX)&MASK;written=(observed-expected)&MASK;
            unsigned age=esp_cpu_get_cycle_count()-epoch;
            if(age>(16384u-128u)*cpp){stats.status=1;stats.detail=age;break;}
            if(written>=THRESHOLD)break;
            if(!requested){stop=true;need_next=false;}
            /* Keep the oldest completed bank available for the full two-bank
             * grace period. Do not erase its tail early merely to prefill a
             * future bank while there is still time to finish processing it. */
            if(!prepared && need_next && (!units[next].pending || written>=THRESHOLD-2048)) {
                retire(next);
                start_probe[next]=(expected+THRESHOLD)&MASK;
                end_probe[next]=(start_probe[next]+THRESHOLD)&MASK;
                fill(next,start_probe[next],start_guard);fill(next,end_probe[next],end_guard);
                prepared=true;continue;
            }
            unsigned budget=stats.slice_max>8192?stats.slice_max:8192;
            if(!capture_units && written+budget/cpp+256<THRESHOLD)slice();
        }
        if(stats.status)break;
        unsigned late=written-THRESHOLD;
        if(late>stats.late_max)stats.late_max=late;
        if(late>400 || (need_next && !prepared)){stats.status=2;stats.detail=written;break;}
        if(stop)REG_WRITE(CTRL,ctrl);else {select_bank(saved,next);epoch=esp_cpu_get_cycle_count();}
        unsigned t=esp_cpu_get_cycle_count();while(esp_cpu_get_cycle_count()-t<16*cpp){}
        unsigned first=unit?first_written(b,start_probe[b],start_guard):first_written(b,(w0-1024u)&MASK,3072);
        unsigned end=unit?end_written(b,end_probe[b],observed,end_guard):end_written(b,observed,observed,1024);
        if(first==NONE || (unit && first!=expected)){stats.status=3;stats.detail=(first&65535)|(expected<<16);break;}
        if(end==NONE){stats.status=4;stats.detail=observed;break;}
        unsigned count=(end-first)&MASK;
        if(count<(unit?THRESHOLD:THRESHOLD-1024) || count>THRESHOLD+2064){stats.status=5;stats.detail=count;break;}
        if(capture_units)captured[unit]=(modem_ring_unit_t){b,first,count,stats.pairs};
        else {
            units[b].first=first;units[b].count=count;units[b].used=0;units[b].pending=true;
            if(fifo_count>=3){stats.status=6;break;}
            fifo[fifo_count++]=b;
        }
        stats.units++;stats.pairs+=count;
        expected=end;b=next;prepared=false;
        if(stop)break;
    }
    REG_WRITE(CTRL,0);REG_WRITE(OWNER,saved);MEMW();
    if(!stats.status)while(fifo_count)slice();else {modem_front_discontinuity();stream_epoch++;}
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
}
static void task(void *unused) {
    (void)unused;
    for(;;) {
        if(!requested){vTaskDelay(1);continue;}
        busy=1;generation++;MEMW();run();requested=0;MEMW();busy=0;MEMW();
    }
}
bool modem_ring_init(void) {
    /* The empty-queue branch uses taskYIELD, which only schedules equal/higher
     * priority tasks. Keep main and DSP equal so command handling can run. */
    if(xTaskCreatePinnedToCore(decode_task,"modem_dsp",4096,NULL,1,&decoder,0)!=pdPASS)return false;
    return xTaskCreatePinnedToCore(task,"modem_rx",8192,NULL,4,&worker,1)==pdPASS;
}
static bool launch(void) {
    unsigned old=generation;MEMW();requested=1;MEMW();
    int64_t until=esp_timer_get_time()+100000;
    while(generation==old && esp_timer_get_time()<until)vTaskDelay(1);
    return generation!=old;
}
bool modem_ring_decoder_sink(modem_block_fn sink){
    if(busy || requested || bb_head!=bb_tail)return false;
    decoder_sink=sink;MEMW();return true;
}
bool modem_ring_decoder_reset(void (*reset)(void)){
    if(busy || requested || bb_head!=bb_tail)return false;
    decoder_reset=reset;MEMW();return true;
}
bool modem_ring_start(uint8_t node,int mix_sign,bool conjugate) {
    if(requested || busy)return false;
    node_id=node;sign=mix_sign;invert=conjugate;rate=6;capture_units=0;
    return launch();
}
bool modem_ring_running(void){MEMW();return busy || requested || bb_head!=bb_tail;}
bool modem_ring_stop(void) {
    requested=0;MEMW();int64_t until=esp_timer_get_time()+1000000;
    while((busy || bb_head!=bb_tail) && esp_timer_get_time()<until)vTaskDelay(1);
    if(!busy && bb_head==bb_tail){modem_native_sink(NULL);return true;}
    return false;
}
bool modem_ring_event(modem_message_t *message) {
    MEMW();if(event_head==event_tail)return false;
    *message=events[event_tail&7];MEMW();event_tail++;MEMW();return true;
}
modem_ring_stats_t modem_ring_stats(void){MEMW();return stats;}
bool modem_ring_capture(unsigned requested_rate,unsigned count) {
    if(requested || busy || (requested_rate!=0 && requested_rate!=1 && requested_rate!=6) || count<1 || count>3)return false;
    rate=requested_rate;capture_units=count;
    memset(captured,0,sizeof(captured));if(!launch())return false;
    int64_t until=esp_timer_get_time()+1000000;
    while(busy && esp_timer_get_time()<until)vTaskDelay(1);
    if(busy){modem_ring_stop();return false;}
    return stats.status==0 && stats.units==count;
}
modem_ring_unit_t modem_ring_capture_unit(unsigned unit){return captured[unit];}
const uint32_t *modem_ring_bank(unsigned b){return bank(b);}
