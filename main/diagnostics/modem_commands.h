#pragma once
#include <math.h>
#include "modem_dsp.h"
#include "modem_ring.h"
#include "lora_codec.h"
#include "lora_rx.h"
#define LORA_APP_CHUNK 64u
#define LORA_APP_FRAGMENTS 4u
static unsigned modem_node=1,modem_peer=2;
static int modem_mix=1;
static bool modem_conjugate;
static uint16_t modem_sequence;
static bool modem_ready;
static bool modem_lora=true,lora_application;
static uint32_t lora_app_fail,lora_messages;
static bool lora_reliable=true;
static unsigned lora_ack_drop;
static uint32_t lora_tx_attempts,lora_tx_retries,lora_ack_timeouts,lora_ack_tx,lora_ack_rx;
static struct {uint8_t peer,index,count;uint16_t sequence;bool active,seen;} lora_wait;
static struct {uint8_t peer,index,count;uint16_t sequence;int64_t due;bool valid;} lora_ack;
static struct {uint8_t peer,index,count;uint16_t sequence;int64_t at;bool valid;} lora_ack_last;
/* Completed RX sessions remain visible after half-duplex ACK/TX restarts.
 * Counters are owned by the command task and folded only after ring_stop drains
 * both cores. MODEMLIFETIME adds the currently running session exactly once. */
static bool modem_health_active;
static struct {uint32_t sessions,adc,bb,events,discontinuities,failures,depth,decode_max;} modem_health;
static void modem_health_read(char *out,unsigned size);
static bool modem_listen(void);
static void lora_send_ack(void);
static struct {modem_message_t message;unsigned mask,fragments,length[LORA_APP_FRAGMENTS];uint32_t crc[LORA_APP_FRAGMENTS];int64_t updated;bool valid;} lora_assembly;
static struct {uint8_t source,destination,count;uint16_t sequence;unsigned length[LORA_APP_FRAGMENTS];uint32_t crc[LORA_APP_FRAGMENTS];int64_t at;bool valid;} lora_last;
static struct {uint8_t payload[255];lora7_header_t header;} lora_events[8];
static volatile uint32_t lora_head,lora_tail,lora_drops;
static void modem_health_read(char *out,unsigned size){
    modem_ring_stats_t r={0};uint32_t discontinuities=0;
    if(modem_health_active){r=modem_ring_stats();discontinuities=modem_lora?lora_rx_stats().discontinuities:modem_decoder_stats().discontinuities;}
    snprintf(out,size,"MODEMLIFETIME rx_sessions=%" PRIu32 " adc_overruns=%" PRIu32 " bb_overruns=%" PRIu32 " event_drops=%" PRIu32 " discontinuities=%" PRIu32 " stream_failures=%" PRIu32 " bb_depth=%" PRIu32 " decode_max=%" PRIu32 "\n",
        modem_health.sessions,modem_health.adc+r.overruns,modem_health.bb+r.bb_overruns,
        modem_health.events+r.event_drops+lora_drops,modem_health.discontinuities+discontinuities,
        modem_health.failures+(r.status!=0),r.bb_depth_max>modem_health.depth?r.bb_depth_max:modem_health.depth,
        r.decode_max>modem_health.decode_max?r.decode_max:modem_health.decode_max);
}
static void lora_queue(const uint8_t *p,const lora7_header_t *h){
    __asm__ volatile("memw" ::: "memory");
    if(lora_head-lora_tail>=8){lora_drops++;return;}
    unsigned at=lora_head&7;memcpy(lora_events[at].payload,p,h->length);lora_events[at].header=*h;
    __asm__ volatile("memw" ::: "memory");lora_head++;__asm__ volatile("memw" ::: "memory");
}
static struct {uint32_t us,ctrl,index;} dac_trace[512];
static void modem_emit(const modem_message_t *m,void *unused) {
    if(terminal_active){terminal_emit(m->source,m->text,m->length);return;}
    (void)unused;char h[600];
    unsigned used=snprintf(h,sizeof(h),"MODEMRX %u %u %u %u %08" PRIx32 " ",
        m->source,m->destination,m->sequence,m->length,esp_rom_crc32_le(0,m->text,m->length));
    const char hex[]="0123456789abcdef";
    for(unsigned k=0;k<m->length;k++){h[used++]=hex[m->text[k]>>4];h[used++]=hex[m->text[k]&15];}
    h[used++]='\n';h[used]=0;reply(h);
}
static void lora_queue_ack(const uint8_t *p,int64_t now){
    unsigned index=p[5],count=p[6];uint16_t sequence=(uint16_t)p[3]<<8|p[4];
    if(lora_reliable && p[2]==modem_node && p[1]!=modem_node){
        bool recent=lora_ack_last.valid && lora_ack_last.peer==p[1] && lora_ack_last.sequence==sequence &&
            lora_ack_last.index==index && lora_ack_last.count==count && now-lora_ack_last.at<150000;
        bool pending=lora_ack.valid && lora_ack.peer==p[1] && lora_ack.sequence==sequence && lora_ack.index==index && lora_ack.count==count;
        if(!recent && !pending){lora_ack.peer=p[1];lora_ack.sequence=sequence;lora_ack.index=index;lora_ack.count=count;lora_ack.due=now+12000;lora_ack.valid=true;}
    }
}
static void lora_application_packet(const uint8_t *p,unsigned length){
    /* ACKs have their own eight-byte record and the same PHY CRC protection.
     * Match every routing/fragment field; unrelated ACKs cannot confirm a send. */
    if(length==8 && p[0]==0xd5 && p[7]==0){
        uint16_t sequence=(uint16_t)p[3]<<8|p[4];
        if(lora_wait.active && p[1]==lora_wait.peer && p[2]==modem_node &&
           sequence==lora_wait.sequence && p[5]==lora_wait.index && p[6]==lora_wait.count){lora_wait.seen=true;lora_ack_rx++;}
        return;
    }
    unsigned n=length>=8?p[7]:0,count=length>=8?p[6]:0,index=length>=8?p[5]:0;
    if(length<9 || p[0]!=0xd4 || !p[1] || p[1]==255 || !p[2] || !n || n>LORA_APP_CHUNK ||
       length!=8+n || count<1 || count>LORA_APP_FRAGMENTS || index>=count || (index+1<count && n!=LORA_APP_CHUNK) ||
       index*LORA_APP_CHUNK+n>MODEM_TEXT_MAX){lora_app_fail++;return;}
    if(p[2]!=modem_node && p[2]!=255)return;
    uint16_t sequence=(uint16_t)p[3]<<8|p[4];int64_t now=esp_timer_get_time();
    uint32_t hash=esp_rom_crc32_le(0,p+8,n);
    if(lora_last.valid && p[1]==lora_last.source && sequence==lora_last.sequence && now-lora_last.at<30000000){
        if(p[2]==lora_last.destination && count==lora_last.count && n==lora_last.length[index] && hash==lora_last.crc[index])lora_queue_ack(p,now);
        else lora_app_fail++;
        return;
    }
    if(!lora_assembly.valid || now-lora_assembly.updated>=30000000 || p[1]!=lora_assembly.message.source ||
       p[2]!=lora_assembly.message.destination || sequence!=lora_assembly.message.sequence){
        lora_assembly.message=(modem_message_t){.source=p[1],.destination=p[2],.sequence=sequence};
        lora_assembly.fragments=count;lora_assembly.mask=0;memset(lora_assembly.length,0,sizeof(lora_assembly.length));memset(lora_assembly.crc,0,sizeof(lora_assembly.crc));lora_assembly.valid=true;
    }
    if(count!=lora_assembly.fragments){lora_assembly.valid=false;lora_app_fail++;return;}
    unsigned offset=index*LORA_APP_CHUNK;
    if(lora_assembly.mask&(1u<<index)){
        if(n!=lora_assembly.length[index] || memcmp(lora_assembly.message.text+offset,p+8,n)){lora_assembly.valid=false;lora_app_fail++;}
        else lora_queue_ack(p,now);
        return;
    }
    memcpy(lora_assembly.message.text+offset,p+8,n);lora_assembly.length[index]=n;lora_assembly.crc[index]=hash;lora_queue_ack(p,now);lora_assembly.mask|=1u<<index;lora_assembly.updated=now;
    if(index+1==count)lora_assembly.message.length=offset+n;
    if(lora_assembly.mask==((1u<<count)-1u)){
        lora_last.source=p[1];lora_last.destination=p[2];lora_last.count=count;memcpy(lora_last.length,lora_assembly.length,sizeof(lora_last.length));memcpy(lora_last.crc,lora_assembly.crc,sizeof(lora_last.crc));lora_last.sequence=sequence;lora_last.at=now;lora_last.valid=true;
        modem_emit(&lora_assembly.message,NULL);lora_messages++;lora_assembly.valid=false;
    }
}
static void modem_poll(void) {
    modem_message_t event;
    while(modem_ring_event(&event))modem_emit(&event,NULL);
    __asm__ volatile("memw" ::: "memory");
    while(lora_head!=lora_tail){
        unsigned at=lora_tail&7,n=lora_events[at].header.length;char h[600];const char hex[]="0123456789abcdef";
        if(lora_application){lora_application_packet(lora_events[at].payload,n);goto consumed;}
        unsigned used=snprintf(h,sizeof(h),"LORARX %u %u %08" PRIx32 " ",n,lora_events[at].header.cr,esp_rom_crc32_le(0,lora_events[at].payload,n));
        for(unsigned j=0;j<n;j++){h[used++]=hex[lora_events[at].payload[j]>>4];h[used++]=hex[lora_events[at].payload[j]&15];}
        h[used++]='\n';h[used]=0;reply(h);
consumed:
        __asm__ volatile("memw" ::: "memory");lora_tail++;__asm__ volatile("memw" ::: "memory");
    }
    if(lora_application && lora_reliable && !lora_wait.active && lora_ack.valid &&
       esp_timer_get_time()>=lora_ack.due && modem_ring_running())lora_send_ack();
}
static void lora_stats(void){
    lora_rx_stats_t s=lora_rx_stats();modem_ring_stats_t r=modem_ring_stats();char h[600];
    snprintf(h,sizeof(h),"LORASTATS input=%" PRIu32 " samples=%" PRIu32 " ffts=%" PRIu32 " syncs=%" PRIu32 " headers=%" PRIu32 " packets=%" PRIu32 " header_fail=%" PRIu32 " crc_fail=%" PRIu32 " discontinuities=%" PRIu32 " fft_max=%" PRIu32 " block_max=%" PRIu32 " quality=%u timing=%d cfo=%d event_drops=%" PRIu32 " adc_overruns=%" PRIu32 " bb_overruns=%" PRIu32 " status=%" PRIu32 " bb_depth=%" PRIu32 "\n",s.input,s.samples,s.ffts,s.syncs,s.headers,s.packets,s.header_fail,s.crc_fail,s.discontinuities,s.fft_max,s.block_max,s.quality,s.timing,s.cfo,lora_drops,r.overruns,r.bb_overruns,r.status,r.bb_depth_max);reply(h);
}
static void modem_stats(void) {
    modem_ring_stats_t r=modem_ring_stats();modem_dsp_stats_t d=modem_decoder_stats();char h[512];
    if(modem_lora){lora_rx_stats_t l=lora_rx_stats();d=(modem_dsp_stats_t){.samples=l.input,.symbols=l.ffts,.frames=l.packets,.crc_fail=l.crc_fail,.header_fail=l.header_fail+lora_app_fail,.syncs=l.syncs,.discontinuities=l.discontinuities,.quality=l.quality,.fft_cycles=l.fft_cycles};r.event_drops+=lora_drops;}
    snprintf(h,sizeof(h),"MODEMSTATS running=%u units=%" PRIu32 " pairs=%" PRIu32
        " late=%" PRIu32 " slice_cycles=%" PRIu32 " overruns=%" PRIu32 " status=%" PRIu32
        " detail=%" PRIu32 " event_drops=%" PRIu32 " bb_samples=%" PRIu32
        " symbols=%" PRIu32 " frames=%" PRIu32 " crc_fail=%" PRIu32 " header_fail=%" PRIu32
        " syncs=%" PRIu32 " discontinuities=%" PRIu32 " quality=%u fft_cycles=%" PRIu32
        " bb_overruns=%" PRIu32 " bb_depth=%" PRIu32 " decode_cycles=%" PRIu32 "\n",
        modem_ring_running(),r.units,r.pairs,r.late_max,r.slice_max,r.overruns,r.status,r.detail,
        r.event_drops,d.samples,d.symbols,d.frames,d.crc_fail,d.header_fail,d.syncs,d.discontinuities,d.quality,d.fft_cycles,
        r.bb_overruns,r.bb_depth_max,r.decode_max);
    reply(h);
}
static bool modem_idle(void) {
    if(!modem_ring_stop()){reply("ERR modem_stop_timeout\n");return false;}
    if(modem_health_active){
        modem_ring_stats_t r=modem_ring_stats();
        modem_health.adc+=r.overruns;modem_health.bb+=r.bb_overruns;modem_health.events+=r.event_drops;
        modem_health.discontinuities+=modem_lora?lora_rx_stats().discontinuities:modem_decoder_stats().discontinuities;
        modem_health.failures+=r.status!=0;
        if(r.bb_depth_max>modem_health.depth)modem_health.depth=r.bb_depth_max;
        if(r.decode_max>modem_health.decode_max)modem_health.decode_max=r.decode_max;
        modem_health_active=false;
    }
    modem_ring_decoder_sink(NULL);modem_ring_decoder_reset(NULL);
    rx_ready=false;prepare_rx();return true;
}
static bool modem_send_text(const uint8_t *text,unsigned length,unsigned destination) {
    unsigned fragments=(length+MODEM_FRAGMENT-1)/MODEM_FRAGMENT;uint16_t sequence=++modem_sequence;
    uint32_t owner=REG_READ(SRAM_OWNER_REG);bool ok=true;
    modem_waveform_init(1);prepare_tx(0);phy_stop_tx_tone(1);
    REG_WRITE(S3_RX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    for(unsigned fragment=0;fragment<fragments;fragment++) {
        unsigned offset=fragment*MODEM_FRAGMENT,n=length-offset;if(n>MODEM_FRAGMENT)n=MODEM_FRAGMENT;
        modem_frame_t frame={.source=modem_node,.destination=destination,.sequence=sequence,
            .fragment=fragment,.fragments=fragments,.length=n};
        memcpy(frame.payload,text+offset,n);uint8_t symbols[MODEM_SYMBOLS_MAX];
        unsigned count=modem_frame_encode(&frame,symbols),words=modem_waveform(symbols,count,IQ_BUFFER);
        if(!words){ok=false;break;}
        uint32_t ctrl=(REG_READ(S3_TX_CTRL_REG)&0x70004000u)|(words&16383u)|((words&255u)<<20);
        /* Every replay is a COMPLETE finite CSS1 frame. CPU retrigger gaps
         * are between frames; this is deliberately not continuous LoRa TX. */
        int64_t end=esp_timer_get_time()+20000;bool completed=false;
        while(esp_timer_get_time()<end) {
            REG_WRITE(S3_TX_CTRL_REG,ctrl);REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
            int64_t timeout=esp_timer_get_time()+2000;
            while(!(REG_READ(S3_TX_CTRL_REG)&BIT(18)) && esp_timer_get_time()<timeout){}
            if(!(REG_READ(S3_TX_CTRL_REG)&BIT(18))){ok=false;break;}
            completed=true;
        }
        REG_WRITE(S3_TX_CTRL_REG,ctrl);if(!ok || !completed){ok=false;break;}
    }
    REG_WRITE(S3_TX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,owner);
    tx_teardown();rx_ready=false;prepare_rx();
    if(ok){char h[100];snprintf(h,sizeof(h),"MODEMSENT %u %u %u\n",sequence,length,fragments);reply(h);}
    else reply("ERR modem_tx_timeout\n");
    return ok;
}
static void modem_dac_pulse(unsigned clock,unsigned mode,unsigned hold_ms,unsigned repeats,unsigned test_bits) {
    prepare_tx(0);phy_stop_tx_tone(1);uint32_t owner=REG_READ(SRAM_OWNER_REG);
    REG_WRITE(S3_RX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    unsigned words=s_loaded_words;
    uint32_t ctrl=(REG_READ(S3_TX_CTRL_REG)&0x70004000u)|(words&16383u)|
        ((repeats&255u)<<20)|(clock?0:BIT(15))|((mode&1)?BIT(19):0)|((mode&2)?BIT(17):0)|test_bits;
    uint32_t before=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,words*4);
    char h[128];snprintf(h,sizeof(h),"DACSTART %u %u %u %u\n",words,clock,mode,hold_ms);reply(h);
    REG_WRITE(S3_TX_CTRL_REG,ctrl);int64_t start=esp_timer_get_time();
    REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
    unsigned recorded=0,changes=0,previous=REG_READ(0x60033d68u),complete_us=0;bool completion=false;
    while(esp_timer_get_time()-start<(int64_t)hold_ms*1000) {
        unsigned elapsed=esp_timer_get_time()-start,index=REG_READ(0x60033d68u),state=REG_READ(S3_TX_CTRL_REG);
        if(index!=previous){changes++;previous=index;}
        if((state&BIT(18)) && !completion){completion=true;complete_us=elapsed;}
        if(recorded<512 && elapsed>=recorded*2) dac_trace[recorded++]=(typeof(dac_trace[0])){elapsed,state,index};
    }
    uint32_t end_state=REG_READ(S3_TX_CTRL_REG),after=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,words*4);
    REG_WRITE(S3_TX_CTRL_REG,0);
    esp_rom_delay_us(4);
    uint32_t stopped=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,words*4);
    REG_WRITE(SRAM_OWNER_REG,owner);
    tx_teardown();rx_ready=false;prepare_rx();
    snprintf(h,sizeof(h),"DACEND %u completed=%u changes=%u ctrl=%08" PRIx32 " before=%08" PRIx32 " after=%08" PRIx32 " stopped=%08" PRIx32 "\n",words,completion,changes,end_state,before,after,stopped);reply(h);
    snprintf(h,sizeof(h),"DACMETA repeats=%u completion_us=%u\n",repeats,complete_us);reply(h);
    for(unsigned j=0;j<recorded;j++) {
        snprintf(h,sizeof(h),"DACTRACE %" PRIu32 " %08" PRIx32 " %08" PRIx32 "\n",dac_trace[j].us,dac_trace[j].ctrl,dac_trace[j].index);reply(h);
    }
    reply("END\n");
}
#include "dac_feed.h"
#include "lora_tx.h"
#include "bb_capture.h"
static bool modem_listen(void){
    prepare_rx();s_loaded_words=0;
    if(modem_lora){
        lora_rx_start(lora_queue);lora_application=true;
        modem_ring_decoder_sink(lora_rx_block);modem_ring_decoder_reset(lora_rx_discontinuity);
        bool ok=modem_ring_start(modem_node,1,true);if(ok){modem_health_active=true;modem_health.sessions++;}return ok;
    }
    lora_application=false;modem_ring_decoder_sink(NULL);modem_ring_decoder_reset(NULL);
    bool ok=modem_ring_start(modem_node,modem_mix,modem_conjugate);if(ok){modem_health_active=true;modem_health.sessions++;}return ok;
}
static void lora_send_ack(void){
    uint8_t packet[8]={0xd5,modem_node,lora_ack.peer,lora_ack.sequence>>8,lora_ack.sequence,lora_ack.index,lora_ack.count,0};
    lora_ack_last.peer=lora_ack.peer;lora_ack_last.sequence=lora_ack.sequence;
    lora_ack_last.index=lora_ack.index;lora_ack_last.count=lora_ack.count;lora_ack_last.valid=true;lora_ack.valid=false;
    if(lora_ack_drop){lora_ack_drop--;lora_ack_last.at=esp_timer_get_time();return;}
    if(modem_idle()){
        if(modem_lora_transmit_for(packet,sizeof(packet),1,false,200))lora_ack_tx++;
        (void)modem_listen();
    }
    lora_ack_last.at=esp_timer_get_time();
}
static bool modem_send_lora(const uint8_t *text,unsigned length,unsigned destination){
    unsigned fragments=(length+LORA_APP_CHUNK-1)/LORA_APP_CHUNK;uint16_t sequence=++modem_sequence;bool ok=true;
    for(unsigned fragment=0;fragment<fragments;fragment++){
        unsigned offset=fragment*LORA_APP_CHUNK,n=length-offset;if(n>LORA_APP_CHUNK)n=LORA_APP_CHUNK;
        uint8_t packet[8+LORA_APP_CHUNK]={0xd4,modem_node,destination,sequence>>8,sequence,fragment,fragments,n};
        memcpy(packet+8,text+offset,n);
        bool delivered=false;
        unsigned attempts=lora_reliable && destination!=255?3:1;
        for(unsigned attempt=0;attempt<attempts;attempt++){
            if(!modem_idle()){ok=false;break;}
            lora_tx_attempts++;if(attempt)lora_tx_retries++;
            /* Keep each fragment below14ms. Long40ms frames proved fragile
             * after moving the dongles. Four bounded fragments still preserve
             * complete256-byte messages, without any host waveform traffic. */
            if(!modem_lora_transmit(packet,n+8,1,false)){ok=false;break;}
            if(attempts==1){delivered=true;break;}
            lora_wait.peer=destination;lora_wait.sequence=sequence;lora_wait.index=fragment;lora_wait.count=fragments;
            lora_wait.seen=false;lora_wait.active=true;
            if(!modem_listen()){lora_wait.active=false;ok=false;break;}
            int64_t deadline=esp_timer_get_time()+350000;
            while(!lora_wait.seen && esp_timer_get_time()<deadline){modem_poll();vTaskDelay(1);}
            delivered=lora_wait.seen;
            if(delivered){
                /* The peer's bounded ACK replay may still be on air. Keep RX
                 * running for its full remaining bound before the next TX. */
                deadline=esp_timer_get_time()+220000;
                while(esp_timer_get_time()<deadline){modem_poll();vTaskDelay(1);}
            }else{lora_ack_timeouts++;vTaskDelay(pdMS_TO_TICKS(10+(esp_random()%20)));}
            lora_wait.active=false;
            if(delivered)break;
        }
        if(!ok || !delivered){ok=false;break;}
    }
    char h[100];
    if(ok){
        if(terminal_active)snprintf(h,sizeof(h),"TX %u bytes=%u fragments=%u broadcast\n",sequence,length,fragments);
        else snprintf(h,sizeof(h),"MODEMSENT %u %u %u\n",sequence,length,fragments);
        reply(h);
    }
    else {snprintf(h,sizeof(h),"ERR lora_delivery_unconfirmed sequence=%u\n",sequence);reply(h);}
    return ok;
}
static bool modem_command(char *line) {
    unsigned n,destination,crc,rate,units,inversion,node,peer;int mix;char extra;
    if(!strncmp(line,"MODEM",5) || !strncmp(line,"LORA",4) || !strncmp(line,"RINGCAP",7) || !strncmp(line,"DAC",3)) {
        if(!modem_ready){reply("ERR modem_init\n");return true;}
    } else return false;
    if(!strcmp(line,"LORARXON")){
        if(!modem_idle())return true;
        lora_rx_start(lora_queue);lora_drops=0;lora_application=false;
        modem_ring_decoder_sink(lora_rx_block);modem_ring_decoder_reset(lora_rx_discontinuity);
        if(modem_ring_start(modem_node,1,true)){modem_health_active=true;modem_health.sessions++;reply("OK\n");}else reply("ERR lora_rx_start\n");
    }else if(!strcmp(line,"LORABENCH")){
        if(!modem_idle())return true;
        modem_iq_t zeros[32] __attribute__((aligned(16)))={0};lora_rx_start(NULL);
        unsigned irq=portSET_INTERRUPT_MASK_FROM_ISR(),begin=esp_cpu_get_cycle_count();
        for(unsigned j=0;j<10000;j++)lora_rx_block(zeros,32);
        unsigned elapsed=esp_cpu_get_cycle_count()-begin;portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
        lora_rx_stats_t s=lora_rx_stats();char out[150];
        snprintf(out,sizeof(out),"LORABENCH cycles=%u budget=19200000 fft_cycles=%" PRIu32 " block_cycles=%" PRIu32 " ffts=%" PRIu32 "\n",elapsed,s.fft_cycles,s.block_cycles,s.ffts);reply(out);
    }else if(!strcmp(line,"LORASTATS"))lora_stats();
    else if(!strcmp(line,"LORAMEM")) {
        char out[100];snprintf(out,sizeof(out),"LORAMEM free=%u largest=%u\n",(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));reply(out);
    } else if(!strcmp(line,"LORAKAT")) {
        const uint8_t s[]={29,1,97,125,37,109,1,97,119,51,20,1,22,82,37,58,2,48,12,126,120,100,110,58};
        uint8_t p[255];lora7_header_t h;char out[100];
        bool ok=lora7_decode(s,sizeof(s),p,sizeof(p),&h);
        snprintf(out,sizeof(out),"LORAKAT %u len=%u cr=%u crc=%u corrected=%u payload=%02x%02x%02x%02x\n",ok,ok?h.length:0,ok?h.cr:0,ok?h.crc:0,ok?h.corrected:0,ok?p[0]:0,ok?p[1]:0,ok?p[2]:0,ok?p[3]:0);reply(out);
    } else if(sscanf(line,"LORAENC %u %u %x %c",&n,&rate,&crc,&extra)==3 && n>=2 && n<=255 && rate>=1 && rate<=4) {
        if(!modem_idle())return true;
        uint8_t text[255],s[LORA7_MAX_SYMBOLS];char h[1250];
        snprintf(h,sizeof(h),"READY %u\n",n);reply(h);
        if(!burst_serial_receive(text,n,5000) || esp_rom_crc32_le(0,text,n)!=crc){reply("ERR lora_text_crc_or_timeout\n");return true;}
        unsigned count=lora7_encode(text,n,rate,true,s,sizeof(s));
        unsigned used=snprintf(h,sizeof(h),"LORASYMS %u %u %u ",n,rate,count);const char hex[]="0123456789abcdef";
        for(unsigned j=0;j<count;j++){h[used++]=hex[s[j]>>4];h[used++]=hex[s[j]&15];}
        h[used++]='\n';h[used]=0;reply(h);
    } else if(sscanf(line,"LORAPROBE %u %u %x %c",&n,&rate,&crc,&extra)==3 && n>=2 && n<=255 && rate>=1 && rate<=4) {
        if(!modem_idle())return true;
        uint8_t text[255];char h[40];snprintf(h,sizeof(h),"READY %u\n",n);reply(h);
        if(!burst_serial_receive(text,n,5000) || esp_rom_crc32_le(0,text,n)!=crc){reply("ERR lora_text_crc_or_timeout\n");return true;}
        modem_lora_probe(text,n,rate);
    } else if(sscanf(line,"MODEMCAPBB %u %c",&n,&extra)==1 && n>=2048 && n<=32768) {
        if(!modem_idle())return true;
        modem_capture_bb(n);
    } else if(!strcmp(line,"MODEMINFO")) {
        char h[256];snprintf(h,sizeof(h),"MODEMINFO %s node=%u peer=%u rx_fs=16000000 bw=%u sf=%u tx_fs=40000000 max_text=256 semtech=0 continuous_dac=%u mix=%d conjugate=%u ack=%u retry_limit=2 wire=2 max_chunk=64 max_fragments=4\n",modem_lora?"LORA7":"CSS1",modem_node,modem_peer,modem_lora?1250000:4000000,modem_lora?7:5,modem_lora,modem_mix,modem_conjugate,lora_reliable);reply(h);
    } else if(!strcmp(line,"MODEMLIFETIME")){char h[300];modem_health_read(h,sizeof(h));reply(h);
    } else if(!strcmp(line,"MODEMDELIVERY")){
        char h[200];snprintf(h,sizeof(h),"MODEMDELIVERY tx_attempts=%" PRIu32 " tx_retries=%" PRIu32 " ack_timeouts=%" PRIu32 " ack_tx=%" PRIu32 " ack_rx=%" PRIu32 " messages=%" PRIu32 "\n",lora_tx_attempts,lora_tx_retries,lora_ack_timeouts,lora_ack_tx,lora_ack_rx,lora_messages);reply(h);
    } else if(sscanf(line,"MODEMREL %u %c",&n,&extra)==1 && n<=1){
        lora_reliable=n;lora_ack.valid=lora_wait.active=lora_ack_last.valid=false;reply("OK\n");
    } else if(sscanf(line,"MODEMACKDROP %u %c",&n,&extra)==1 && n<=3){
        lora_ack_drop=n;reply("OK\n");
    } else if(!strcmp(line,"MODEMPHY LORA7") || !strcmp(line,"MODEMPHY CSS1")){
        if(!modem_idle())return true;
        modem_lora=!strcmp(line,"MODEMPHY LORA7");lora_assembly.valid=lora_last.valid=false;reply("OK\n");
    } else if(!strcmp(line,"MODEMSTATS"))modem_stats();
    else if(sscanf(line,"MODEMCONF %u %u %d %u %c",&node,&peer,&mix,&inversion,&extra)==4 &&
            node>=1 && node<=254 && peer>=1 && peer<=255 && (mix==1 || mix==-1) && inversion<=1) {
        if(!modem_idle())return true;
        lora_assembly.valid=lora_last.valid=lora_ack.valid=lora_ack_last.valid=false;modem_node=node;modem_peer=peer;modem_mix=mix;modem_conjugate=inversion;reply("OK\n");
    } else if(!strcmp(line,"MODEMON")) {
        if(modem_ring_running()){reply("OK\n");return true;}
        if(modem_listen())reply("OK\n");else reply("ERR modem_start\n");
    } else if(!strcmp(line,"MODEMOFF")) {
        if(modem_idle()){modem_poll();reply("OK\n");}
    } else if(sscanf(line,"MODEMSEND %u %u %x %c",&n,&destination,&crc,&extra)==3 &&
              n>=1 && n<=MODEM_TEXT_MAX && destination>=1 && destination<=255) {
        bool resume=modem_ring_running();if(!modem_idle())return true;
        uint8_t text[MODEM_TEXT_MAX];char h[40];snprintf(h,sizeof(h),"READY %u\n",n);reply(h);
        bool valid=burst_serial_receive(text,n,5000) && esp_rom_crc32_le(0,text,n)==crc;
        if(valid){if(modem_lora)modem_send_lora(text,n,destination);else modem_send_text(text,n,destination);}else reply("ERR modem_text_crc_or_timeout\n");
        if(resume && !modem_ring_running() && !modem_listen())reply("ERR modem_resume\n");
        if(!resume && modem_ring_running())(void)modem_idle();
    } else if(sscanf(line,"MODEMINJECT %u %x %c",&n,&crc,&extra)==2 && n>=128 && n<=IQ_WORDS) {
        if(!modem_idle())return true;
        if(tx_receive_payload(n*4,crc)) {
            modem_decoder_start(modem_node,modem_emit,NULL);unsigned start=esp_cpu_get_cycle_count();
            modem_decoder_native(IQ_BUFFER,0,n,modem_mix,modem_conjugate);
            unsigned elapsed=esp_cpu_get_cycle_count()-start;char h[80];
            snprintf(h,sizeof(h),"MODEMINJECT %u cycles=%u\n",n,elapsed);reply(h);modem_stats();
        }
    } else if(!strcmp(line,"MODEMBENCH")) {
        if(!modem_idle())return true;
        modem_frame_t f={.source=1,.destination=2,.sequence=42,.fragment=0,.fragments=1,.length=16};
        memcpy(f.payload,"on-chip pipeline",16);uint8_t symbols[MODEM_SYMBOLS_MAX];unsigned count=modem_frame_encode(&f,symbols),words=count*128;
        for(unsigned a=0;a<count;a++)for(unsigned k=0;k<128;k++) {
            double t=fmod((double)k/4+symbols[a],32),phase=3.14159265358979323846*(t*t/32-t)+2*3.14159265358979323846*k/8;
            int i=lrint(350*cos(phase)),q=lrint(350*sin(phase));IQ_BUFFER[a*128+k]=(i&1023)|((q&1023)<<10);
        }
        modem_decoder_start(2,NULL,NULL);unsigned start=esp_cpu_get_cycle_count();
        for(unsigned k=0;k<100;k++)modem_decoder_native(IQ_BUFFER,0,words,1,false);
        unsigned elapsed=esp_cpu_get_cycle_count()-start;char h[120];
        unsigned front=modem_front_bench(IQ_BUFFER,words,100);
        snprintf(h,sizeof(h),"MODEMBENCH words=%u iterations=100 cycles=%u budget=%u frames=%" PRIu32 " vector_cycles=%u\n",words,elapsed,words*100*15,modem_decoder_stats().frames,front);reply(h);
    } else if(sscanf(line,"RINGCAP %u %u %c",&rate,&units,&extra)==2 && (rate==0 || rate==1 || rate==6) && units>=1 && units<=3) {
        if(!modem_idle())return true;
        s_loaded_words=0;prepare_rx();
        if(!modem_ring_capture(rate,units)){modem_stats();reply("ERR ring_capture\n");return true;}
        for(unsigned j=0;j<units;j++) {
            modem_ring_unit_t u=modem_ring_capture_unit(j);const uint32_t *p=modem_ring_bank(u.bank);
            unsigned first=16384-u.first;if(first>u.count)first=u.count;
            uint32_t hash=esp_rom_crc32_le(0,(const uint8_t *)(p+u.first),first*4);
            hash=esp_rom_crc32_le(hash,(const uint8_t *)p,(u.count-first)*4);
            char h[100];snprintf(h,sizeof(h),"RINGUNIT %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %08" PRIx32 "\n",u.index,u.bank,u.first,u.count,hash);reply(h);
            if(!send_bytes(p+u.first,first*4) || !send_bytes(p,(u.count-first)*4)){reply("ERR ring_transport\n");return true;}
        }
        modem_stats();reply("END\n");
    } else if(sscanf(line,"DACFEED %u %c",&n,&extra)==1 && n>=1 && n<=100) {
        if(!modem_idle())return true;
        modem_feed(n,false);
    } else if(sscanf(line,"DACCHIRP %u %c",&n,&extra)==1 && n>=1 && n<=100) {
        if(!modem_idle())return true;
        modem_feed(n,true);
    } else if(sscanf(line,"DACPULSE %u %u %u %c",&rate,&units,&n,&extra)==3 &&
              rate<=1 && units<=3 && n>=1 && n<=500 && s_loaded_words) {
        if(!modem_idle())return true;
        modem_dac_pulse(rate,units,n,s_loaded_words&255u,0);
    } else if(sscanf(line,"DACREPEAT %u %u %u %c",&rate,&units,&n,&extra)==3 &&
              rate<=1 && units<=255 && n>=1 && n<=500 && s_loaded_words) {
        if(!modem_idle())return true;
        modem_dac_pulse(rate,1,n,units,0);
    } else if(sscanf(line,"DACMASK %u %u %u %c",&rate,&units,&n,&extra)==3 &&
              rate<=1 && units<=1 && n>=1 && n<=100 && s_loaded_words) {
        if(!modem_idle())return true;
        modem_dac_pulse(rate,1,n,0,units?BIT(16):0);
    } else reply("ERR modem_command\n");
    return true;
}
