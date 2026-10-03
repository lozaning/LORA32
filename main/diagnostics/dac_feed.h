#pragma once
#include "esp_attr.h"
/* Bounded TIMED overlapping-bank handoff witness, never used by MODEMSEND. The read cursor is
 * still unknown; timing assumptions must be tested against actual RF. */
#define FEED_HALF 8160u
#define FEED_WORDS (FEED_HALF*2u)
#define FEED_GUARD 1536u /* 6.4 us at 240 MHz, experimental phase margin */
static struct {unsigned hold,late,copy_max,refills,status,cycles,owner;uint32_t *chirp;void (*render)(uint32_t *,unsigned,unsigned);volatile bool done;} feed;
static uint32_t feed_lut[4][32] __attribute__((aligned(16)));
IRAM_ATTR static void feed_copy(uint32_t *dst,const uint32_t *src,unsigned words) {
    unsigned repeats=words/32;
    __asm__ volatile("loopnez %[n], 1f\n"
        "ee.vld.128.ip q0, %[src], 16\n" "ee.vld.128.ip q1, %[src], 16\n"
        "ee.vld.128.ip q2, %[src], 16\n" "ee.vld.128.ip q3, %[src], 16\n"
        "ee.vld.128.ip q4, %[src], 16\n" "ee.vld.128.ip q5, %[src], 16\n"
        "ee.vld.128.ip q6, %[src], 16\n" "ee.vld.128.ip q7, %[src], 16\n"
        "ee.vst.128.ip q0, %[dst], 16\n" "ee.vst.128.ip q1, %[dst], 16\n"
        "ee.vst.128.ip q2, %[dst], 16\n" "ee.vst.128.ip q3, %[dst], 16\n"
        "ee.vst.128.ip q4, %[dst], 16\n" "ee.vst.128.ip q5, %[dst], 16\n"
        "ee.vst.128.ip q6, %[dst], 16\n" "ee.vst.128.ip q7, %[dst], 16\n1:"
        :[src]"+a"(src),[dst]"+a"(dst):[n]"a"(repeats):"memory");
}
IRAM_ATTR static void feed_write(uint32_t *dst,unsigned pattern) {
    if(feed.render){feed.render(dst,pattern*FEED_HALF,FEED_HALF);return;}
    if(feed.chirp) {
        unsigned index=(pattern*FEED_HALF)&4095u,left=FEED_HALF;
        while(left) {
            unsigned n=4096u-index;if(n>left)n=left;
            feed_copy(dst,feed.chirp+index,n);dst+=n;left-=n;index=0;
        }
        __asm__ volatile("memw" ::: "memory");return;
    }
    const uint32_t *src=feed_lut[pattern&3u];unsigned repeats=FEED_HALF/32;
    /* Source table lives in CPU-owned normal RAM. Load every vector before
     * writing SRAM; no CPU reads from any RF-owned bank while DAC runs. */
    __asm__ volatile("ee.vld.128.ip q0, %[src], 16\n"
        "ee.vld.128.ip q1, %[src], 16\n"
        "ee.vld.128.ip q2, %[src], 16\n"
        "ee.vld.128.ip q3, %[src], 16\n"
        "ee.vld.128.ip q4, %[src], 16\n"
        "ee.vld.128.ip q5, %[src], 16\n"
        "ee.vld.128.ip q6, %[src], 16\n"
        "ee.vld.128.ip q7, %[src], 16\n"
        "loopnez %[n], 1f\n"
        "ee.vst.128.ip q0, %[dst], 16\n"
        "ee.vst.128.ip q1, %[dst], 16\n"
        "ee.vst.128.ip q2, %[dst], 16\n"
        "ee.vst.128.ip q3, %[dst], 16\n"
        "ee.vst.128.ip q4, %[dst], 16\n"
        "ee.vst.128.ip q5, %[dst], 16\n"
        "ee.vst.128.ip q6, %[dst], 16\n"
        "ee.vst.128.ip q7, %[dst], 16\n1:"
        :[src]"+a"(src),[dst]"+a"(dst):[n]"a"(repeats):"memory");
    __asm__ volatile("memw" ::: "memory");
}
IRAM_ATTR static void feed_task(void *unused) {
    (void)unused;
    unsigned half_cycles=FEED_HALF*6u;
    uint32_t ctrl=FEED_WORDS|BIT(19); /* 40M, repeat field0 -> indefinite */
    unsigned irq=portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(S3_TX_CTRL_REG,ctrl);
    unsigned start=esp_cpu_get_cycle_count();
    REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
    unsigned duration=feed.hold*240000u,active=2;
    for(unsigned block=1;block*half_cycles+FEED_GUARD<duration;block++) {
        unsigned deadline=block*half_cycles+FEED_GUARD,at;
        do {at=esp_cpu_get_cycle_count()-start;}while(at<deadline);
        unsigned late=at-deadline;if(late>feed.late)feed.late=late;
        if(late>512){feed.status=1;break;}
        unsigned current_half=block&1u,next=active==2?0:2;
        /* Both banks contain the same currently consumed half. Handoff may
         * happen anywhere inside that overlap; write only the released bank. */
        REG_WRITE(SRAM_OWNER_REG,(feed.owner&~15u)|(1u<<next));
        __asm__ volatile("memw" ::: "memory");
        unsigned t=esp_cpu_get_cycle_count();
        uint32_t *released=(uint32_t *)(0x3fcb0000u+active*65536u);
        feed_write(released+current_half*FEED_HALF,block+2u);
        feed_write(released+(1u-current_half)*FEED_HALF,block+1u);
        active=next;
        unsigned cost=esp_cpu_get_cycle_count()-t;if(cost>feed.copy_max)feed.copy_max=cost;
        feed.refills++;
        if(esp_cpu_get_cycle_count()-start>(block+1u)*half_cycles-FEED_GUARD){feed.status=2;break;}
    }
    feed.cycles=esp_cpu_get_cycle_count()-start;
    REG_WRITE(S3_TX_CTRL_REG,0);
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    __asm__ volatile("memw" ::: "memory");feed.done=true;
    vTaskDelete(NULL);
}
static void modem_feed(unsigned hold_ms,bool chirp) {
    memset(&feed,0,sizeof(feed));feed.hold=hold_ms;
    if(chirp) {
        feed.chirp=heap_caps_aligned_alloc(16,4096u*4u,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(!feed.chirp){reply("ERR chirp_memory\n");return;}
        /* SF7 at experimental BW1.25 MHz, 40 MS/s. This is a raw chirp
         * witness, not a Semtech packet. Half-buffer joins move relative to
         * chirp phase, exposing errors hidden by integer-period tone joins. */
        for(unsigned j=0;j<4096;j++) {
            double p=2*3.14159265358979323846*((2.5e6-.625e6)*j/40e6+
                1.25e6*j*j/(2*4096*40e6));
            int i=lrint(180*cos(p)),q=lrint(180*sin(p));
            feed.chirp[j]=(i&1023)|((q&1023)<<10);
        }
    }
    /* Integer cycles in each half, so all four frequency changes start with
     * zero phase. A correct witness cycles 1.25/2.5/3.75/5 MHz every204us. */
    for(unsigned k=0;k<4;k++)for(unsigned j=0;j<32;j++) {
        double p=2*3.14159265358979323846*(k+1)*j/32;
        int i=lrint(180*cos(p)),q=lrint(180*sin(p));
        feed_lut[k][j]=(i&1023)|((q&1023)<<10);
    }
    feed_write(IQ_BUFFER,0);feed_write(IQ_BUFFER+FEED_HALF,1);
    feed_write((uint32_t *)0x3fcb0000u,2);
    feed_write((uint32_t *)0x3fcb0000u+FEED_HALF,1);
    prepare_tx(0);phy_stop_tx_tone(1);
    uint32_t owner=REG_READ(SRAM_OWNER_REG);feed.owner=owner;
    REG_WRITE(S3_RX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    char h[192];snprintf(h,sizeof(h),"DACFEEDSTART %u %u guard=%u chirp=%u\n",FEED_WORDS,hold_ms,FEED_GUARD,chirp);reply(h);
    TaskHandle_t task;
    if(xTaskCreatePinnedToCore(feed_task,"dac_witness",4096,NULL,4,&task,1)!=pdPASS){
        REG_WRITE(S3_TX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,owner);
        tx_teardown();rx_ready=false;prepare_rx();free(feed.chirp);feed.chirp=NULL;reply("ERR dac_task\n");return;
    }
    while(!feed.done)vTaskDelay(1);
    REG_WRITE(SRAM_OWNER_REG,owner);tx_teardown();rx_ready=false;prepare_rx();s_loaded_words=0;
    free(feed.chirp);feed.chirp=NULL;
    snprintf(h,sizeof(h),"DACFEEDEND status=%u refills=%u late_cycles=%u copy_cycles=%u duration_cycles=%u\n",feed.status,feed.refills,feed.late,feed.copy_max,feed.cycles);reply(h);reply("END\n");
}
