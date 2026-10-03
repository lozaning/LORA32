#pragma once
#include "esp_heap_caps.h"
/* Bounded packet witness, independent of the stable CSS1 modem. SF7 coding
 * is standard, BW1.25MHz is experimental (not an SX1280 bandwidth). */
extern void lora_rotate_pack(uint32_t *,const uint32_t *,const uint32_t *,unsigned);
static struct {uint32_t *up,*down;uint32_t rotate[128][4] __attribute__((aligned(16)));uint8_t symbols[LORA7_MAX_SYMBOLS];unsigned count,samples;} lora_tx __attribute__((aligned(16)));
IRAM_ATTR static void lora_render(uint32_t *dst,unsigned offset,unsigned words){
    while(words){
        unsigned pos=offset%lora_tx.samples,k,s=0,left;bool down=false;
        if(pos<8*4096u){k=pos&4095;left=4096-k;}
        else if(pos<10*4096u){k=pos&4095;left=4096-k;s=pos<9*4096u?8:16;}
        else if(pos<12*4096u){k=pos&4095;left=4096-k;down=true;}
        else if(pos<12*4096u+1024u){k=pos-12*4096u;left=1024-k;down=true;}
        else{unsigned payload=pos-(12*4096u+1024u);s=lora_tx.symbols[payload/4096];k=payload&4095;left=4096-k;}
        unsigned index=down?k:(k+s*32u)&4095u;
        if(left>4096-index)left=4096-index;
        if(left>words)left=words;
        lora_rotate_pack(dst,(down?lora_tx.down:lora_tx.up)+index,lora_tx.rotate[down?0:s],left/4);
        dst+=left;offset+=left;words-=left;
    }
    __asm__ volatile("memw" ::: "memory");
}
static bool modem_lora_transmit_for(const uint8_t *text,unsigned length,unsigned cr,bool diagnostic,unsigned hold_ms){
    memset(&feed,0,sizeof(feed));feed.hold=hold_ms;feed.render=lora_render;
    lora_tx.count=lora7_encode(text,length,cr,true,lora_tx.symbols,sizeof(lora_tx.symbols));
    lora_tx.samples=(12*4096u+1024u)+lora_tx.count*4096u;
    /* TX hands off only banks0/2. Bank1 remains CPU-owned for the entire
     * bounded witness and holds the lookup tables; it is never the DAC bank.
     * The caller has already stopped ADC capture before using this space. */
    lora_tx.up=(uint32_t *)0x3fcc0000u;lora_tx.down=lora_tx.up+4096;
    for(unsigned j=0;j<4096;j++){
        double base=2*3.14159265358979323846*((double)j*j/(2*4096*32)-(double)j/64);
        double carrier=2*3.14159265358979323846*j/16;
        int16_t i=lrint(180*cos(base+carrier)),q=lrint(180*sin(base+carrier));
        lora_tx.up[j]=(uint16_t)i|((uint32_t)(uint16_t)q<<16);
        i=lrint(180*cos(-base+carrier));q=lrint(180*sin(-base+carrier));
        lora_tx.down[j]=(uint16_t)i|((uint32_t)(uint16_t)q<<16);
    }
    for(unsigned s=0;s<128;s++){
        double j=s*32,phase=-2*3.14159265358979323846*(j*j/(2*4096*32)-j/64+j/16);
        int16_t i=lrint(32767*cos(phase)),q=lrint(32767*sin(phase));
        for(unsigned k=0;k<4;k++)lora_tx.rotate[s][k]=(uint16_t)i|((uint32_t)(uint16_t)q<<16);
    }
    lora_render((uint32_t *)0x3fcb0000u,12*4096u+1024u,FEED_WORDS);
    uint32_t payload_crc=esp_rom_crc32_le(0,(const uint8_t *)0x3fcb0000u,FEED_WORDS*4);
    feed_write(IQ_BUFFER,0);feed_write(IQ_BUFFER+FEED_HALF,1);
    feed_write((uint32_t *)0x3fcb0000u,2);feed_write((uint32_t *)0x3fcb0000u+FEED_HALF,1);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,FEED_WORDS*4);
    prepare_tx(0);phy_stop_tx_tone(1);uint32_t owner=REG_READ(SRAM_OWNER_REG);feed.owner=owner;
    REG_WRITE(S3_RX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    char h[240];
    if(diagnostic){snprintf(h,sizeof(h),"LORAPROBESTART payload=%u cr=%u symbols=%u samples=%u fs=40000000 bw=1250000 first_crc=%08" PRIx32 " payload_crc=%08" PRIx32 "\n",length,cr,lora_tx.count,lora_tx.samples,crc,payload_crc);reply(h);}
    TaskHandle_t task;
    if(xTaskCreatePinnedToCore(feed_task,"lora_witness",4096,NULL,4,&task,1)!=pdPASS){feed.status=3;feed.done=true;}
    while(!feed.done)vTaskDelay(1);
    REG_WRITE(S3_TX_CTRL_REG,0);REG_WRITE(SRAM_OWNER_REG,owner);tx_teardown();rx_ready=false;prepare_rx();s_loaded_words=0;
    lora_tx.up=lora_tx.down=NULL;feed.render=NULL;
    if(diagnostic){snprintf(h,sizeof(h),"LORAPROBEEND status=%u refills=%u late_cycles=%u copy_cycles=%u duration_cycles=%u\n",feed.status,feed.refills,feed.late,feed.copy_max,feed.cycles);reply(h);reply("END\n");}
    return feed.status==0;
}
static bool modem_lora_transmit(const uint8_t *text,unsigned length,unsigned cr,bool diagnostic){return modem_lora_transmit_for(text,length,cr,diagnostic,100);}
static void modem_lora_probe(const uint8_t *text,unsigned length,unsigned cr){modem_lora_transmit(text,length,cr,true);}
