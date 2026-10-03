#pragma once
#include "esp_heap_caps.h"
static struct {int8_t *chunk[16];unsigned target,phase;volatile unsigned used;} bb_capture;
IRAM_ATTR static void bb_capture_sink(const modem_iq_t *in,unsigned n){
    for(unsigned j=0;j<n && bb_capture.used<bb_capture.target;j++){
        if(bb_capture.phase++&1u)continue; /* explicit2:1 diagnostic decimation */
        unsigned index=bb_capture.used;int i=in[j].i/4,q=in[j].q/4;
        if(i>127)i=127;
        if(i<-128)i=-128;
        if(q>127)q=127;
        if(q<-128)q=-128;
        int8_t *p=bb_capture.chunk[index/2048]+2*(index&2047);p[0]=i;p[1]=q;
        __asm__ volatile("memw" ::: "memory");bb_capture.used=index+1;
    }
}
static void modem_capture_bb(unsigned count){
    /* A previous failed stop deliberately retained its storage. The caller
     * has now drained the ring, so it can be released before a new capture. */
    modem_ring_decoder_sink(NULL);
    for(unsigned j=0;j<16;j++)free(bb_capture.chunk[j]);
    memset(&bb_capture,0,sizeof(bb_capture));bb_capture.target=count;unsigned chunks=(count+2047)/2048;
    for(unsigned j=0;j<chunks;j++){
        bb_capture.chunk[j]=heap_caps_malloc(4096,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(!bb_capture.chunk[j]){reply("ERR bb_capture_memory\n");goto cleanup;}
    }
    modem_ring_decoder_sink(bb_capture_sink);prepare_rx();
    if(!modem_ring_start(modem_node,modem_mix,modem_conjugate)){reply("ERR bb_capture_start\n");goto cleanup;}
    int64_t timeout=esp_timer_get_time()+200000;
    while(bb_capture.used<count && modem_ring_running() && esp_timer_get_time()<timeout)vTaskDelay(1);
    if(!modem_ring_stop()){reply("ERR bb_capture_stop\n");return;} /* retain live storage if shutdown failed */
    modem_ring_stats_t stats=modem_ring_stats();
    if(bb_capture.used!=count || stats.overruns || stats.bb_overruns || stats.status || modem_decoder_stats().discontinuities){reply("ERR bb_capture_continuity\n");goto cleanup;}
    uint32_t crc=0;
    for(unsigned j=0;j<chunks;j++){unsigned n=count-j*2048;if(n>2048)n=2048;crc=esp_rom_crc32_le(crc,(uint8_t *)bb_capture.chunk[j],n*2);}
    char h[128];snprintf(h,sizeof(h),"BBDATA %u 2000000 %08" PRIx32 "\n",count,crc);reply(h);
    for(unsigned j=0;j<chunks;j++){unsigned n=count-j*2048;if(n>2048)n=2048;if(!send_bytes(bb_capture.chunk[j],n*2))break;}
    modem_stats();reply("END\n");
cleanup:
    modem_ring_decoder_sink(NULL);
    for(unsigned j=0;j<16;j++){free(bb_capture.chunk[j]);bb_capture.chunk[j]=NULL;}
}
