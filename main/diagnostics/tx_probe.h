/* Research probes derived from the linked S3 PHY library, not packet DMA.
 * Peripheral completion/readback is not evidence of radiated raw I/Q. */
#pragma once
#define S3_TONE1_REG 0x60006040u
#define S3_TONE_LSB_REG 0x60006050u
static const uint32_t rf_probe_regs[]={
    0x60006000u,0x60006004u,S3_TONE1_REG,0x60006044u,S3_TONE_LSB_REG,
    0x600061e4u,0x60033d08u,S3_RX_CTRL_REG,0x60033d60u,S3_TX_CTRL_REG,
    0x60033d68u,0x60033d90u,SRAM_OWNER_REG,0x6001c02cu
};
static void rf_state(void) {
    char h[64];
    for(unsigned i=0;i<sizeof(rf_probe_regs)/sizeof(rf_probe_regs[0]);i++) {
        snprintf(h,sizeof(h),"REG %08" PRIx32 " %08" PRIx32 "\n",
                 rf_probe_regs[i],REG_READ(rf_probe_regs[i]));reply(h);
    }
    reply("END\n");
}

/* start_tx_tone_step splits a 12-bit step into 10 high + 2 low bits in
 * wide mode (6040 bit29); narrow mode uses 10 bits. Preserve attenuation,
 * enable, phase and mode bits. RF Hz/code still needs receiver calibration. */
static void tone_step(int step,bool wide) {
    uint32_t code=(uint32_t)step&(wide?4095u:1023u);
    if(wide)REG_WRITE(S3_TONE_LSB_REG,(REG_READ(S3_TONE_LSB_REG)&~3u)|(code&3u));
    REG_WRITE(S3_TONE1_REG,(REG_READ(S3_TONE1_REG)&~1023u)|(wide?code>>2:code));
}
static void tone_probe(int step,unsigned hold_ms) {
    prepare_tx(0);
    uint32_t saved=REG_READ(S3_TONE1_REG),lsb=REG_READ(S3_TONE_LSB_REG);
    bool wide=(saved&BIT(29))!=0;
    uint32_t plcp=REG_READ(0x60033d08u);
    tone_step(step,wide);
    uint32_t observed=REG_READ(S3_TONE1_REG),observed_lsb=REG_READ(S3_TONE_LSB_REG);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    REG_WRITE(S3_TONE1_REG,saved);REG_WRITE(S3_TONE_LSB_REG,lsb);
    tx_teardown();rx_ready=false;prepare_rx();
    char h[180];
    snprintf(h,sizeof(h),"TONEPROBE %d %u wide=%u reg=%08" PRIx32
             " lsb=%08" PRIx32 " plcp=%08" PRIx32 "\n",
             step,hold_ms,wide,observed,observed_lsb,plcp);reply(h);
}
static bool tone_sequence(unsigned count,uint32_t period,unsigned reps) {
    if(period<240u || period>240000u || !reps || reps>1000u ||
       (uint64_t)count*period*reps>120000000u) {
        reply("ERR tone_duration\n");return false;
    }
    for(unsigned i=0;i<count;i++) {
        int32_t s=(int32_t)IQ_BUFFER[i];
        if(s < -512 || s > 511){reply("ERR tone_step\n");return false;}
    }
    prepare_tx(0);
    uint32_t saved=REG_READ(S3_TONE1_REG),lsb=REG_READ(S3_TONE_LSB_REG);
    bool wide=(saved&BIT(29))!=0;
    uint32_t next=esp_cpu_get_cycle_count(),max_late=0,start=next;
    for(unsigned r=0;r<reps;r++)for(unsigned i=0;i<count;i++) {
        while((int32_t)(esp_cpu_get_cycle_count()-next)<0){}
        uint32_t late=esp_cpu_get_cycle_count()-next;
        if(late>max_late)max_late=late;
        tone_step((int32_t)IQ_BUFFER[i],wide);
        next+=period;
    }
    while((int32_t)(esp_cpu_get_cycle_count()-next)<0){}
    uint32_t elapsed=esp_cpu_get_cycle_count()-start;
    REG_WRITE(S3_TONE1_REG,saved);REG_WRITE(S3_TONE_LSB_REG,lsb);
    tx_teardown();rx_ready=false;prepare_rx();
    char h[100];snprintf(h,sizeof(h),"TONESEQ %u cycles=%" PRIu32
                        " max_late=%" PRIu32 "\n",count,elapsed,max_late);reply(h);
    return true;
}

/* REPLAYPROBE varies only the real dactrig fields and SRAM bank ownership.
 * route=0 keeps the tone as a control; route=1 disables the tone generator
 * after RF bring-up. It does not assume that stopping tone selects SRAM. */
static void replay_probe(unsigned clock,unsigned flag,unsigned owner_bits,
                         unsigned route,unsigned hold_ms) {
    prepare_tx(0);
    if(route)phy_stop_tx_tone(1);
    uint32_t owner=REG_READ(SRAM_OWNER_REG),saved=REG_READ(S3_TX_CTRL_REG);
    uint32_t ctrl=(saved&0x70004000u)|(s_loaded_words&0x3fffu)|
                  ((s_loaded_words&255u)<<20)|(clock?0:BIT(15))|(flag?BIT(19):0);
    REG_WRITE(S3_RX_CTRL_REG,0);
    REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|owner_bits);
    uint32_t completions=0,timeouts=0,last=0;
    int64_t start=esp_timer_get_time();
    do {
        REG_WRITE(S3_TX_CTRL_REG,ctrl);
        REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
        int64_t deadline=esp_timer_get_time()+2000;
        do {last=REG_READ(S3_TX_CTRL_REG);}while(!(last&BIT(18)) && esp_timer_get_time()<deadline);
        if(last&BIT(18))completions++;else timeouts++;
    }while(esp_timer_get_time()-start<(int64_t)hold_ms*1000);
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-start);
    REG_WRITE(S3_TX_CTRL_REG,saved&~BIT(31));
    REG_WRITE(SRAM_OWNER_REG,owner);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,s_loaded_words*4u);
    tx_teardown();rx_ready=false;prepare_rx();
    char h[160];
    snprintf(h,sizeof(h),"REPLAYPROBE done=%" PRIu32 " timeout=%" PRIu32
             " ctrl=%08" PRIx32 " us=%" PRIu32 " crc=%08" PRIx32 "\n",
             completions,timeouts,last,elapsed,crc);reply(h);
}
