#pragma once
/* INFO19 human terminal interface; waveform/PHY/wire2 remain INFO18. */
static void terminal_status(void){
    char out[200];
    snprintf(out,sizeof(out),"CHAT node=%u destination=255 carrier=2439500000 sf=7 bw=1250000 max_bytes=256 receiving=%u\n",
             modem_node,modem_ring_running());reply(out);
}
static void terminal_help(void){
    reply("Type CHAT (or CHAT <node 1..254>) to start. In CHAT, type a UTF8 message and press Enter to broadcast.\n"
          "Commands: /help /status /node <1..254> /send <text> /exit. Use // to send a leading /. Maximum 256 UTF8 bytes.\n"
          "Numeric SYNC is reserved for the GUI; /send sends command-like text literally.\n"
          "TX reports local transmission only; broadcasts have no delivery ACK.\n");
}
static void terminal_emit(unsigned source,const uint8_t *text,unsigned length){
    char out[MODEM_TEXT_MAX*4+40];const char hex[]="0123456789abcdef";
    unsigned used=snprintf(out,sizeof(out),"RX %u bytes=%u ",source,length);
    for(unsigned j=0;j<length;j++){
        unsigned ch=text[j];
        if(ch=='\\'){out[used++]='\\';out[used++]='\\';}
        else if(ch=='\n' || ch=='\r' || ch=='\t'){
            out[used++]='\\';out[used++]=ch=='\n'?'n':ch=='\r'?'r':'t';
        }else if(ch<32 || ch==127){
            out[used++]='\\';out[used++]='x';out[used++]=hex[ch>>4];out[used++]=hex[ch&15];
        }else out[used++]=(char)ch;
    }
    out[used++]='\n';out[used]=0;reply(out);
}
static bool terminal_enter(unsigned node){
    if(!modem_ready){reply("ERR modem_init\n");return false;}
    if(!modem_idle())return false;
    if(!node){
        uint8_t mac[6];
        if(esp_read_mac(mac,ESP_MAC_WIFI_STA)!=ESP_OK){reply("ERR chat_identity\n");return false;}
        node=1+esp_rom_crc32_le(0,mac,sizeof(mac))%254;
    }
    frequency_mhz=2437;rx_ready=false;gain_mode=GAIN_MANUAL;gain_code=65;prepare_rx();
    modem_node=node;modem_peer=255;modem_mix=1;modem_conjugate=1;
    modem_lora=true;lora_reliable=true;lora_ack_drop=0;
    lora_assembly.valid=lora_last.valid=lora_ack.valid=lora_ack_last.valid=lora_wait.active=false;
    terminal_active=true;burst_serial_text_mode(true);
    if(!modem_listen()){
        terminal_active=false;burst_serial_text_mode(false);reply("ERR chat_start\n");return false;
    }
    terminal_status();reply("Ready. Enter sends a broadcast; /help shows commands.\n");return true;
}
static bool terminal_command(char *line){
    unsigned node;char extra;uint64_t nonce;
    /* A fresh GUI/API SYNC switches back to machine replies without changing
     * RF settings. Its existing MODEMOFF/config/MODEMON startup then owns RX. */
    if(terminal_active && sscanf(line,"SYNC %" SCNu64 " %c",&nonce,&extra)==1){
        terminal_active=false;burst_serial_text_mode(false);return false;
    }
    if(!terminal_active){
        if(!strcmp(line,"CHAT")){terminal_enter(0);return true;}
        if(sscanf(line,"CHAT %u %c",&node,&extra)==1 && node>=1 && node<=254){terminal_enter(node);return true;}
        if(!strncmp(line,"CHAT",4)){terminal_help();return true;}
        return false;
    }
    if(!strcmp(line,"/help")){terminal_help();return true;}
    if(!strcmp(line,"/status")){terminal_status();modem_stats();char out[300];modem_health_read(out,sizeof(out));reply(out);return true;}
    if(!strcmp(line,"/exit")){
        terminal_active=false;burst_serial_text_mode(false);
        if(modem_idle()){modem_poll();reply("OK chat_exit\n");}return true;
    }
    if(sscanf(line,"/node %u %c",&node,&extra)==1 && node>=1 && node<=254){terminal_enter(node);return true;}
    bool explicit_send=!strncmp(line,"/send ",6);
    if(line[0]=='/' && line[1]!='/' && !explicit_send){reply("ERR chat_command_use_/help\n");return true;}
    const char *text=explicit_send?line+6:line[0]=='/'?line+1:line;unsigned length=strlen(text);
    if(!length || length>MODEM_TEXT_MAX){reply("ERR chat_text_1_to_256_bytes\n");return true;}
    (void)modem_send_lora((const uint8_t *)text,length,255);
    if(!modem_ring_running() && !modem_listen())reply("ERR modem_resume\n");
    return true;
}
