# INFO19 architecture and implementation map

This document describes the included source. Historical diagnostics remain in
the tree but are separate from the default CHAT/LORA7 path.

## Initialization and task ownership

`main/targets/esp32s3/receiver.c::app_main` initializes NVS, temporarily installs
the USB Serial/JTAG driver, initializes Wi-Fi in `WIFI_MODE_NULL`, disables power
saving, and establishes promiscuous/PHY state. It prepares RX, uninstalls the USB
driver, and uses `burst_serial.c` to directly poll the USB FIFO and optional UART0.
It initializes DSP and ring tasks, seeds a 16-bit message sequence, and enters a
nonblocking command/event loop. Boot does not start modem listening.

CHAT is implemented in `main/diagnostics/terminal_chat.h`. It stops an existing
RX session, sets the LO/gain/mixer orientation, chooses a source ID, clears the
assembly/ACK caches and starts the LORA7 ring. The main loop continues decoding
events while the user is typing; a line transmits only when Enter arrives.

Core 0 runs the main task and decoder worker. Core 1 runs ADC acquisition or the
DAC feeder, never both at once. Stop/wait transitions move between RX and TX.
The code tracks timeout/failure conditions rather than treating a task launch
or hardware completion bit as proof of delivery.

## Fixed memory and RF controls

The source reserves 192 KiB with `SOC_RESERVE_MEMORY_REGION`. A linker script
checks that IRAM and static data/BSS stay below the RF aperture. This reservation
is in the firmware source; no modified ESP-IDF memory-layout file is needed.

| Bank | CPU aperture | TX role | RX role |
|---|---|---|---|
| 0 | `0x3fcb0000`–`0x3fcc0000` | Alternating sample bank | Rotating ADC bank |
| 1 | `0x3fcc0000`–`0x3fcd0000` | CPU-owned chirp lookup tables | Rotating ADC bank |
| 2 | `0x3fcd0000`–`0x3fce0000` | Alternating sample bank / IQ_BUFFER | Rotating ADC bank |

End addresses are exclusive. Each is 64 KiB / 16384 32-bit words. The ordinary
raw snapshot limit is 16380 words. The streaming feeder uses 16320 words, split
into two 8160-word halves. A native I/Q word contains signed 10-bit I in bits
0–9 and signed 10-bit Q in bits 10–19. Serial IQ8 diagnostics use interleaved
signed I/Q bytes; that diagnostic format is not the CHAT transport.

| Register | Use in this pinned S3 implementation |
|---|---|
| `0x600c101c` | Low ownership bits select RF sample SRAM banks |
| `0x60033d5c` | ADC dump control |
| `0x60033d60` | ADC write index |
| `0x60033d64` | DAC/sample replay control |
| `0x60033d68` | Replay-index diagnostic observations |

These are private/reverse-engineered controls, not a supported public I/Q API.
In particular `0x60033d08` is a PLCP descriptor-address register, not a tunable
NCO. Old NCOTONE/NCOSEQ experiments return an error rather than writing it.
The normal transmit route removes the vendor tone generator's override before
feeding samples; merely invoking the vendor ramp/replay helper is insufficient.

## TX: text becomes radio samples

1. `modem_send_lora` splits 1–256 text bytes into one to four 64-byte chunks and
   prepends an eight-byte D4 application header. A PHY payload is at most 72 bytes.
2. `lora7_encode` creates the explicit length/CR/CRC header, header checksum,
   whitened payload nibbles, Hamming codewords, diagonal interleaving and mapped
   SF7 symbols. Header length nibbles are high first. The reduced first block
   includes parity and its trailing zero. PHY payload CRC uses polynomial
   `0x1021` and the LoRa-style final-byte XOR handling in the codec.
3. `lora_tx.h` constructs 4096-sample up/down lookup tables in bank 1 at 40 MS/s,
   plus symbol phase rotations. It renders eight upchirps, sync bins 8/16, two
   downchirps plus a quarter downchirp, then the encoded symbols. The waveform
   center is +2.5 MHz relative to the LO, with 1.25 MHz bandwidth.
4. PIE assembly in `modem_vector.S::lora_rotate_pack` rotates lookup samples and
   packs I/Q words. Rendering advances through successive portions of a complete
   frame, wrapping to repeat the frame during the bounded transmit period.
5. `dac_feed.h::feed_task` arms replay once with its repeat control, then alternates
   RF ownership between banks 0 and 2. Their overlapping currently consumed half
   is duplicated, so the released bank can be filled with future halves. Core 1
   uses the 240 MHz cycle counter: 8160 samples × six CPU cycles/sample is
   48960 cycles, or 204 µs. An experimental 1536-cycle guard is 6.4 µs.
6. The feeder records late/refill/copy limits and stops on missed deadlines. At
   the end it disables replay, restores SRAM ownership and RF/RX state. A data
   fragment is replayed for 100 ms; an ACK uses 200 ms. Table preparation,
   transitions, ACK waits and guards add further host-visible latency.

At 40 MS/s, four-byte native words imply 160 MB/s internally. On-chip modulation
removes serial sample uploads but does not remove this refill deadline. The
timed bank handoff is functional in CRC-valid packet tests; instantaneous read
position and sample-exact, phase-perfect handoffs have not been established.
The diagnostic-origin comment at the top of `dac_feed.h` predates its reuse:
`lora_tx.h` selects `feed.render=lora_render`, so LORA7 MODEMSEND and CHAT do use
that feeder. The older CSS1 path instead replays a finite complete short frame.

## RX: radio samples become packets

`modem_ring.c` starts the native ADC stream at 16 MS/s. It rotates three banks,
uses guard regions/sentinels to establish consumed sample regions, and keeps
the oldest completed bank available while processing it. The acquisition task
is pinned to core 1 and masks interrupts for its critical timing loop. It stops
or reports a discontinuity when safe processing/bank deadlines cannot be met.

`modem_dsp.c` and `modem_vector.S` unpack IQ10, apply the eight-step mixer and
four-sample boxcar reduction, producing 4 MS/s complex int16 data. The chosen
mix/conjugation corrects the measured radio I/Q orientation. A bounded,
single-producer/single-consumer 128-slot baseband queue crosses to core 0, using
memory barriers and a stream epoch so discontinuities reset the decoder.

`lora_rx.c` reduces by two to 2 MS/s, mixes by -500 kHz using `1,-j,-1,+j`, and
linearly resamples by 5/8 to 1.25 MS/s. A 128-sample ring holds one SF7 symbol.
Dechirping and a fixed-point 128-point PIE FFT identify the peak bin. The ESP-DSP
derived FFT adds a rounding/bias correction and returns bit-reversed output,
which the detector maps back to symbol bins.

Acquisition follows explicit states: search for repeated upchirps; find sync
bin 8; find sync bin 16; identify downchirps and timing/CFO; collect payload
symbols. Quality gates prevent weak/noise windows from immediately becoming
packets. The first eight symbols are checked as a reduced explicit header,
including small carrier-offset correction candidates. Header length and CR
determine the remaining symbol count. The decoder accepts only CRC-enabled
payloads on this application path and emits packets after successful coding
and CRC checks. CR1/CR2 detect errors; CR3/CR4 allow only a unique one-bit
codeword correction. No correction bypasses the final payload CRC.

The PHY emits into an eight-slot application packet-event queue. The main task
validates D4/D5 headers, addresses and chunk structure, then reassembles text.
One partial message and one recently completed-message cache are maintained;
interleaving independent senders can replace an in-progress assembly. The
prototype does not provide a general multi-user MAC or multi-message queue.

## Half duplex, ACKs and health

The unicast host path uses a D5 ACK for each fragment. There are three attempts
per fragment, at most two retries. An ACK is scheduled after 12 ms; its bounded
replay lets the sender finish its own data replay and switch to RX. The sender
waits up to 350 ms and leaves a 220 ms receive guard after an ACK, with 10–29 ms
jitter between unsuccessful attempts. Destination 255 has one attempt and no
ACK. Repeated packets are content-checked and deduplicated; caches expire after
30 seconds. Exact timing constants are in `modem_commands.h`.

`MODEMSTATS` describes the current RX session and resets on restart.
`MODEMLIFETIME` accumulates ADC/BB overruns, dropped events, discontinuities and
stream failures across sessions, plus maximum depth/decode cost. `MODEMDELIVERY`
tracks attempts, retries, ACK timeouts and counts. Zero pipeline overruns does
not prove RF delivery: a sustained run lost an ACK despite healthy counters.

## Source map

| File | Main responsibility |
|---|---|
| `targets/esp32s3/receiver.c` | Boot, PHY state, raw RF controls, command dispatch |
| `common/burst_serial.c` | USB FIFO/UART transport, line parser, binary transfers |
| `diagnostics/terminal_chat.h` | Human CHAT mode and escaped RX text |
| `diagnostics/modem_commands.h` | Application D4/D5 frames, fragments, ACK/retry, commands |
| `diagnostics/lora_tx.h` | Packet/chirp rendering and bounded transmit orchestration |
| `diagnostics/dac_feed.h` | Timed TX bank handoffs and deadline accounting |
| `common/modem_ring.c` | Native ADC banks, queue scheduling, events and health |
| `common/modem_dsp.c` | Native mixer/decimator; retained CSS1 implementation |
| `common/lora_rx.c` | SF7 streaming acquisition/demodulation |
| `common/lora_codec.c` | Explicit header, whitening, FEC, interleaving, Gray/CRC |
| `targets/esp32s3/modem_vector.S` | PIE unpack, mix, rotate, dechirp |
| `targets/esp32s3/modem_fft.S` | PIE FFT |
| `targets/esp32s3/sram_guard.ld` | Link-time aperture overlap checks |
| `targets/esp32s3/display_modem.c` | Display stub, not a text/LCD UI |

Public RF snapshot and diagnostic commands remain available but are not needed
for serial chat. Both CPUs run at 240 MHz; watchdogs are disabled. Those choices
serve dedicated timing work, not a general Wi-Fi/GUI application scheduler.
