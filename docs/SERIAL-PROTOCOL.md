# INFO19 serial and application protocol

All commands are case-sensitive. Native USB Serial/JTAG is the tested transport;
open it at nominal 115200 baud, 8N1, no flow control. Physical UART0 uses 2 Mbaud
on GPIO43/TX and GPIO44/RX and remains untested. Only one host reader should own
a port. USB disconnection is an error, not evidence of successful transmission.

## Human terminal mode

After boot, type `CHAT` or `CHAT <node>` and press Enter. Node IDs range from 1
through 254. Without an explicit ID, the code derives one from the Wi-Fi STA MAC:
`1 + CRC32(mac_bytes) % 254`. IDs are not globally unique or authenticated.

CHAT selects LORA7, LO 2437 MHz, manual gain code 65, mix=1 and conjugate=1,
clears message/ACK caches, and starts listening. It uses destination 255 for
every text transmission. Ordinary text followed by Enter is a broadcast;
`TX <sequence> bytes=<length> fragments=<count> broadcast` means the local
bounded waveform transmission completed. Only an independently observed RX
confirms that another radio received it.

The text limit is 256 **bytes**. UTF-8 characters can occupy multiple bytes, and
fragments can divide a character; reassembly precedes display. The firmware
does not validate arbitrary input as well-formed UTF-8. LF, CR and CRLF terminate
a CHAT line; the CRLF pair produces one send. Empty lines do not transmit.
Backspace and DEL remove a UTF-8 code point. Embedded NUL, oversized input,
unknown slash commands and an unfinished line idle for five minutes are rejected.
The parser buffer is 384 bytes including its terminator; overflowing it rejects
the line instead of sending a truncated message.

| Input | Result |
|---|---|
| `/help` | Usage |
| `/status` | CHAT settings, MODEMSTATS and MODEMLIFETIME |
| `/node 1` | Stop/reconfigure/restart RX with ID 1 |
| `/send INFO` | Broadcast literal `INFO` |
| `/send SYNC 123` | Broadcast literal reserved SYNC text |
| `//test` | Broadcast `/test` |
| `/exit` | Stop RX, exit CHAT, reply `OK chat_exit` |

In CHAT, `INFO` alone is ordinary text. A syntactically valid numeric
`SYNC <nonce>` is the exception: it switches to the machine protocol and echoes
the nonce without stopping the existing RX session. This allows a GUI to take
over, then issue MODEMOFF and configure the radio. Use `/send` for such literal
text. Reboot returns to machine/idle mode; CHAT and settings are not saved.

Received output is `RX <source> bytes=<length> <escaped bytes>`. Backslash is
escaped as `\\`; LF/CR/tab as `\n`, `\r`, `\t`; other ASCII controls and DEL
as `\xHH`. Other bytes are copied unchanged. This format keeps a message on
one display line and prevents ASCII control bytes from being executed.

## Machine mode startup

Machine command lines use LF; CRLF also works because CR is ignored. A bare CR
does not terminate a normal machine command. INFO19 specially accepts CHAT entry
with CR. An incomplete machine line expires after three seconds. Responses and
asynchronous received-message events share the same byte stream, so a client
must parse complete records and correlate command replies rather than assume
that the next line is its reply.

A typical node 1 setup is:

```text
SYNC 123456789
INFO
MODEMOFF
FREQ 2437
GAINMANUAL 65
MODEMPHY LORA7
MODEMCONF 1 255 1 1
MODEMREL 1
MODEMACKDROP 0
MODEMON
MODEMINFO
```

Wait for each reply before issuing the next command. SYNC echoes its numeric
nonce; INFO must identify `S3SDR-CODEX 19 autonomous-sf7-terminal 16380`.
Successful configuration/start/stop commands reply `OK`; errors start with
`ERR`. MODEMOFF is required before RF reconfiguration while RX is running.
Peer in MODEMCONF is informational configuration; each MODEMSEND supplies its
own destination. LORA7's acquisition orientation is fixed at mix=1/conjugate=1.

| Command | Purpose |
|---|---|
| `INFO`, `CAPS` | Release identity and capabilities |
| `TRANSPORT?` | Active USB/UART transport and nominal baud |
| `MODEMINFO` | PHY, node/peer, sample rates, byte/fragment limits, ACK settings |
| `MODEMCONF <node> <peer> <mix> <conjugate>` | IDs 1–254 / peer 1–255; mix ±1; conjugate 0/1; stops RX |
| `MODEMPHY LORA7` | Select this release's SF7 PHY; stops RX |
| `MODEMON`, `MODEMOFF` | Start/stop application decoding |
| `MODEMREL 0` or `1` | Disable/enable unicast ACKs and retries |
| `MODEMACKDROP 0..3` | Test control: deliberately drop that many upcoming ACK transmissions |
| `MODEMSTATS` | Current session counters and stream status |
| `MODEMLIFETIME` | Aggregated RX loss/failure counters across TX/RX restarts |
| `MODEMDELIVERY` | TX attempts/retries, ACK timeouts/TX/RX and received messages |
| `LORASTATS`, `LORAKAT` | PHY counters and a fixed codec known-answer test |
| `RELEASE` | Stop ring and release the USB/UART command ownership lease |

USB and UART share the command dispatcher. A five-second lease after each
command rejects commands from the other transport with `ERR busy`. It does not
prevent two programs on the **same** host port from contending; the host must
enforce one owner. RELEASE is a machine command; leave CHAT first. MODEMOFF
uses the full idle/health accounting path and is preferable before RELEASE.
Closing a serial port by itself does not reliably stop the receiver.

## Upload a payload without sending I/Q

The binary-safe send transaction is:

```text
MODEMSEND <byte_count> <destination> <crc32_as_hex>\n
READY <byte_count>\n
<exact byte_count raw payload bytes>
MODEMSENT <sequence> <byte_count> <fragment_count>\n
```

Byte count is 1–256; destination is 1–255. Compute the CRC as
`zlib.crc32(payload) & 0xffffffff`. After READY, write exactly the declared bytes
with **no extra newline**. The upload times out after five seconds. NUL and
newline inside this binary payload are allowed; neither is a command delimiter.
This CRC protects the serial upload and is separate from the PHY's 16-bit CRC.

For example, the machine-side payload preparation for a broadcast is:

```python
import zlib
payload = "Hello from the host!".encode("utf-8")
command = f"MODEMSEND {len(payload)} 255 {zlib.crc32(payload) & 0xffffffff:08x}\n"
# Write command; parse until READY; write payload; parse completion/error.
```

When reliable mode is enabled and destination is 1–254, MODEMSENT requires a
matching ACK for every fragment. Destination 255 is broadcast: one attempt per
fragment, no ACK, and MODEMSENT confirms local completion only. With reliable
mode disabled, unicast completion also has only local meaning. A missing ACK
returns `ERR lora_delivery_unconfirmed sequence=<n>`; the recipient may still
have received the data. Failed upload returns `ERR modem_text_crc_or_timeout`.
Use a timeout long enough for four fragments and retries, rather than a short
terminal-response timeout. A previously listening radio resumes afterward;
`ERR modem_resume` means that restoration failed.

Machine receive records are:

```text
MODEMRX <source> <destination> <sequence> <byte_count> <crc32_hex> <payload_hex>
```

There are exactly seven whitespace-separated fields including MODEMRX. Decode
the hex, check its declared byte length and CRC32, then decode UTF-8 if the
application expects text. Events are emitted only after all fragments are
received with valid PHY header/CRC and pass the application framing rules.

## Application bytes carried by the PHY: wire version 2

The PHY body contains a custom application record, followed by its PHY CRC.
Data tag D4 and ACK tag D5 are hexadecimal byte values, not ASCII strings.

| Byte offset | D4 data record | D5 ACK record |
|---|---|---|
| 0 | `0xd4` | `0xd5` |
| 1 | Source node, 1–254 | ACK sender node |
| 2 | Destination node, 1–255 | Original data source node |
| 3–4 | 16-bit sequence, high byte first | Original sequence, high byte first |
| 5 | Zero-based fragment index | Original fragment index |
| 6 | Fragment count, 1–4 | Original fragment count |
| 7 | Chunk byte length, 1–64 | Zero |
| 8 onward | Chunk bytes | None |

Every non-final fragment is exactly 64 bytes. The final chunk is at most 64.
Normal PHY payloads therefore contain 9–72 bytes for data or 8 for ACKs. RX
accepts records addressed to the local node or broadcast 255. It has one partial
assembly and one completed-message duplicate cache, with a 30-second expiry.
Conflicting fragments are rejected. Duplicate retransmissions can be ACKed
without printing the same completed message again. Concurrent senders can
replace each other's partial assembly; there is no multi-sender scheduler.

Unicast sends make at most three attempts per fragment (two retries). Data
replays last 100 ms. The receiver schedules an ACK 12 ms after valid data and
replays it for 200 ms. The sender waits up to 350 ms for an ACK, and, after
receiving one, keeps RX running for an additional 220 ms before the next TX.
Failed attempts use 10–29 ms retry jitter. ACKs match peer, destination, sequence,
fragment index and fragment count; they are not cryptographically authenticated.

The spreading/encoding resembles LoRa, but these wire records, experimental
1.25 MHz bandwidth and host API do not implement LoRaWAN. A generic receiver
would need matching RF/PHY settings and this application framing to deliver the
same text. Commercial Semtech-chip interoperability remains untested.

## Diagnostics and recovery

The source retains raw sample and PHY probes for research, including TXIQ8,
CAP16, RINGCAP, DACFEED, LORAENC, LORAPROBE and LORARXON. They are not required
for ordinary terminal use. Stop the application ring with MODEMOFF before raw
tests; consult their source handlers for binary framing and hardware changes.
LORARXON exposes PHY payloads as LORARX instead of applying wire2 reassembly.
Do not feed unsupported addresses/settings to the retained register diagnostics.

Monitor MODEMLIFETIME's ADC/BB overruns, event drops, discontinuities and stream
failures as well as MODEMSTATS.running and advancing sample counters. A header
or CRC reject can arise from noise and does not by itself establish a stream
overrun. Counters reaching zero do not prove sample-perfect RF timing. A USB
error, nonzero stream status, stop timeout or resume error must be reported
explicitly; reconnect/reset may be necessary and can renumber the port.
