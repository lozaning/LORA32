# What has actually been validated

This is an engineering prototype with working bounded RF demonstrations, not
a production release. The exact tested INFO19 application SHA256 is
`9b52c7b676945247ba00ccaf386008f390fb34358b4165cf0a664ef41867d665`.
The three prebuilt images in this export are copied from that preserved package.
The full source snapshot and hashes are included; a newly compiled binary is a
new artifact and needs its own RF validation.

The project retained serial transcripts, raw captures, harness versions and
failures in its local research workspace. This portable export contains a
curated scope summary and receipt hashes in [validation-summary.json](validation-summary.json),
not the private workspace or full recordings. Those hashes identify the original
receipts; they do not allow an outside reader to independently reproduce an RF
measurement without the recordings and hardware.

## Exact INFO19 tests

Two LilyGO T-Dongle S3 boards ran the exact images, with native USB and the
experimental SF7, 1.25 MHz PHY at 2439.5 MHz:

| Measurement | Observed result and scope |
|---|---|
| Terminal regression | Clean run passed 26 cases, including 18 positive/behavior cases and eight invalid-input controls |
| Bidirectional RF text | Exact decoded plain text, Unicode and 256-byte UTF-8 messages, including an emoji split across 64-byte fragments |
| Terminal line handling | CR, LF, CRLF; slow typing across the old three-second deadline; backspace/DEL; slash and numeric-SYNC escaping |
| Rejection and recovery | Oversized 257/400-byte input, embedded NUL and invalid slash controls rejected; subsequent valid messages received |
| Existing binary API | Binary payload bytes, including NUL/newline, still transported by MODEMSEND |
| GUI handoff | Four exact peer receptions with ACKs: plain and maximum UTF-8 text in each direction, after CHAT → numeric SYNC, without reset |
| Health checkpoints | No ADC/BB overruns, event drops, discontinuities or stream failures at the tested checkpoints |

An earlier terminal harness had a cleanup error after its message cases; an
initial GUI handoff harness checked stale worker status. Both attempts were
preserved, corrected and rerun clean. Neither failure was silently counted as
a clean full run.

INFO19 changes the terminal/parser/formatting wrapper. Its waveform renderer,
continuous acquisition, codec and wire2 application pipeline are unchanged from
INFO18. This is why earlier RF investigations are useful context, but they are
not a new independent SDR test of the INFO19 binary.

## Earlier INFO18 evidence

The preserved INFO18 application SHA256 is
`1dba010fd2e8e9680740a7f657fd0c51eb6ae18966f07203016330679285e89a`.
The INFO19 image is the release shipped here.

- ACK/retry tests passed 10/10 bounded cases, including deliberately suppressed
  ACKs, truthful missing-ACK reporting after peer reception, and subsequent
  maximum-size recovery.
- A 275-second acquisition test crossed the 32-bit native sample-counter wrap
  on two boards. Repeated stop accounting did not double-count lifetime health.
- A sustained delivery run succeeded for 232 messages, then message 233 failed
  with an ACK timeout. Repeating that payload 12 times passed; that does not
  resolve the sustained failure.
- Fourteen raw post-own-TX captures were nonconstant, and known peer tones were
  received in both directions without an intervening reset. Some broader raw
  test matrices failed background-contrast controls. Their source of broad RF
  activity was not established; the whole matrix was not a blanket pass.

## Independent LibreSDR witness

A USB B210-compatible **LibreSDR B210mini clone**, identified by UHD as
type=b200/product=B210, provided an independent radio. It was not a genuine
Ettus device and did not run Semtech firmware. The host reference coding and
demodulation followed generic LoRaPHY / gr-lora_sdr algorithms, separate from
the ESP C implementation.

For one of the two boards running INFO18:

- ESP → SDR: the independent receiver accepted preamble, sync bins 8/16,
  downchirps, explicit header and payload CRC, recovering exact ESP text.
- SDR → ESP: a reference-generated waveform produced fresh exact MCU-decoded
  MODEMRX text. Own ESP transmission followed by SDR reception also passed.
- A deliberately corrupted payload CRC produced no text event and incremented
  the MCU CRC-failure count.
- A four-fragment 256-byte UTF-8 broadcast reassembled exactly on the ESP, and
  the following recovery message was received.

Clean SDR TX used 2.5 MS/s, 2439.5 MHz, gain 70 and amplitude 0.2, for bounded
two-second cases. SDR RX used 5 MS/s and gain 30. Zero UHD errors were reported
in those clean trials. Earlier weak-gain, underflow and software-preflight
failures remain recorded. Some replay candidates still failed header/CRC gates.
These bounded results neither characterize range/sensitivity nor establish
sample-perfect TX handoffs. Independent SDR validation of the second board was
not completed during those tests.

## Four-board deployment and range attempt

Both original boards subsequently received INFO19 with flash image hashes
verified; the newer pair already had INFO19 from the earlier successful test.
USB loss interrupted that four-board follow-up during a backup, before a new
write to the newer pair. A complete simultaneous four-board runtime/RF test was
not finished. Do not represent it as a four-board acceptance pass.

The five-second clock broadcaster recorded 22 **local** TX completions, then
exited on USB `Device not configured` / Errno 6. Cleanup also failed because
USB was gone. The script was already stopped when the user later requested
stop. Independent remote reception throughout the walk was not recorded, so
that attempt establishes neither radio range nor uninterrupted transmission.

## Outstanding work

- Determine the cause of intermittent USB handle/disconnect failures and RF
  delivery failures; the current evidence does not identify a unique cause.
- Measure spectrum, RF power, receiver sensitivity, range, frequency error,
  packet error rate and TX handoff continuity with controlled equipment.
- Validate prolonged uptime, multiple simultaneous senders, collision recovery,
  physical UART operation and additional S3 board variants.
- Test an actual compatible Semtech radio before claiming interoperability.
  The current 1.25 MHz bandwidth is experimental and the application is custom.
- Add LCD presentation or auto-enter CHAT only as a separately validated change;
  neither is included here.

The exported source was freshly compiled on macOS with the pinned SDK. It
produced a 709152-byte application that fits the 1 MB partition. The portable
host codec check passed its reference decode, 60 length/coding-rate round trips
and rejection controls. The initial new host test incorrectly required matching
unused padding with a historical reference vector; correcting that check left
the firmware source unchanged. CLI imports/help, image-verification and flash
dry-run checks passed without opening a port. Windows/Linux paths are documented
but were not executed on those operating systems.

The export's build/host checks are recorded separately in validation-summary.
No device was opened, reset, transmitted or flashed merely to prepare this
GitHub folder.
