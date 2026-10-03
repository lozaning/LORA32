#!/usr/bin/env python3
"""Broadcast wall-clock text every five seconds using one INFO19 USB radio."""
import argparse
import json
import os
import signal
import subprocess
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
target = p.add_mutually_exclusive_group(required=True)
target.add_argument('--serial', help='USB serial identity shown by firmware.py ports')
target.add_argument('--port', help='COM5, /dev/ttyACM0, or /dev/cu.usbmodem...')
p.add_argument('--interval', type=float, default=5)
p.add_argument('--timezone', default='America/Chicago')
p.add_argument('--evidence', type=Path, default=ROOT / 'captures/clock')
a = p.parse_args()
assert a.interval >= 2
out = a.evidence.resolve()
out.mkdir(parents=True, exist_ok=True)
tz = ZoneInfo(a.timezone)
stopping = False
device = None
state = {'pid': os.getpid(), 'serial': a.serial, 'requested_port': a.port, 'interval_seconds': a.interval,
         'timezone': a.timezone, 'started_utc': datetime.now(timezone.utc).isoformat(),
         'status': 'starting', 'tx_confirmed': 0, 'errors': 0, 'evidence': str(out)}
events = (out / 'events.jsonl').open('a', buffering=1)


def record(kind, **fields):
    item = {'utc': datetime.now(timezone.utc).isoformat(), 'kind': kind, **fields}
    events.write(json.dumps(item, ensure_ascii=False) + '\n')
    print(json.dumps(item, ensure_ascii=False), flush=True)


def save():
    tmp = out / 'state.tmp'
    tmp.write_text(json.dumps(state, indent=2) + '\n')
    tmp.replace(out / 'state.json')


def stop(*_):
    global stopping
    stopping = True


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)


def line_until(wanted, timeout=4):
    deadline = time.monotonic() + timeout
    pending = b''
    while time.monotonic() < deadline:
        part = device.readline()
        if not part:
            continue
        pending += part
        if not pending.endswith(b'\n'):
            continue
        text = pending.rstrip(b'\r\n').decode('utf8', errors='backslashreplace')
        pending = b''
        record('serial_reply', text=text)
        if text.startswith('ERR '):
            raise RuntimeError(text)
        if wanted(text):
            return text
    raise TimeoutError('No expected firmware reply')


def command(text, wanted):
    device.write((text + '\n').encode('utf8'))
    device.flush()
    return line_until(wanted)


def connect():
    global device
    ports = [a.port] if a.port else [x.device for x in list_ports.comports()
             if (x.serial_number or '').casefold() == a.serial.casefold()]
    if len(ports) != 1:
        raise RuntimeError('Target USB serial is absent: ' + a.serial)
    port = ports[0]
    if shutil.which('lsof'):
        owners = subprocess.run(['lsof', '-nP', port, port.replace('/cu.', '/tty.')],
                                capture_output=True, text=True)
        if owners.stdout.strip():
            raise RuntimeError('Another program owns the serial port: ' + owners.stdout.strip())
    device = serial.Serial(port, 115200, timeout=.1, write_timeout=3, exclusive=True if os.name == 'posix' else None)
    device.reset_input_buffer()
    command('SYNC 2026100305', lambda s: s == 'SYNC 2026100305')
    info = command('INFO', lambda s: s.startswith('S3SDR-CODEX '))
    if not info.startswith('S3SDR-CODEX 19 '):
        raise RuntimeError('INFO19 terminal firmware required: ' + info)
    command('MODEMOFF', lambda s: s == 'OK')
    chat = command('CHAT', lambda s: s.startswith('CHAT node='))
    if 'receiving=1' not in chat:
        raise RuntimeError(chat)
    line_until(lambda s: s.startswith('Ready.'))
    state.update(port=port, firmware=info, chat=chat, status='broadcasting')
    record('ready', serial=a.serial, port=port, chat=chat)
    save()


try:
    save()
    connect()
    deadline = time.monotonic()
    while not stopping:
        remaining = deadline - time.monotonic()
        if remaining > 0:
            # Keep reading incoming decoded text while waiting for the next TX.
            raw = device.readline()
            if raw:
                record('serial_async', text=raw.decode('utf8', errors='backslashreplace').rstrip())
            continue
        text = datetime.now(tz).strftime('%Y-%m-%d %H:%M:%S %Z')
        state['last_attempt_text'] = text
        record('send', text=text, destination=255)
        sent = command(text, lambda s: s.startswith('TX '))
        if 'bytes=%d ' % len(text.encode()) not in sent or not sent.endswith(' broadcast'):
            raise RuntimeError('Unexpected TX completion: ' + sent)
        state.update(tx_confirmed=state['tx_confirmed'] + 1, last_sent_text=text,
                     last_tx_utc=datetime.now(timezone.utc).isoformat())
        record('tx_complete', text=text, firmware_reply=sent,
               delivery_confirmed=False)
        save()
        deadline += a.interval
        if deadline < time.monotonic():
            # Do not send bursts to catch up after a delayed operation.
            deadline = time.monotonic() + a.interval
except BaseException as e:
    state.update(status='failed', errors=state['errors'] + 1, error=repr(e))
    record('failure', error=repr(e))
    raise
finally:
    if device and device.is_open:
        try:
            command('SYNC 202610030599', lambda s: s == 'SYNC 202610030599')
            command('MODEMOFF', lambda s: s == 'OK')
            command('RELEASE', lambda s: s == 'OK')
        except Exception as e:
            state['cleanup_error'] = repr(e)
            record('cleanup_error', error=repr(e))
        finally:
            device.close()
    if state['status'] != 'failed':
        state['status'] = 'stopped'
    state['ended_utc'] = datetime.now(timezone.utc).isoformat()
    save()
    events.close()
