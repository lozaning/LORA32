#!/usr/bin/env python3
"""Verify, build and flash this standalone INFO19 release. No ports open on import."""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IDF_COMMIT = '25fe69f946311abdaf9ad56591f25fedbc20ac98'


def manifest():
    return json.loads((ROOT / 'manifest.json').read_text())


def verify(images_only=False):
    files = manifest()['files']
    if images_only:
        files = {n: h for n, h in files.items() if n.startswith('firmware/info19/')}
        if len(files) != 3:
            raise RuntimeError('Expected three prebuilt image hashes')
    for name, wanted in files.items():
        path = ROOT / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != wanted:
            raise RuntimeError('Missing or changed release file: ' + name)
    print(json.dumps({'verified_files': len(files), 'app_sha256': manifest()['app_sha256']}))


def ports():
    from serial.tools import list_ports
    return list(list_ports.comports())


def resolve_port(args):
    if args.serial:
        found = [x.device for x in ports() if (x.serial_number or '').casefold() == args.serial.casefold()]
        if len(found) != 1:
            raise RuntimeError('Serial identity did not resolve uniquely: ' + repr(found))
        return found[0]
    return args.port


def flash(args):
    verify(images_only=True)
    port = resolve_port(args)
    candidates = [x for x in ports() if x.device == port]
    native = bool(candidates and candidates[0].vid == 0x303a and candidates[0].pid == 0x1001)
    if not args.dry_run and not candidates:
        raise RuntimeError('Port is absent; run the ports subcommand: ' + port)
    if not args.dry_run and shutil.which('lsof'):
        aliases = [port]
        if port.startswith('/dev/cu.'):
            aliases.append(port.replace('/cu.', '/tty.'))
        owner = subprocess.run(['lsof', '-nP', *aliases], capture_output=True, text=True)
        if owner.stdout.strip():
            raise RuntimeError('Close the program owning the serial port:\n' + owner.stdout)
    command = [sys.executable, '-m', 'esptool', '--chip', 'esp32s3', '-p', port,
               '--before', 'usb-reset' if native else 'default-reset',
               '--after', 'watchdog-reset', 'write-flash', '--flash-mode', 'dio',
               '--flash-size', '2MB', '--flash-freq', '80m',
               '0x0', str(ROOT / 'firmware/info19/bootloader.bin'),
               '0x8000', str(ROOT / 'firmware/info19/partition-table.bin'),
               '0x10000', str(ROOT / 'firmware/info19/esp_sdr.bin')]
    print(json.dumps({'port': port, 'dry_run': args.dry_run, 'command': command}), flush=True)
    if not args.dry_run:
        subprocess.run(command, check=True)


def build(args):
    if sys.version_info < (3, 10):
        raise RuntimeError('The pinned ESP-IDF needs Python 3.10+. Activate its environment first.')
    sdk = Path(args.idf_path or os.environ.get('IDF_PATH', ROOT / '.esp-idf')).resolve()
    revision = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != IDF_COMMIT:
        raise RuntimeError('Expected ESP-IDF ' + IDF_COMMIT + ', got ' + revision)
    out = Path(args.build_dir).resolve() if args.build_dir else ROOT / 'build'
    out.mkdir(parents=True, exist_ok=True)
    config = out / 'sdkconfig'
    if not config.exists() or args.fresh_config:
        shutil.copy2(ROOT / 'config/sdkconfig.info19', config)
    env = os.environ.copy()
    env.update(IDF_PATH=str(sdk), IDF_COMPONENT_MANAGER='0', PYTHONDONTWRITEBYTECODE='1')
    for key, folder in [('TMPDIR', '.tmp'), ('CCACHE_DIR', '.cache/ccache'), ('XDG_CACHE_HOME', '.cache')]:
        path = ROOT / folder
        path.mkdir(parents=True, exist_ok=True)
        env[key] = str(path)
    command = [sys.executable, str(sdk / 'tools/idf.py'), '-C', str(ROOT), '-B', str(out),
               '-DIDF_TARGET=esp32s3', '-DSDKCONFIG=' + str(config),
               '-DSDKCONFIG_DEFAULTS=' + str(ROOT / 'sdkconfig.defaults') + ';'
               + str(ROOT / 'sdkconfig.defaults.esp32s3'), 'build']
    subprocess.run(command, check=True, env=env)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    sub.add_parser('verify', help='Verify all files in the frozen release manifest')
    sub.add_parser('ports', help='List devices without opening any port')
    f = sub.add_parser('flash', help='Flash the tested prebuilt images, not a new build')
    which = f.add_mutually_exclusive_group(required=True)
    which.add_argument('--port', help='COM5, /dev/cu.usbmodem..., or /dev/ttyACM0')
    which.add_argument('--serial', help='USB serial identity reported by ports')
    f.add_argument('--dry-run', action='store_true', help='Print the command without opening or flashing a port')
    b = sub.add_parser('build', help='Build source with the pinned, already-installed ESP-IDF')
    b.add_argument('--idf-path')
    b.add_argument('--build-dir')
    b.add_argument('--fresh-config', action='store_true', help='Replace generated build config with the release config')
    args = p.parse_args()
    if args.action == 'verify':
        verify()
    elif args.action == 'ports':
        print(json.dumps([{'port': x.device, 'serial': x.serial_number, 'description': x.description,
                           'vid': x.vid, 'pid': x.pid} for x in ports()], indent=2))
    elif args.action == 'flash':
        flash(args)
    else:
        build(args)


if __name__ == '__main__':
    main()
