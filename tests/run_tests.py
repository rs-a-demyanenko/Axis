"""Run native C++ tests against the real, pinned GyverMotor2 header.

Python 3 standard library only. Optional portable Zig stays inside .tools/.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / '.build'
GYVER_COMMIT = '29972794183db1d92df00ef0e71a38dcac48545d'


def dependency():
    target = BUILD / 'deps' / 'GyverMotor2.h'
    target.parent.mkdir(parents=True, exist_ok=True)
    url = f'https://raw.githubusercontent.com/GyverLibs/GyverMotor2/{GYVER_COMMIT}/GyverMotor2.h'
    if not target.exists():
        print('Downloading GyverMotor2 at', GYVER_COMMIT, flush=True)
        target.write_bytes(urllib.request.urlopen(url, timeout=60).read())
    return target.parent


def compiler(download):
    for name in ('g++', 'clang++'):
        found = shutil.which(name)
        if found:
            return [found]
    zig = shutil.which('zig')
    portable = ROOT / '.tools' / 'zig-windows-x86_64-0.13.0' / 'zig.exe'
    if not zig and portable.exists():
        zig = str(portable)
    if not zig and download and sys.platform == 'win32':
        index = json.load(urllib.request.urlopen('https://ziglang.org/download/index.json', timeout=60))
        release = index['0.13.0']['x86_64-windows']
        archive = urllib.request.urlopen(release['tarball'], timeout=120).read()
        if hashlib.sha256(archive).hexdigest() != release['shasum']:
            raise RuntimeError('Zig archive checksum mismatch')
        tools_dir = ROOT / '.tools'
        tools_dir.mkdir(exist_ok=True)
        archive_path = tools_dir / 'zig.zip'
        archive_path.write_bytes(archive)
        with zipfile.ZipFile(archive_path) as package:
            for name in package.namelist():
                if not (tools_dir / name).resolve().is_relative_to(tools_dir.resolve()):
                    raise RuntimeError('Unsafe archive path')
            package.extractall(tools_dir)
        archive_path.unlink()
        zig = str(portable)
    if zig:
        return [zig, 'c++']
    raise RuntimeError('Install g++/clang++/Zig, or use --download-compiler on Windows')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--download-compiler', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--no-motor-accel', action='store_true')
    parser.add_argument('--sanitize', action='store_true', help='Undefined behavior sanitizer (GCC/Clang)')
    args = parser.parse_args()
    deps = dependency()
    cc = compiler(args.download_compiler)
    print('Compiler:', cc, flush=True)
    if args.prepare_only:
        return
    output = BUILD / ('axis_tests.exe' if os.name == 'nt' else 'axis_tests')
    command = cc + ['-std=c++11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-I' + str(ROOT / 'src'), '-I' + str(ROOT / 'tests' / 'support'),
                    '-isystem', str(deps), str(ROOT / 'tests' / 'test_axis.cpp'),
                    str(ROOT / 'src' / 'AxisCore.cpp'),
                    '-o', str(output)]
    if args.no_motor_accel:
        command += ['-DGMOTOR2_NO_ACCEL']
    if args.sanitize:
        command += ['-fsanitize=undefined', '-fno-sanitize-recover=all']
    environment = os.environ.copy()
    environment['ZIG_GLOBAL_CACHE_DIR'] = str(BUILD / 'zig-global-cache')
    environment['ZIG_LOCAL_CACHE_DIR'] = str(BUILD / 'zig-cache')
    subprocess.run(command, cwd=BUILD, env=environment, check=True)
    subprocess.run([str(output)], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
