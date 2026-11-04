"""Compile/link examples with installed Arduino cores; never upload firmware."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from run_tests import ROOT, BUILD, dependency


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cli', default=shutil.which('arduino-cli') or 'arduino-cli')
    parser.add_argument('--config')
    parser.add_argument('--properties-json', help='Optional list of extra CLI arguments')
    parser.add_argument('--target', choices=('avr', 'esp32', 'esp32s3', 'all'), default='all')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--log', help='Save full compiler output, print summaries only')
    args = parser.parse_args()
    deps = dependency()
    extra = json.loads(Path(args.properties_json).read_text()) if args.properties_json else []
    if args.log:
        Path(args.log).write_text('', encoding='utf-8')
    matrix = [('avr', 'arduino:avr:uno', 'Linear_AVR'),
              ('avr', 'arduino:avr:nano', 'Linear_AVR'),
              ('avr', 'arduino:avr:uno', 'ExternalFeedback'),
              ('avr', 'arduino:avr:uno', 'SCurvePlotter'),
              ('avr', 'arduino:avr:mega', 'SynchronizedXY_Mega'),
              ('esp32', 'esp32:esp32:esp32', 'Linear_ESP32'),
              ('esp32', 'esp32:esp32:esp32', 'Rotary_ESP32'),
              ('esp32', 'esp32:esp32:esp32', 'ExternalFeedback'),
              ('esp32', 'esp32:esp32:esp32', 'SCurvePlotter'),
              ('esp32', 'esp32:esp32:esp32', 'SynchronizedXY_ESP32'),
              ('esp32', 'esp32:esp32:esp32', 'SpindleSync_ESP32'),
              ('esp32', 'esp32:esp32:esp32', 'RotaryGear_ESP32'),
              ('esp32s3', 'esp32:esp32:esp32s3', 'ExternalFeedback'),
              ('esp32s3', 'esp32:esp32:esp32s3', 'SpindleSync_ESP32')]
    for target, fqbn in (('avr', 'arduino:avr:uno'), ('esp32', 'esp32:esp32:esp32'),
                         ('esp32s3', 'esp32:esp32:esp32s3')):
        matrix.extend((target, fqbn, sketch) for sketch in ('Stepper_OpenLoop', 'Stepper_Encoder'))
    for target, fqbn, sketch in matrix:
        if args.target != 'all' and target != args.target:
            continue
        boardSuffix = '-nano' if fqbn == 'arduino:avr:nano' else ''
        command = [args.cli, 'compile', '--fqbn', fqbn, '--library', str(ROOT),
                   '--library', str(deps), '--build-path', str(BUILD / (target + '-' + sketch + boardSuffix)),
                   '--warnings', 'all']
        if args.config:
            command += ['--config-file', args.config]
        if args.verbose:
            command += ['--verbose']
        command += extra + [str(ROOT / 'examples' / sketch)]
        print('Building', fqbn, sketch, flush=True)
        if not args.log:
            subprocess.run(command, cwd=ROOT, check=True)
        else:
            result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
            with Path(args.log).open('a', encoding='utf-8') as log:
                log.write(f'Building {fqbn} {sketch}\n' + result.stdout + '\n')
            for line in result.stdout.splitlines():
                if line.startswith(('Sketch uses ', 'Global variables use ')):
                    print(line, flush=True)
            if result.returncode:
                print(result.stdout, flush=True)
                result.check_returncode()
    print('PASS: selected examples compiled and linked', flush=True)


if __name__ == '__main__':
    main()
