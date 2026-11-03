"""Create an Arduino-installable ZIP without caches, tools or Git metadata."""
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[1]
version = json.loads((root / 'library.json').read_text())['version']
destination = root / 'dist' / ('Axis-' + version + '.zip')
destination.parent.mkdir(exist_ok=True)
files = [root / name for name in ('README.md', 'AXIS_PLAN.md', 'library.properties',
                                 'library.json', 'keywords.txt')]
for directory in ('src', 'examples', 'docs', 'tests', 'tools', '.github'):
    files.extend(path for path in (root / directory).rglob('*')
                 if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc')
with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(files):
        archive.write(path, 'Axis/' + path.relative_to(root).as_posix())
print(destination)
