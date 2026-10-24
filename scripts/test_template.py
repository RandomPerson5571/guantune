"""Install a built template into a new consumer project and link a smoke program."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    destination = args.destination.resolve()
    archive = args.archive.resolve()
    if destination.exists():
        parser.error('Consumer destination must not exist')
    versions = json.loads((archive.parent / 'dependency-versions.json').read_text())
    with zipfile.ZipFile(archive) as package:
        names = package.namelist()
        if any(name.startswith(('include/pros/', 'include/lemlib/', 'include/fmt/')) for name in names):
            raise SystemExit('Template must not bundle dependency headers')
        if 'firmware/autotune.a' not in names:
            raise SystemExit('Template is missing its library archive')
    pros = shutil.which('pros')
    if not pros:
        parser.error('PROS CLI must be on PATH')

    def run(*command):
        subprocess.run([pros, *command], cwd=destination, check=True)

    subprocess.run([pros, 'conduct', 'new-project', str(destination), 'v5',
                    versions['kernel'], '--no-default-libs'], check=True)
    run('conduct', 'add-depot', 'LemLib',
        'https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json')
    run('conduct', 'apply', 'LemLib@' + versions['LemLib'])
    run('conduct', 'fetch', str(archive))
    run('conduct', 'apply', 'autotune@0.1.0')
    for name in ('main.c', 'main.cc', 'main.cpp'):
        (destination / 'src' / name).unlink(missing_ok=True)
    # This fixture is compile/link only, never uploaded to a robot. The null guard
    # prevents movement even if someone accidentally uploads the smoke program.
    (destination / 'src/main.cpp').write_text('''#include "autotune/api.hpp"
lemlib::Chassis* validationChassis = nullptr;
void initialize() {}
void disabled() {}
void competition_initialize() {}
void opcontrol() {}
void autonomous() {
    if (!validationChassis) return;
    auto result = guantune::tuneLateral(*validationChassis);
    (void)guantune::saveAcceptedGains(result, "/usd/guantune.json");
    auto snippet = guantune::controllerSettingsSnippet(result);
    (void)snippet;
}
''')
    run('make', 'all', 'USE_PACKAGE=0',
        'EXTRA_CXXFLAGS=-Iinclude -DGUANTUNE_ALLOW_UNTESTED_LEMLIB')
    if not (destination / 'bin/monolith.bin').is_file():
        raise SystemExit('PROS did not produce a consumer binary')


if __name__ == '__main__':
    main()
