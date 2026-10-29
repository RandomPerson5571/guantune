"""Shrink the distributable template without changing the local debug archive."""

import argparse
from pathlib import Path
import subprocess
import tempfile
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--strip", default="arm-none-eabi-strip")
    args = parser.parse_args()
    archive = args.archive.resolve()
    original_size = archive.stat().st_size
    # Replace only after both stripping and ZIP creation succeed. Keep symbols
    # and relocations needed by consumer links; --strip-all is not appropriate.
    with tempfile.TemporaryDirectory(dir=archive.parent) as directory:
        directory = Path(directory)
        library = directory / "autotune.a"
        output = directory / archive.name
        with zipfile.ZipFile(archive) as source:
            library.write_bytes(source.read("firmware/autotune.a"))
            subprocess.run([args.strip, "--strip-debug", str(library)], check=True)
            with zipfile.ZipFile(output, "w") as destination:
                for entry in source.infolist():
                    data = library.read_bytes() if entry.filename == "firmware/autotune.a" else source.read(entry)
                    destination.writestr(entry, data, compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)
        output.replace(archive)
    print(f"Template optimized: {original_size:,} -> {archive.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()
