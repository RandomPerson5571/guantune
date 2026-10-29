"""Install upstream dependencies into a new, disposable PROS build project."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="New build directory (must not exist)")
    parser.add_argument("--kernel-version", default="latest", help="PROS kernel version; default: latest")
    parser.add_argument("--lemlib-version", help="Optional LemLib version; default: Conductor's compatible stable release")
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1]
    destination = args.destination.expanduser().resolve()
    if destination.exists():
        parser.error("Destination already exists; choose a new directory to preserve existing files.")
    if source == destination or source in destination.parents:
        parser.error("Choose a build directory outside the library source checkout.")
    pros = shutil.which("pros")
    if not pros:
        parser.error("PROS CLI is not installed or is not on PATH. Install the official PROS toolchain first.")

    # Conductor downloads the actual kernel and LemLib headers/archives. Nothing
    # here redefines their namespaces, types, or implementations.
    subprocess.run([pros, "conduct", "new-project", str(destination), "v5",
                    args.kernel_version, "--no-default-libs"], check=True)
    if not (destination / "project.pros").is_file():
        raise SystemExit("PROS did not create a project; inspect CLI diagnostics (including its version detection).")
    subprocess.run([pros, "conduct", "add-depot", "LemLib",
                    "https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json"],
                   cwd=destination, check=True)
    query = "LemLib" if not args.lemlib_version else "LemLib@" + args.lemlib_version
    subprocess.run([pros, "conduct", "apply", query], cwd=destination, check=True)
    if not (destination / "common.mk").is_file() or not (destination / "include/lemlib/api.hpp").is_file():
        raise SystemExit("Dependency installation did not supply the expected PROS/LemLib files.")

    for filename in ("main.c", "main.cc", "main.cpp"):
        (destination / "src" / filename).unlink(missing_ok=True)
    (destination / "static/example.txt").unlink(missing_ok=True)
    for directory in ("include/autotune", "src/autotune"):
        shutil.copytree(source / directory, destination / directory)
    # project.pros is jsonpickle JSON; the version fields remain ordinary strings.
    project = json.loads((destination / "project.pros").read_text())
    project = project.get("py/state", project)
    versions = {name: entry["version"] for name, entry in project["templates"].items()}
    (destination / "dependency-versions.json").write_text(json.dumps(versions, indent=2) + "\n")
    version = versions["LemLib"]
    if version != "0.5.6":
        print("WARNING: LemLib", version, "is unvalidated; CI is probing compatibility.")
    parts = version.split(".")
    numeric = int(parts[0]) * 10000 + int(parts[1]) * 100 + int(parts[2].split("-")[0])
    (destination / "include/autotune/dependency_version.hpp").write_text(
        "#pragma once\n#define GUANTUNE_LEMLIB_VERSION " + str(numeric) + "\n")
    shutil.copytree(source / "docs", destination / "docs")
    (destination / "scripts").mkdir(exist_ok=True)
    shutil.copy2(source / "scripts/optimize_template.py", destination / "scripts/optimize_template.py")
    for filename in ("Makefile", "README.md", "LICENSE", "CHANGELOG.md", "CONTRIBUTING.md"):
        shutil.copy2(source / filename, destination / filename)
    print("Build project ready:", destination)
    print("Run 'pros make library' and 'pros make template' from that directory.")


if __name__ == "__main__":
    main()
