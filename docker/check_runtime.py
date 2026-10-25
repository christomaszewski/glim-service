"""Gate the runtime image: GLIM's compiled tree must load against exactly what this stage ships.

  - every ros-* package present in both stages has the version GLIM compiled against (cached apt
    layers from different days must not mix ABIs);
  - every ELF file under <prefix>, executables included, resolves all of its NEEDED libraries;
  - every shared library under <prefix>/lib loads with all symbols bound (RTLD_NOW), each in a
    fresh interpreter so one library's load cannot mask another's gap — the estimation modules are
    dlopen()ed by name at runtime, so a gap would otherwise surface only on the vehicle.
"""
import argparse
from pathlib import Path
import subprocess
import sys


def is_elf(path):
    with open(path, "rb") as stream:
        return stream.read(4) == b"\x7fELF"


def check_runtime(prefix, build_packages=None):
    errors = []
    if build_packages is not None:
        built = dict(line.split("\t") for line in build_packages.read_text().splitlines() if line)
        packages = subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Package}\t${Version}\t${db:Status-Status}\n", "ros-*"],
            text=True)
        for line in packages.splitlines():
            name, version, status = line.split("\t")
            if status == "installed" and name in built and built[name] != version:
                errors.append(f"{name}: build={built[name]}, runtime={version}")

    # Every ELF file (executables included) must resolve every NEEDED library in this stage.
    elves = sorted({p.resolve() for p in prefix.rglob("*") if p.is_file() and is_elf(p)})
    for elf in elves:
        result = subprocess.run(["ldd", str(elf)], capture_output=True, text=True, timeout=60)
        missing = [line.strip() for line in result.stdout.splitlines() if "not found" in line]
        if missing:
            errors.append(f"{elf}: unresolved " + "; ".join(missing))

    # versioned names and their dev symlinks resolve to one file; load each file once
    libraries = sorted({p.resolve() for p in (prefix / "lib").glob("lib*.so*") if p.is_file()})
    if not libraries:
        errors.append(f"no shared libraries found in {prefix / 'lib'}")
    for library in libraries:
        result = subprocess.run(
            [sys.executable, "-c", "import ctypes, sys; ctypes.CDLL(sys.argv[1])", str(library)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            errors.append(f"{library}: exit {result.returncode}\n{result.stderr.strip()}")
    if errors:
        print("ERROR: GLIM build/runtime check failed:", file=sys.stderr)
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"GLIM runtime check passed ({len(elves)} ELF files, {len(libraries)} shared libraries loaded)")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--build-packages", type=Path)
    args = parser.parse_args()
    sys.exit(check_runtime(args.prefix, args.build_packages))
