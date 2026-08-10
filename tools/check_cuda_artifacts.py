#!/usr/bin/env python3
"""Verify the native cubins and PTX images embedded in an LPSim binary."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path
from typing import Iterable


_ARCHITECTURE_PATTERN = re.compile(r"(?:sm|compute)_([0-9]{2,3})(?:[a-z])?")


def parse_architectures(cuobjdump_output: str) -> set[int]:
    """Return CUDA architecture numbers found in cuobjdump listing output."""
    return {
        int(match.group(1))
        for match in _ARCHITECTURE_PATTERN.finditer(cuobjdump_output)
    }


def inspect_binary(
    binary: Path, cuobjdump: str = "cuobjdump"
) -> tuple[set[int], set[int]]:
    """Return (native cubin architectures, PTX architectures) for *binary*."""
    listings = []
    for option in ("--list-elf", "--list-ptx"):
        result = subprocess.run(
            [cuobjdump, option, str(binary)],
            check=True,
            capture_output=True,
            text=True,
        )
        listings.append(parse_architectures(result.stdout + result.stderr))
    return listings[0], listings[1]


def missing_architectures(found: Iterable[int], required: Iterable[int]) -> set[int]:
    """Return required architectures that were not found."""
    return set(required) - set(found)


def _format_architectures(architectures: Iterable[int], prefix: str = "sm") -> str:
    values = sorted(architectures)
    return ", ".join(f"{prefix}_{value}" for value in values) if values else "none"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="LPSim executable or CUDA object")
    parser.add_argument(
        "--cuobjdump",
        default="cuobjdump",
        help="cuobjdump executable (default: cuobjdump)",
    )
    parser.add_argument(
        "--require-real",
        type=int,
        nargs="*",
        default=[],
        metavar="SM",
        help="required native cubin architectures, such as 80 89 90 100",
    )
    parser.add_argument(
        "--require-virtual",
        type=int,
        nargs="*",
        default=[],
        metavar="COMPUTE",
        help="required embedded PTX architectures, such as 80 100",
    )
    args = parser.parse_args()

    if not args.binary.is_file():
        parser.error(f"binary does not exist: {args.binary}")

    try:
        native, ptx = inspect_binary(args.binary, args.cuobjdump)
    except FileNotFoundError:
        parser.error(f"cuobjdump was not found: {args.cuobjdump}")
    except subprocess.CalledProcessError as error:
        parser.error(f"cuobjdump failed with exit code {error.returncode}")

    print(f"Native cubins: {_format_architectures(native)}")
    print(f"Embedded PTX:  {_format_architectures(ptx, 'compute')}")

    missing_native = missing_architectures(native, args.require_real)
    missing_ptx = missing_architectures(ptx, args.require_virtual)
    if missing_native or missing_ptx:
        if missing_native:
            print(f"Missing native cubins: {_format_architectures(missing_native)}")
        if missing_ptx:
            print(f"Missing PTX: {_format_architectures(missing_ptx, 'compute')}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
