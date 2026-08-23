#!/usr/bin/env python3
"""Fetch versioned multimode supply inputs without committing large data files."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import ssl
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

MANIFEST = Path(__file__).resolve().parents[1] / "lpsim_multimode" / "sources.json"
GTFS_TABLES = {
    "agency.txt",
    "calendar.txt",
    "calendar_dates.txt",
    "feed_info.txt",
    "routes.txt",
    "shapes.txt",
    "stop_times.txt",
    "stops.txt",
    "transfers.txt",
    "trips.txt",
}
GTFS_REQUIRED_TABLES = {"routes.txt", "stops.txt", "stop_times.txt", "trips.txt"}


def _tls_context() -> ssl.SSLContext:
    """Use certifi when available, otherwise the platform trust store."""

    try:
        import certifi
    except ImportError:
        return ssl.create_default_context()
    return ssl.create_default_context(cafile=certifi.where())


def _curl_bytes(url: str) -> bytes:
    """Use curl's verified TLS stack when Python lacks an intermediate CA."""

    try:
        completed = subprocess.run(
            [
                "curl",
                "--fail",
                "--location",
                "--silent",
                "--show-error",
                "--connect-timeout",
                "15",
                "--max-time",
                "120",
                url,
            ],
            capture_output=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise OSError("TLS verification failed and curl is not installed") from exc
    if completed.returncode != 0:
        message = completed.stderr.decode("utf-8", errors="replace").strip()
        raise OSError(f"curl download failed: {message}")
    return completed.stdout


def download(url: str, target: Path, *, force: bool = False) -> None:
    if target.exists() and not force:
        print(f"exists, skipping: {target}")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".part")
    request = urllib.request.Request(
        url, headers={"User-Agent": "LPSim multimode data fetcher"}
    )
    try:
        try:
            with urllib.request.urlopen(
                request, context=_tls_context(), timeout=60
            ) as response, temporary.open("wb") as handle:
                shutil.copyfileobj(response, handle)
        except urllib.error.URLError as exc:
            if not isinstance(exc.reason, ssl.SSLCertVerificationError):
                raise
            temporary.write_bytes(_curl_bytes(url))
        temporary.replace(target)
    finally:
        if temporary.exists():
            temporary.unlink()
    print(f"downloaded: {target}")


def fetch_source(
    name: str, source: dict[str, object], output: Path, force: bool
) -> None:
    if source.get("manual"):
        raise ValueError(
            f"{name} requires manual acceptance/download from {source['landing_page']}"
        )
    target = output / str(source["target"])
    if source.get("ckan_api") and target.is_dir() and not force:
        _validate_gtfs(target)
        print(f"exists, skipping: {target}")
        return
    if source.get("google_drive_id"):
        file_id = urllib.parse.quote(str(source["google_drive_id"]))
        download(
            f"https://drive.usercontent.google.com/download?id={file_id}&export=download&confirm=t",
            target,
            force=force,
        )
    elif source.get("ckan_api"):
        _fetch_ckan_gtfs(source, target, force)
    elif source.get("token_env"):
        token_name = str(source["token_env"])
        token = os.environ.get(token_name)
        if not token:
            raise ValueError(f"{name} requires the {token_name} environment variable")
        query = urllib.parse.urlencode(
            {"api_key": token, "operator_id": source.get("operator_id", "RG")}
        )
        download(f"{source['api']}?{query}", target, force=force)
    elif source.get("url"):
        download(str(source["url"]), target, force=force)
    else:
        raise ValueError(f"{name} has no automated acquisition method")
    if source.get("expected_size_bytes") and target.is_file():
        expected = int(source["expected_size_bytes"])
        actual = target.stat().st_size
        if actual != expected:
            raise ValueError(
                f"download size mismatch for {target}: expected {expected}, got {actual}"
            )
    if source.get("data_format") == "gtfs":
        _validate_gtfs(target)


def _validate_gtfs(target: Path) -> None:
    if target.is_dir():
        names = {path.name for path in target.iterdir() if path.is_file()}
    else:
        try:
            with zipfile.ZipFile(target) as archive:
                names = {Path(name).name for name in archive.namelist()}
        except zipfile.BadZipFile as exc:
            raise ValueError(f"invalid GTFS zip archive: {target}") from exc
    missing = GTFS_REQUIRED_TABLES - names
    if missing:
        raise ValueError(
            f"GTFS input {target} is missing: {', '.join(sorted(missing))}"
        )


def _fetch_ckan_gtfs(source: dict[str, object], target: Path, force: bool) -> None:
    query = urllib.parse.urlencode({"id": source["ckan_package"]})
    request = urllib.request.Request(f"{source['ckan_api']}?{query}")
    try:
        with urllib.request.urlopen(
            request, context=_tls_context(), timeout=60
        ) as response:
            package = json.load(response)["result"]
    except urllib.error.URLError as exc:
        if not isinstance(exc.reason, ssl.SSLCertVerificationError):
            raise
        package = json.loads(_curl_bytes(request.full_url))["result"]
    target.mkdir(parents=True, exist_ok=True)
    found = set()
    for resource in package["resources"]:
        name = str(resource.get("name") or "").lower()
        url_name = Path(urllib.parse.urlparse(resource["url"]).path).name.lower()
        normalized = re.sub(r"[^a-z0-9]+", "_", f"{name}_{url_name}").strip("_")
        filename = next(
            (
                table
                for table in sorted(GTFS_TABLES, key=len, reverse=True)
                if re.search(
                    rf"(?:^|_){re.escape(table.replace('.txt', ''))}(?:_|txt|csv|$)",
                    normalized,
                )
            ),
            None,
        )
        if filename and filename not in found:
            download(resource["url"], target / filename, force=force)
            found.add(filename)
    if not GTFS_REQUIRED_TABLES.issubset(found):
        missing = ", ".join(sorted(GTFS_REQUIRED_TABLES - found))
        raise ValueError(
            f"AC Transit CKAN package did not expose required tables: {missing}"
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", nargs="*", help="Source names from sources.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--list", action="store_true")
    parser.add_argument(
        "--group", action="append", default=[], help="Source group from sources.json"
    )
    parser.add_argument("--include-large", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    catalog = json.loads(MANIFEST.read_text(encoding="utf-8"))
    sources = catalog["sources"]
    groups = catalog.get("source_groups", {})
    if args.list:
        for name, source in sources.items():
            print(f"{name}\t{','.join(source['modes'])}\t{source['provider']}")
        for name, group in groups.items():
            print(
                f"group:{name}\t{len(group['sources'])} sources\t{group['description']}"
            )
        return 0
    selected = list(args.sources)
    for group_name in args.group:
        if group_name not in groups:
            print(f"unknown source group: {group_name}", file=sys.stderr)
            return 1
        selected.extend(groups[group_name]["sources"])
    if not selected:
        selected = [
            name
            for name, source in sources.items()
            if not source.get("large")
            and not source.get("manual")
            and not source.get("token_env")
            and not source.get("group_only")
        ]
    selected = list(dict.fromkeys(selected))
    failures = 0
    for name in selected:
        if name not in sources:
            print(f"unknown source: {name}", file=sys.stderr)
            failures += 1
            continue
        source = sources[name]
        if source.get("large") and not args.include_large:
            print(
                f"skipping large source {name}; pass --include-large", file=sys.stderr
            )
            continue
        try:
            fetch_source(name, source, args.output, args.force)
        except (OSError, ValueError) as exc:
            print(f"{name}: {exc}", file=sys.stderr)
            failures += 1
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
