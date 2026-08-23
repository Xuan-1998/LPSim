#!/usr/bin/env python3
"""Build a namespaced, date-audited multi-provider GTFS bundle for LPSim."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lpsim_multimode.gtfs import GtfsFeed, route_type_mode

MANIFEST = ROOT / "lpsim_multimode" / "sources.json"
DEFAULT_GROUP = "bay_area_fixed_route_gtfs"


def build_bundle(
    *, raw_root: Path, output: Path, group_name: str, scenario_date: dt.date
) -> dict[str, object]:
    catalog = json.loads(MANIFEST.read_text(encoding="utf-8"))
    try:
        group = catalog["source_groups"][group_name]
    except KeyError as exc:
        raise ValueError(f"unknown GTFS source group: {group_name}") from exc

    output = output.resolve()
    missing_files = []
    components = []
    covered_regions: set[str] = set()
    actual_modes: Counter[str] = Counter()
    for source_name in group["sources"]:
        source = catalog["sources"][source_name]
        source_path = (raw_root / source["target"]).resolve()
        if not source_path.exists():
            missing_files.append(f"{source_name}: {source_path}")
            continue
        unfiltered = GtfsFeed(source_path)
        representative_date = unfiltered.representative_service_date(scenario_date)
        if representative_date is None:
            raise ValueError(
                f"{source_name} has no usable {scenario_date:%A} service date"
            )
        active = GtfsFeed(source_path, representative_date)
        route_modes: dict[str, str] = {}
        for trip in active.trips.values():
            route_id = trip["route_id"]
            route = active.routes.get(route_id, {})
            mode = route_type_mode(route.get("route_type", "3"))
            if mode is not None:
                route_modes[route_id] = mode.value
        mode_counts = Counter(route_modes.values())
        actual_modes.update(mode_counts)
        regions = [str(value) for value in source.get("regions", [])]
        covered_regions.update(regions)
        components.append(
            {
                "source_name": source_name,
                "namespace": source["namespace"],
                "provider": source["provider"],
                "regions": regions,
                "declared_modes": source["modes"],
                "path": os.path.relpath(source_path, output),
                "service_date": representative_date.isoformat(),
                "service_age_days": (scenario_date - representative_date).days,
                "stale_snapshot": (scenario_date - representative_date).days > 366,
                "stops": len(active.stops),
                "trips": len(active.trips),
                "active_routes_by_mode": dict(sorted(mode_counts.items())),
                "license_url": source.get("license_url"),
                "source_url": source.get("url") or source.get("landing_page"),
            }
        )

    if missing_files:
        raise FileNotFoundError(
            "missing GTFS inputs; fetch the group first:\n" + "\n".join(missing_files)
        )
    expected_regions = set(group.get("expected_regions", []))
    missing_regions = expected_regions - covered_regions
    if missing_regions:
        raise ValueError(
            "GTFS group does not cover required regions: "
            + ", ".join(sorted(missing_regions))
        )
    required_modes = {"bus", "metro"}
    missing_modes = required_modes - set(actual_modes)
    if missing_modes:
        raise ValueError(
            "GTFS group has no active routes for modes: "
            + ", ".join(sorted(missing_modes))
        )

    result: dict[str, object] = {
        "schema_version": 1,
        "name": group_name,
        "description": group["description"],
        "scenario_date": scenario_date.isoformat(),
        "date_policy": (
            "Each provider uses its latest available service day with the same "
            "weekday at or before scenario_date; service_date and age are retained."
        ),
        "expected_regions": sorted(expected_regions),
        "covered_regions": sorted(covered_regions),
        "active_routes_by_mode": dict(sorted(actual_modes.items())),
        "components": components,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "bundle.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def _date(value: str) -> dt.date:
    return dt.date.fromisoformat(value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--group", default=DEFAULT_GROUP)
    parser.add_argument("--scenario-date", type=_date, required=True)
    args = parser.parse_args(argv)
    try:
        result = build_bundle(
            raw_root=args.raw_root,
            output=args.output,
            group_name=args.group,
            scenario_date=args.scenario_date,
        )
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1
    summary = {
        "bundle": str(args.output / "bundle.json"),
        "components": len(result["components"]),
        "covered_regions": result["covered_regions"],
        "active_routes_by_mode": result["active_routes_by_mode"],
        "stale_components": [
            component["source_name"]
            for component in result["components"]
            if component["stale_snapshot"]
        ],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
