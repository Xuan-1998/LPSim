#!/usr/bin/env python3
"""Validate normalized OD trips, candidate options, and itinerary legs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lpsim_multimode.modes import Mode

ENDPOINT_TYPES = {"road_node", "gtfs_stop", "airport", "coordinate"}
MODE_VALUES = {mode.value for mode in Mode}

TRIP_FIELDS = {
    "trip_id",
    "dep_time",
    "origin_type",
    "origin_id",
    "destination_type",
    "destination_id",
    "passengers",
}
OPTION_FIELDS = {
    "trip_id",
    "option_id",
    "mode_sequence",
    "available",
    "chosen",
    "probability",
    "estimated_time_s",
    "estimated_cost_usd",
    "transfer_count",
    "data_status",
}
LEG_FIELDS = {
    "trip_id",
    "option_id",
    "leg_index",
    "mode",
    "from_type",
    "from_id",
    "to_type",
    "to_id",
    "provider_id",
    "route_id",
    "service_trip_id",
    "aircraft_code",
    "departure_time_s",
    "arrival_time_s",
    "distance_m",
    "cost_usd",
}


def _read(path: Path, required_fields: set[str], errors: list[str]):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        missing = required_fields - fields
        if missing:
            errors.append(f"{path}: missing columns: {', '.join(sorted(missing))}")
            return []
        rows = []
        for line, row in enumerate(reader, start=2):
            if None in row:
                errors.append(f"{path}:{line}: row has more values than columns")
                continue
            row["_line"] = str(line)
            rows.append(row)
        return rows


def _required(row, field: str, table: str, errors: list[str]) -> str:
    value = (row.get(field) or "").strip()
    if not value:
        errors.append(f"{table}:{row['_line']}: {field} is required")
    return value


def _number(
    row,
    field: str,
    table: str,
    errors: list[str],
    *,
    required: bool = False,
    minimum: float = 0.0,
):
    text = (row.get(field) or "").strip()
    if not text:
        if required:
            errors.append(f"{table}:{row['_line']}: {field} is required")
        return None
    try:
        value = float(text)
    except ValueError:
        errors.append(f"{table}:{row['_line']}: {field} must be numeric")
        return None
    if not math.isfinite(value) or value < minimum:
        errors.append(f"{table}:{row['_line']}: {field} must be >= {minimum:g}")
    return value


def _integer(
    row,
    field: str,
    table: str,
    errors: list[str],
    *,
    required: bool = False,
    minimum: int = 0,
):
    value = _number(
        row, field, table, errors, required=required, minimum=float(minimum)
    )
    if value is not None and not value.is_integer():
        errors.append(f"{table}:{row['_line']}: {field} must be an integer")
        return None
    return int(value) if value is not None else None


def _boolean(row, field: str, table: str, errors: list[str]):
    text = _required(row, field, table, errors).lower()
    if text in {"1", "true"}:
        return True
    if text in {"0", "false"}:
        return False
    if text:
        errors.append(f"{table}:{row['_line']}: {field} must be 0/1 or true/false")
    return None


def validate_files(trips_path: Path, options_path: Path, legs_path: Path):
    """Return a summary and all validation errors without mutating inputs."""

    errors: list[str] = []
    trips = _read(trips_path, TRIP_FIELDS, errors)
    options = _read(options_path, OPTION_FIELDS, errors)
    legs = _read(legs_path, LEG_FIELDS, errors)

    trip_by_id = {}
    for row in trips:
        trip_id = _required(row, "trip_id", "od_trips", errors)
        if trip_id in trip_by_id:
            errors.append(f"od_trips:{row['_line']}: duplicate trip_id {trip_id!r}")
        trip_by_id[trip_id] = row
        _number(row, "dep_time", "od_trips", errors, required=True)
        _integer(row, "passengers", "od_trips", errors, required=True, minimum=1)
        for field in ("origin_type", "destination_type"):
            value = _required(row, field, "od_trips", errors)
            if value and value not in ENDPOINT_TYPES:
                errors.append(f"od_trips:{row['_line']}: invalid {field} {value!r}")
        _required(row, "origin_id", "od_trips", errors)
        _required(row, "destination_id", "od_trips", errors)

    option_by_key = {}
    options_by_trip = defaultdict(list)
    for row in options:
        trip_id = _required(row, "trip_id", "od_mode_options", errors)
        option_id = _required(row, "option_id", "od_mode_options", errors)
        key = (trip_id, option_id)
        if key in option_by_key:
            errors.append(f"od_mode_options:{row['_line']}: duplicate option {key!r}")
        option_by_key[key] = row
        options_by_trip[trip_id].append(row)
        if trip_id not in trip_by_id:
            errors.append(
                f"od_mode_options:{row['_line']}: unknown trip_id {trip_id!r}"
            )
        modes = _required(row, "mode_sequence", "od_mode_options", errors).split(">")
        if not modes or any(mode not in MODE_VALUES for mode in modes):
            errors.append(f"od_mode_options:{row['_line']}: invalid mode_sequence")
        row["_modes"] = modes
        row["_available"] = _boolean(row, "available", "od_mode_options", errors)
        row["_chosen"] = _boolean(row, "chosen", "od_mode_options", errors)
        if row["_chosen"] and not row["_available"]:
            errors.append(
                f"od_mode_options:{row['_line']}: chosen option must be available"
            )
        probability = _number(row, "probability", "od_mode_options", errors, minimum=0)
        if probability is not None and probability > 1:
            errors.append(f"od_mode_options:{row['_line']}: probability must be <= 1")
        row["_probability"] = probability
        _number(row, "estimated_time_s", "od_mode_options", errors)
        _number(row, "estimated_cost_usd", "od_mode_options", errors)
        _integer(row, "transfer_count", "od_mode_options", errors)
        if _required(row, "data_status", "od_mode_options", errors) not in {
            "candidate",
            "routed",
            "simulated",
        }:
            errors.append(f"od_mode_options:{row['_line']}: invalid data_status")

    for trip_id, rows in options_by_trip.items():
        chosen = [row for row in rows if row.get("_chosen")]
        if len(chosen) > 1:
            errors.append(
                f"od_mode_options: trip {trip_id!r} has multiple chosen options"
            )
        available = [row for row in rows if row.get("_available")]
        probabilities = [row.get("_probability") for row in available]
        if (
            available
            and all(value is not None for value in probabilities)
            and not math.isclose(sum(probabilities), 1.0, abs_tol=1e-6)
        ):
            errors.append(
                f"od_mode_options: probabilities for trip {trip_id!r} do not sum to 1"
            )
    for trip_id in trip_by_id:
        if trip_id not in options_by_trip:
            errors.append(f"od_mode_options: trip {trip_id!r} has no candidate options")

    legs_by_option = defaultdict(list)
    for row in legs:
        trip_id = _required(row, "trip_id", "od_mode_legs", errors)
        option_id = _required(row, "option_id", "od_mode_legs", errors)
        key = (trip_id, option_id)
        if key not in option_by_key:
            errors.append(f"od_mode_legs:{row['_line']}: unknown option {key!r}")
        index = _integer(
            row, "leg_index", "od_mode_legs", errors, required=True, minimum=0
        )
        row["_index"] = index
        mode = _required(row, "mode", "od_mode_legs", errors)
        if mode and mode not in MODE_VALUES:
            errors.append(f"od_mode_legs:{row['_line']}: invalid mode {mode!r}")
        for field in ("from_type", "to_type"):
            value = _required(row, field, "od_mode_legs", errors)
            if value and value not in ENDPOINT_TYPES:
                errors.append(f"od_mode_legs:{row['_line']}: invalid {field} {value!r}")
        _required(row, "from_id", "od_mode_legs", errors)
        _required(row, "to_id", "od_mode_legs", errors)
        departure = _number(row, "departure_time_s", "od_mode_legs", errors)
        arrival = _number(row, "arrival_time_s", "od_mode_legs", errors)
        if departure is not None and arrival is not None and arrival < departure:
            errors.append(
                f"od_mode_legs:{row['_line']}: arrival_time_s precedes departure_time_s"
            )
        _number(row, "distance_m", "od_mode_legs", errors)
        _number(row, "cost_usd", "od_mode_legs", errors)
        legs_by_option[key].append(row)

    for key, option in option_by_key.items():
        rows = legs_by_option.get(key, [])
        if not rows:
            errors.append(f"od_mode_legs: option {key!r} has no legs")
            continue
        if any(row.get("_index") is None for row in rows):
            continue
        rows.sort(key=lambda row: row["_index"])
        indexes = [row["_index"] for row in rows]
        if indexes != list(range(len(rows))):
            errors.append(f"od_mode_legs: option {key!r} leg_index is not contiguous")
        if [row["mode"] for row in rows] != option.get("_modes"):
            errors.append(f"od_mode_legs: option {key!r} does not match mode_sequence")
        trip = trip_by_id.get(key[0])
        if trip:
            start = (rows[0]["from_type"], rows[0]["from_id"])
            end = (rows[-1]["to_type"], rows[-1]["to_id"])
            expected_start = (trip["origin_type"], trip["origin_id"])
            expected_end = (trip["destination_type"], trip["destination_id"])
            if start != expected_start:
                errors.append(
                    f"od_mode_legs: option {key!r} starts at {start!r}, not OD origin"
                )
            if end != expected_end:
                errors.append(
                    f"od_mode_legs: option {key!r} ends at {end!r}, not OD destination"
                )
        for first, second in zip(rows, rows[1:]):
            if (first["to_type"], first["to_id"]) != (
                second["from_type"],
                second["from_id"],
            ):
                errors.append(f"od_mode_legs: option {key!r} has disconnected legs")

    summary = {
        "trips": len(trips),
        "options": len(options),
        "legs": len(legs),
        "errors": len(errors),
    }
    return summary, errors


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trips", type=Path, required=True)
    parser.add_argument("--options", type=Path, required=True)
    parser.add_argument("--legs", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        summary, errors = validate_files(args.trips, args.options, args.legs)
    except OSError as exc:
        print(exc, file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
