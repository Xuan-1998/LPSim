#!/usr/bin/env python3
"""Export namespaced GTFS routes, stops, and scheduled connections to CSV."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lpsim_multimode.gtfs import GtfsFeed
from lpsim_multimode.modes import Mode


def _provider(identifier: str, fallback: str) -> str:
    return identifier.split(":", 1)[0] if ":" in identifier else fallback


def _gtfs_time(seconds: float) -> str:
    value = round(seconds)
    hours, remainder = divmod(value, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def export_supply(
    source: Path,
    output: Path,
    *,
    service_date=None,
    modes=None,
    provider_id: str = "feed",
):
    feed = GtfsFeed(source, service_date)
    selected_modes = set(modes or (Mode.BUS, Mode.METRO))
    connections = [
        connection
        for connection in feed.connections()
        if connection.mode in selected_modes
    ]
    used_routes = {connection.route_id for connection in connections}
    used_stops = {
        stop_id
        for connection in connections
        for stop_id in (connection.from_stop, connection.to_stop)
    }
    route_modes = {}
    for connection in connections:
        route_modes[connection.route_id] = connection.mode
    component_dates = {
        str(component["namespace"]): str(component["service_date"])
        for component in feed.components
        if component.get("namespace") and component.get("service_date")
    }
    default_date = feed.service_date.isoformat() if feed.service_date else ""

    output.mkdir(parents=True, exist_ok=True)
    with (output / "transit_stops.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(["provider_id", "stop_id", "stop_name", "lat", "lon"])
        for stop_id in sorted(used_stops):
            stop = feed.stops[stop_id]
            writer.writerow(
                [
                    _provider(stop_id, provider_id),
                    stop.stop_id,
                    stop.name,
                    stop.lat,
                    stop.lon,
                ]
            )

    with (output / "transit_routes.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "provider_id",
                "route_id",
                "route_short_name",
                "route_long_name",
                "mode",
            ]
        )
        for route_id in sorted(used_routes):
            row = feed.routes.get(route_id, {})
            writer.writerow(
                [
                    _provider(route_id, provider_id),
                    route_id,
                    row.get("route_short_name", ""),
                    row.get("route_long_name", ""),
                    route_modes[route_id].value,
                ]
            )

    with (output / "transit_connections.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "provider_id",
                "mode",
                "route_id",
                "service_id",
                "service_date",
                "trip_id",
                "sequence",
                "from_stop",
                "to_stop",
                "departure_time_s",
                "arrival_time_s",
                "departure_time_gtfs",
                "arrival_time_gtfs",
            ]
        )
        for connection in connections:
            provider = _provider(connection.trip_id, provider_id)
            writer.writerow(
                [
                    provider,
                    connection.mode.value,
                    connection.route_id,
                    connection.service_id,
                    component_dates.get(provider, default_date),
                    connection.trip_id,
                    connection.sequence,
                    connection.from_stop,
                    connection.to_stop,
                    connection.departure_time,
                    connection.arrival_time,
                    _gtfs_time(connection.departure_time),
                    _gtfs_time(connection.arrival_time),
                ]
            )

    summary = {
        "source": str(source),
        "output": str(output),
        "providers": len(
            {_provider(connection.trip_id, provider_id) for connection in connections}
        ),
        "stops": len(used_stops),
        "routes": len(used_routes),
        "connections": len(connections),
        "connections_by_mode": {
            mode.value: sum(connection.mode == mode for connection in connections)
            for mode in sorted(selected_modes, key=lambda value: value.value)
        },
    }
    (output / "transit_supply_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def _mode(value: str) -> Mode:
    mode = Mode(value.lower())
    if mode not in {Mode.BUS, Mode.METRO}:
        raise argparse.ArgumentTypeError("mode must be bus or metro")
    return mode


def _date(value: str) -> dt.date:
    return dt.date.fromisoformat(value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gtfs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", type=_mode, action="append", dest="modes")
    parser.add_argument(
        "--service-date",
        type=_date,
        help="Service date for a single feed; bundles retain component dates",
    )
    parser.add_argument(
        "--provider-id",
        default="feed",
        help="Provider ID for a non-bundle feed (bundles use their namespaces)",
    )
    args = parser.parse_args(argv)
    try:
        summary = export_supply(
            args.gtfs,
            args.output,
            service_date=args.service_date,
            modes=args.modes,
            provider_id=args.provider_id,
        )
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
