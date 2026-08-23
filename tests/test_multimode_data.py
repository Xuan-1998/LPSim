import csv
import datetime as dt
import json
from pathlib import Path

from lpsim_multimode.gtfs import (
    GtfsFeed,
    parse_gtfs_time,
    resolve_stop_times,
    route_type_mode,
)
from lpsim_multimode.modes import Mode
from tools import build_gtfs_bundle
from tools.export_gtfs_supply import export_supply
from tools.validate_mode_choice_inputs import validate_files

ROOT = Path(__file__).resolve().parents[1]


def _gtfs(parent: Path) -> Path:
    feed = parent / "gtfs"
    feed.mkdir()
    (feed / "stops.txt").write_text(
        "stop_id,stop_name,stop_lat,stop_lon\n"
        "A,Alpha,37.0,-122.0\nB,Beta,37.0,-121.99\nC,Gamma,37.0,-121.98\n",
        encoding="utf-8",
    )
    (feed / "routes.txt").write_text(
        "route_id,route_short_name,route_type\nBUS,10,3\nRAIL,M,1\n",
        encoding="utf-8",
    )
    (feed / "trips.txt").write_text(
        "route_id,service_id,trip_id\nBUS,WKD,bus-trip\nRAIL,WKD,rail-trip\n",
        encoding="utf-8",
    )
    (feed / "stop_times.txt").write_text(
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n"
        "bus-trip,06:00:00,06:00:00,A,1\n"
        "bus-trip,,,B,2\n"
        "bus-trip,06:10:00,06:10:00,C,3\n"
        "rail-trip,06:20:00,06:20:00,A,1\n"
        "rail-trip,06:25:00,06:25:00,C,2\n",
        encoding="utf-8",
    )
    (feed / "calendar.txt").write_text(
        "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date\n"
        "WKD,1,1,1,1,1,0,0,20260101,20261231\n",
        encoding="utf-8",
    )
    return feed


def test_aviation_supply_snapshot_has_expected_identifiers_and_counts():
    air = ROOT / "data" / "multimode" / "supply" / "air"
    with (air / "airports_k21.csv").open(newline="", encoding="utf-8") as handle:
        airports = list(csv.DictReader(handle))
    with (air / "aircraft_k21.csv").open(newline="", encoding="utf-8") as handle:
        aircraft = list(csv.DictReader(handle))

    assert len(airports) == 21
    assert len({row["airport_code"] for row in airports}) == 21
    assert {"WVI", "CCR"}.issubset(row["airport_code"] for row in airports)
    assert all(float(row["nearest_node_distance_m"]) >= 0 for row in airports)
    assert len(aircraft) == 9
    assert len({row["aircraft_code"] for row in aircraft}) == 9
    assert all(int(row["seats"]) > 0 and float(row["range_km"]) > 0 for row in aircraft)


def test_regional_gtfs_catalog_covers_nine_counties_with_unique_namespaces():
    catalog = json.loads(
        (ROOT / "lpsim_multimode" / "sources.json").read_text(encoding="utf-8")
    )
    group = catalog["source_groups"]["bay_area_fixed_route_gtfs"]
    sources = [catalog["sources"][name] for name in group["sources"]]

    assert len(sources) == 23
    assert len(group["expected_regions"]) == 9
    assert len({source["namespace"] for source in sources}) == 23
    assert {mode for source in sources for mode in source["modes"]} >= {
        "bus",
        "metro",
    }


def test_gtfs_extracts_bus_and_rail_connections_and_extended_time(tmp_path):
    feed = GtfsFeed(_gtfs(tmp_path), dt.date(2026, 8, 24))

    assert len(feed.connections(Mode.BUS)) == 2
    assert len(feed.connections(Mode.METRO)) == 1
    assert feed.connections(Mode.BUS)[0].arrival_time == 6 * 3600 + 5 * 60
    assert parse_gtfs_time("25:01:02") == 90062
    assert route_type_mode("3") == Mode.BUS
    assert route_type_mode("1") == Mode.METRO
    assert route_type_mode("4") is None
    assert feed.representative_service_date(dt.date(2026, 8, 31)) == dt.date(
        2026, 8, 31
    )


def test_blank_stop_time_interpolation():
    rows = [
        {
            "stop_id": "A",
            "stop_sequence": "1",
            "arrival_time": "06:00:00",
            "departure_time": "06:00:00",
        },
        {
            "stop_id": "B",
            "stop_sequence": "2",
            "arrival_time": "",
            "departure_time": "",
        },
        {
            "stop_id": "C",
            "stop_sequence": "3",
            "arrival_time": "06:10:00",
            "departure_time": "06:10:00",
        },
    ]

    assert resolve_stop_times(rows)[1][1:] == (
        6 * 3600 + 5 * 60,
        6 * 3600 + 5 * 60,
    )


def test_gtfs_export_preserves_routes_and_scheduled_times(tmp_path):
    feed = _gtfs(tmp_path)
    output = tmp_path / "export"

    summary = export_supply(
        feed,
        output,
        service_date=dt.date(2026, 8, 24),
        provider_id="test",
    )
    with (output / "transit_connections.csv").open(
        newline="", encoding="utf-8"
    ) as handle:
        connections = list(csv.DictReader(handle))

    assert summary["routes"] == 2
    assert summary["connections_by_mode"] == {"bus": 2, "metro": 1}
    assert connections[0]["provider_id"] == "test"
    assert connections[0]["service_date"] == "2026-08-24"
    assert connections[0]["departure_time_gtfs"] == "06:00:00"
    assert connections[0]["arrival_time_gtfs"] == "06:05:00"


def test_bundle_builder_namespaces_providers_and_records_dates(tmp_path, monkeypatch):
    raw = tmp_path / "raw"
    first_parent = raw / "first"
    second_parent = raw / "second"
    first_parent.mkdir(parents=True)
    second_parent.mkdir(parents=True)
    _gtfs(first_parent)
    _gtfs(second_parent)
    catalog = tmp_path / "sources.json"
    catalog.write_text(
        json.dumps(
            {
                "sources": {
                    "first": {
                        "target": "first/gtfs",
                        "namespace": "first",
                        "provider": "First Transit",
                        "regions": ["North"],
                        "modes": ["bus", "metro"],
                    },
                    "second": {
                        "target": "second/gtfs",
                        "namespace": "second",
                        "provider": "Second Transit",
                        "regions": ["South"],
                        "modes": ["bus", "metro"],
                    },
                },
                "source_groups": {
                    "test": {
                        "description": "test bundle",
                        "expected_regions": ["North", "South"],
                        "sources": ["first", "second"],
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(build_gtfs_bundle, "MANIFEST", catalog)
    output = tmp_path / "bundle"

    result = build_gtfs_bundle.build_bundle(
        raw_root=raw,
        output=output,
        group_name="test",
        scenario_date=dt.date(2026, 8, 24),
    )
    bundled = GtfsFeed(output)

    assert result["covered_regions"] == ["North", "South"]
    assert result["active_routes_by_mode"] == {"bus": 2, "metro": 2}
    assert "first:A" in bundled.stops
    assert "second:A" in bundled.stops
    assert len(bundled.connections(Mode.BUS)) == 4
    assert len(bundled.connections(Mode.METRO)) == 2


def test_mode_choice_examples_satisfy_normalized_constraints():
    examples = ROOT / "data" / "multimode" / "examples"
    summary, errors = validate_files(
        examples / "od_trips.csv",
        examples / "od_mode_options.csv",
        examples / "od_mode_legs.csv",
    )

    assert errors == []
    assert summary == {"trips": 3, "options": 6, "legs": 12, "errors": 0}
