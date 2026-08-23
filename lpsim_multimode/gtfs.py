"""Dependency-free GTFS Schedule importer and connection builder."""

from __future__ import annotations

import csv
import datetime as dt
import io
import json
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

from .modes import Mode


@dataclass(frozen=True)
class Stop:
    stop_id: str
    name: str
    lat: float
    lon: float


@dataclass(frozen=True)
class Connection:
    trip_id: str
    route_id: str
    service_id: str
    from_stop: str
    to_stop: str
    departure_time: float
    arrival_time: float
    sequence: int
    mode: Mode


class GtfsFeed:
    """Static GTFS feed filtered to a service date when one is supplied."""

    def __init__(self, source: Path, service_date: dt.date | None = None):
        self.source = Path(source)
        self.service_date = service_date
        self.components: list[dict[str, object]] = []
        self._bundle_children: list[tuple[str, GtfsFeed]] | None = None
        if self.source.is_dir() and (self.source / "bundle.json").exists():
            self._load_bundle(service_date)
            return
        self.stops = self._load_stops()
        self.routes = {row["route_id"]: row for row in self._rows("routes.txt")}
        self._active_services = self._service_ids(service_date)
        self.trips = {
            row["trip_id"]: row
            for row in self._rows("trips.txt")
            if self._active_services is None
            or row["service_id"] in self._active_services
        }
        self._connections: list[Connection] | None = None

    @staticmethod
    def _qualified(namespace: str, identifier: str) -> str:
        return f"{namespace}:{identifier}"

    def _load_bundle(self, service_date: dt.date | None) -> None:
        manifest_path = self.source / "bundle.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("schema_version") != 1:
            raise ValueError(f"unsupported GTFS bundle schema in {manifest_path}")
        scenario_date = manifest.get("scenario_date")
        if service_date is None and scenario_date:
            self.service_date = dt.date.fromisoformat(str(scenario_date))

        self.stops: dict[str, Stop] = {}
        self.routes: dict[str, dict[str, str]] = {}
        self.trips: dict[str, dict[str, str]] = {}
        self._active_services: set[str] = set()
        self._bundle_children = []
        self._connections = None
        seen_namespaces: set[str] = set()
        for component in manifest.get("components", []):
            namespace = str(component["namespace"])
            if namespace in seen_namespaces:
                raise ValueError(f"duplicate GTFS bundle namespace: {namespace}")
            seen_namespaces.add(namespace)
            component_path = (self.source / str(component["path"])).resolve()
            component_date = dt.date.fromisoformat(str(component["service_date"]))
            child = GtfsFeed(component_path, component_date)
            self._bundle_children.append((namespace, child))

            for stop in child.stops.values():
                stop_id = self._qualified(namespace, stop.stop_id)
                self.stops[stop_id] = Stop(
                    stop_id=stop_id,
                    name=stop.name,
                    lat=stop.lat,
                    lon=stop.lon,
                )
            for route_id, row in child.routes.items():
                qualified = self._qualified(namespace, route_id)
                copied = dict(row)
                copied["route_id"] = qualified
                if copied.get("agency_id"):
                    copied["agency_id"] = self._qualified(
                        namespace, copied["agency_id"]
                    )
                self.routes[qualified] = copied
            for trip_id, row in child.trips.items():
                qualified = self._qualified(namespace, trip_id)
                copied = dict(row)
                copied["trip_id"] = qualified
                copied["route_id"] = self._qualified(namespace, row["route_id"])
                copied["service_id"] = self._qualified(namespace, row["service_id"])
                self.trips[qualified] = copied
                self._active_services.add(copied["service_id"])

            summary = dict(component)
            summary.update(
                {
                    "source": str(component_path),
                    "stops": len(child.stops),
                    "trips": len(child.trips),
                }
            )
            self.components.append(summary)

    def _text(self, name: str) -> str:
        if self.source.is_dir():
            path = self.source / name
            if not path.exists():
                raise FileNotFoundError(f"required GTFS table is missing: {path}")
            return path.read_text(encoding="utf-8-sig")
        with zipfile.ZipFile(self.source) as archive:
            try:
                return archive.read(name).decode("utf-8-sig")
            except KeyError as exc:
                raise FileNotFoundError(
                    f"required GTFS table is missing: {name}"
                ) from exc

    def _optional_text(self, name: str) -> str | None:
        try:
            return self._text(name)
        except FileNotFoundError:
            return None

    def _rows(self, name: str) -> Iterator[dict[str, str]]:
        yield from csv.DictReader(io.StringIO(self._text(name)))

    def _optional_rows(self, name: str) -> Iterable[dict[str, str]]:
        text = self._optional_text(name)
        return [] if text is None else csv.DictReader(io.StringIO(text))

    def _load_stops(self) -> dict[str, Stop]:
        result = {}
        for row in self._rows("stops.txt"):
            # Parent station rows sometimes omit coordinates and cannot be boarded.
            if not row.get("stop_lat") or not row.get("stop_lon"):
                continue
            stop = Stop(
                stop_id=row["stop_id"],
                name=row.get("stop_name", row["stop_id"]),
                lat=float(row["stop_lat"]),
                lon=float(row["stop_lon"]),
            )
            result[stop.stop_id] = stop
        return result

    def _service_ids(self, service_date: dt.date | None) -> set[str] | None:
        if service_date is None:
            return None
        active: set[str] = set()
        weekday = service_date.strftime("%A").lower()
        date_value = service_date.strftime("%Y%m%d")
        for row in self._optional_rows("calendar.txt"):
            if row.get(weekday) != "1":
                continue
            if (
                row.get("start_date", "00000000")
                <= date_value
                <= row.get("end_date", "99999999")
            ):
                active.add(row["service_id"])
        for row in self._optional_rows("calendar_dates.txt"):
            if row.get("date") != date_value:
                continue
            if row.get("exception_type") == "1":
                active.add(row["service_id"])
            elif row.get("exception_type") == "2":
                active.discard(row["service_id"])
        return active

    def connections(self, mode: Mode | None = None) -> list[Connection]:
        if self._connections is None:
            if self._bundle_children is not None:
                result = []
                for namespace, child in self._bundle_children:
                    for connection in child.connections():
                        result.append(
                            Connection(
                                trip_id=self._qualified(namespace, connection.trip_id),
                                route_id=self._qualified(
                                    namespace, connection.route_id
                                ),
                                service_id=self._qualified(
                                    namespace, connection.service_id
                                ),
                                from_stop=self._qualified(
                                    namespace, connection.from_stop
                                ),
                                to_stop=self._qualified(namespace, connection.to_stop),
                                departure_time=connection.departure_time,
                                arrival_time=connection.arrival_time,
                                sequence=connection.sequence,
                                mode=connection.mode,
                            )
                        )
                result.sort(
                    key=lambda connection: (
                        connection.departure_time,
                        connection.arrival_time,
                    )
                )
                self._connections = result
            else:
                self._connections = self._build_connections()
        if mode is None:
            return self._connections
        if mode not in (Mode.BUS, Mode.METRO):
            raise ValueError("GTFS connections are only valid for bus or metro mode")
        return [
            connection for connection in self._connections if connection.mode == mode
        ]

    def _build_connections(self) -> list[Connection]:
        stop_times: dict[str, list[dict[str, str]]] = {}
        for row in self._rows("stop_times.txt"):
            if row["trip_id"] in self.trips and row["stop_id"] in self.stops:
                stop_times.setdefault(row["trip_id"], []).append(row)

        result: list[Connection] = []
        for trip_id, rows in stop_times.items():
            trip = self.trips[trip_id]
            route = self.routes.get(trip["route_id"], {})
            trip_mode = route_type_mode(route.get("route_type", "3"))
            if trip_mode is None:
                continue
            rows.sort(key=lambda row: int(row["stop_sequence"]))
            resolved = resolve_stop_times(rows)
            for first, second in zip(resolved, resolved[1:]):
                first_row, _, departure = first
                second_row, arrival, _ = second
                if departure is None or arrival is None:
                    continue
                if arrival < departure:
                    continue
                result.append(
                    Connection(
                        trip_id=trip_id,
                        route_id=trip["route_id"],
                        service_id=trip["service_id"],
                        from_stop=first_row["stop_id"],
                        to_stop=second_row["stop_id"],
                        departure_time=departure,
                        arrival_time=arrival,
                        sequence=int(first_row["stop_sequence"]),
                        mode=trip_mode,
                    )
                )
        result.sort(
            key=lambda connection: (
                connection.departure_time,
                connection.arrival_time,
            )
        )
        return result

    def modes(self) -> set[Mode]:
        return {connection.mode for connection in self.connections()}

    def representative_service_date(
        self, target_date: dt.date, *, max_lookback_days: int = 3650
    ) -> dt.date | None:
        """Choose the latest active day with the target weekday at/before target."""

        if self._bundle_children is not None:
            return self.service_date
        calendar = list(self._optional_rows("calendar.txt"))
        exceptions = list(self._optional_rows("calendar_dates.txt"))
        bounds = [
            parse_gtfs_date(value)
            for row in calendar
            for value in (row.get("start_date"), row.get("end_date"))
            if value
        ] + [parse_gtfs_date(row["date"]) for row in exceptions if row.get("date")]
        if not bounds:
            return None
        lower = max(min(bounds), target_date - dt.timedelta(days=max_lookback_days))
        current = min(max(bounds), target_date)
        additions: dict[str, set[str]] = {}
        removals: dict[str, set[str]] = {}
        trip_service_ids = {row["service_id"] for row in self.trips.values()}
        for row in exceptions:
            bucket = additions if row.get("exception_type") == "1" else removals
            bucket.setdefault(row.get("date", ""), set()).add(row["service_id"])
        while current >= lower:
            if current.weekday() == target_date.weekday():
                date_value = current.strftime("%Y%m%d")
                weekday = current.strftime("%A").lower()
                active = {
                    row["service_id"]
                    for row in calendar
                    if row.get(weekday) == "1"
                    and row.get("start_date", "00000000")
                    <= date_value
                    <= row.get("end_date", "99999999")
                }
                active.update(additions.get(date_value, set()))
                active.difference_update(removals.get(date_value, set()))
                if active & trip_service_ids:
                    return current
            current -= dt.timedelta(days=1)
        return None


def parse_gtfs_time(value: str) -> float:
    """Convert GTFS HH:MM:SS, including times after 24:00, to seconds."""

    parts = value.strip().split(":")
    if len(parts) != 3:
        raise ValueError(f"invalid GTFS time {value!r}")
    hours, minutes, seconds = (int(part) for part in parts)
    if hours < 0 or not 0 <= minutes < 60 or not 0 <= seconds < 60:
        raise ValueError(f"invalid GTFS time {value!r}")
    return float(hours * 3600 + minutes * 60 + seconds)


def parse_gtfs_date(value: str) -> dt.date:
    """Convert a GTFS YYYYMMDD date without introducing timezone semantics."""

    if len(value) != 8 or not value.isdigit():
        raise ValueError(f"invalid GTFS date {value!r}")
    return dt.date(int(value[:4]), int(value[4:6]), int(value[6:8]))


def resolve_stop_times(
    rows: list[dict[str, str]],
) -> list[tuple[dict[str, str], float | None, float | None]]:
    """Resolve optional intermediate GTFS times by linear stop interpolation."""

    arrivals: list[float | None] = []
    departures: list[float | None] = []
    points: list[float | None] = []
    for row in rows:
        arrival_text = (row.get("arrival_time") or "").strip()
        departure_text = (row.get("departure_time") or "").strip()
        arrival = parse_gtfs_time(arrival_text) if arrival_text else None
        departure = parse_gtfs_time(departure_text) if departure_text else None
        if arrival is None and departure is not None:
            arrival = departure
        if departure is None and arrival is not None:
            departure = arrival
        arrivals.append(arrival)
        departures.append(departure)
        points.append(departure if departure is not None else arrival)

    previous: list[int | None] = []
    index: int | None = None
    for position, value in enumerate(points):
        if value is not None:
            index = position
        previous.append(index)
    following: list[int | None] = [None] * len(rows)
    index = None
    for position in range(len(points) - 1, -1, -1):
        if points[position] is not None:
            index = position
        following[position] = index

    for position, value in enumerate(points):
        if value is not None:
            continue
        before = previous[position]
        after = following[position]
        if before is None or after is None or before == after:
            continue
        start = points[before]
        end = points[after]
        if start is None or end is None or end < start:
            continue
        fraction = (position - before) / (after - before)
        interpolated = start + fraction * (end - start)
        arrivals[position] = interpolated
        departures[position] = interpolated
    return list(zip(rows, arrivals, departures))


def route_type_mode(value: str) -> Mode | None:
    """Map base and extended GTFS route types to LPSim bus/metro supply."""

    route_type = int(value or 3)
    if route_type in {3, 11} or 700 <= route_type < 900:
        return Mode.BUS
    # Tram, subway, rail, cable/funicular and their extended rail categories.
    if (
        route_type in {0, 1, 2, 5, 6, 7, 12}
        or 100 <= route_type < 700
        or 900 <= route_type < 1000
        or 1300 <= route_type < 1500
    ):
        return Mode.METRO
    # Ferry, water, air, and taxi route types are not silently relabeled.
    return None
