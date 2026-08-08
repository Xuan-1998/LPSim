#!/usr/bin/env python3
# ruff: noqa: UP045 -- pyproject supports Python 3.8, before PEP 604 types.
"""Generate a current-schema LPSim road network from OpenStreetMap data.

The exporter accepts a live OSM place/bounding-box query or an offline GraphML
file. It normalizes OSM attributes, reduces the MultiDiGraph to the single-edge
``(u, v)`` representation used by LPSim, assigns dense integer identifiers, and
writes ``nodes.csv`` and ``edges.csv``.
"""

from __future__ import annotations

import argparse
import ast
import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

KPH_PER_MPH = 1.609344
MPS_PER_MPH = 0.44704
NUMBER_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)")

NODE_COLUMNS = ["osmid", "x", "y", "ref", "highway", "index"]
EDGE_COLUMNS = [
    "uniqueid",
    "osmid_u",
    "osmid_v",
    "length",
    "lanes",
    "speed_mph",
    "u",
    "v",
]


@dataclass(frozen=True)
class NormalizedEdge:
    """One edge after OSM attributes have been normalized."""

    osmid_u: int
    osmid_v: int
    length: float
    lanes: int
    speed_mph: float
    source_key: str

    @property
    def free_flow_time_seconds(self) -> float:
        return self.length / (self.speed_mph * MPS_PER_MPH)

    @property
    def selection_rank(self) -> tuple[float, float, float, str]:
        """Deterministic policy for reducing parallel OSM edges."""

        return (
            self.free_flow_time_seconds,
            self.length,
            -self.speed_mph,
            self.source_key,
        )


def _require_networkx() -> Any:
    try:
        import networkx as nx
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "Network generation dependencies are missing. "
            "Install them with: pip install -e '.[network]'"
        ) from error
    return nx


def _require_osmnx() -> Any:
    try:
        import osmnx as ox
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "Live OpenStreetMap downloads require OSMnx. "
            "Install it with: pip install -e '.[network]'"
        ) from error
    return ox


def _flatten_attribute(value: Any) -> list[Any]:
    """Turn OSM scalar/list/list-like-string attributes into scalar tokens."""

    if value is None:
        return []
    if isinstance(value, float) and math.isnan(value):
        return []
    if isinstance(value, set):
        value = sorted(value, key=str)
    if isinstance(value, (list, tuple)):
        flattened: list[Any] = []
        for item in value:
            flattened.extend(_flatten_attribute(item))
        return flattened
    if not isinstance(value, str):
        return [value]

    text = value.strip()
    if not text:
        return []

    if text[:1] in "[(" and text[-1:] in ")]":
        try:
            parsed = ast.literal_eval(text)
        except (SyntaxError, ValueError):
            parsed = None
        if parsed is not None and parsed != value:
            return _flatten_attribute(parsed)

    return [part.strip() for part in re.split(r"[;,|]", text) if part.strip()]


def _first_positive_number(value: Any) -> Optional[float]:
    for token in _flatten_attribute(value):
        match = NUMBER_PATTERN.search(str(token))
        if match is None:
            continue
        number = float(match.group())
        if math.isfinite(number) and number > 0:
            return number
    return None


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "y", "-1"}


def _parse_lanes(value: Any, default_lanes: int, oneway: bool) -> int:
    explicit_value = _first_positive_number(value)
    if explicit_value is None:
        return default_lanes

    lane_count = max(1, round(explicit_value))
    if not oneway:
        # OSM's lanes tag normally describes both directions on two-way ways.
        lane_count = max(1, math.ceil(lane_count / 2))
    return lane_count


def _speed_tokens_to_mph(value: Any, default_unit: str) -> list[float]:
    speeds: list[float] = []
    for token in _flatten_attribute(value):
        text = str(token).strip().lower()
        match = NUMBER_PATTERN.search(text)
        if match is None:
            continue
        speed = float(match.group())
        if not math.isfinite(speed) or speed <= 0:
            continue

        is_mph = "mph" in text or default_unit == "mph"
        speeds.append(speed if is_mph else speed / KPH_PER_MPH)
    return speeds


def _parse_speed_mph(edge_data: dict[str, Any], default_speed_kph: float) -> float:
    candidates = (
        (edge_data.get("speed_mph"), "mph"),
        (edge_data.get("speed_kph"), "kph"),
        # Per OSM tagging conventions, numeric maxspeed values without a unit
        # are kilometres per hour. Explicit "mph" strings remain mph.
        (edge_data.get("maxspeed"), "kph"),
    )
    for value, default_unit in candidates:
        speeds = _speed_tokens_to_mph(value, default_unit)
        if speeds:
            return sum(speeds) / len(speeds)
    return default_speed_kph / KPH_PER_MPH


def _stringify_attribute(value: Any) -> str:
    values = _flatten_attribute(value)
    return ";".join(str(item) for item in values)


def _format_float(value: float, decimal_places: int) -> str:
    text = f"{value:.{decimal_places}f}".rstrip("0").rstrip(".")
    return text if text not in {"", "-0"} else "0"


def _load_graphml(path: Path) -> Any:
    if not path.is_file():
        raise ValueError(f"GraphML file does not exist: {path}")

    nx = _require_networkx()
    try:
        graph = nx.read_graphml(path, node_type=int, force_multigraph=True)
    except (OSError, ValueError, nx.NetworkXError) as error:
        raise ValueError(f"Unable to read GraphML file {path}: {error}") from error
    return nx.MultiDiGraph(graph)


def _download_graph(
    place: Optional[str],
    bbox: Optional[Sequence[float]],
    network_type: str,
) -> Any:
    ox = _require_osmnx()
    if place is not None:
        return ox.graph.graph_from_place(
            place,
            network_type=network_type,
            simplify=True,
            retain_all=True,
        )
    if bbox is not None:
        return ox.graph.graph_from_bbox(
            tuple(bbox),
            network_type=network_type,
            simplify=True,
            retain_all=True,
        )
    raise ValueError("one of --place, --bbox, or --graphml is required")


def _remove_unusable_edges(graph: Any) -> int:
    unusable: list[tuple[Any, Any, Any]] = []
    for source, target, key, data in graph.edges(keys=True, data=True):
        length = _first_positive_number(data.get("length"))
        if source == target or length is None:
            unusable.append((source, target, key))
            continue
        data["_lpsim_length"] = length

    graph.remove_edges_from(unusable)
    graph.remove_nodes_from(list(_isolated_nodes(graph)))
    return len(unusable)


def _isolated_nodes(graph: Any) -> Iterable[Any]:
    return (node for node, degree in graph.degree() if degree == 0)


def _select_component(graph: Any, connectivity: str) -> Any:
    nx = _require_networkx()
    if graph.number_of_nodes() == 0 or graph.number_of_edges() == 0:
        raise ValueError("the source graph contains no usable road edges")

    if connectivity == "all":
        return graph.copy()
    if connectivity == "strong":
        components = nx.strongly_connected_components(graph)
    elif connectivity == "weak":
        components = nx.weakly_connected_components(graph)
    else:
        raise ValueError(f"unsupported connectivity mode: {connectivity}")

    largest = max(
        components, key=lambda component: (len(component), -min(map(int, component)))
    )
    return graph.subgraph(largest).copy()


def _validate_node_coordinates(graph: Any) -> None:
    for osmid, data in graph.nodes(data=True):
        try:
            x = float(data["x"])
            y = float(data["y"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"node {osmid} is missing numeric x/y coordinates"
            ) from error
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"node {osmid} has non-finite x/y coordinates")


def _normalized_edges(
    graph: Any,
    default_speed_kph: float,
    default_lanes: int,
) -> list[NormalizedEdge]:
    selected: dict[tuple[int, int], NormalizedEdge] = {}

    for source, target, key, data in graph.edges(keys=True, data=True):
        edge = NormalizedEdge(
            osmid_u=int(source),
            osmid_v=int(target),
            length=float(data["_lpsim_length"]),
            lanes=_parse_lanes(
                data.get("lanes"),
                default_lanes,
                _as_bool(data.get("oneway", True)),
            ),
            speed_mph=_parse_speed_mph(data, default_speed_kph),
            source_key=str(key),
        )
        pair = (edge.osmid_u, edge.osmid_v)
        previous = selected.get(pair)
        if previous is None or edge.selection_rank < previous.selection_rank:
            selected[pair] = edge

    return list(selected.values())


def _write_network(
    graph: Any,
    edges: list[NormalizedEdge],
    output: Path,
) -> None:
    osmids = sorted((int(node) for node in graph.nodes()), key=int)
    node_indices = {osmid: index for index, osmid in enumerate(osmids)}
    node_data = {int(node): data for node, data in graph.nodes(data=True)}

    output.mkdir(parents=True, exist_ok=True)
    nodes_path = output / "nodes.csv"
    edges_path = output / "edges.csv"

    with nodes_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=NODE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for osmid in osmids:
            data = node_data[osmid]
            writer.writerow(
                {
                    "osmid": osmid,
                    "x": _format_float(float(data["x"]), 10),
                    "y": _format_float(float(data["y"]), 10),
                    "ref": _stringify_attribute(data.get("ref")),
                    "highway": _stringify_attribute(data.get("highway")),
                    "index": node_indices[osmid],
                }
            )

    ordered_edges = sorted(
        edges,
        key=lambda edge: (
            node_indices[edge.osmid_u],
            node_indices[edge.osmid_v],
            edge.selection_rank,
        ),
    )
    with edges_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=EDGE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for unique_id, edge in enumerate(ordered_edges):
            writer.writerow(
                {
                    "uniqueid": unique_id,
                    "osmid_u": edge.osmid_u,
                    "osmid_v": edge.osmid_v,
                    "length": _format_float(edge.length, 3),
                    "lanes": edge.lanes,
                    "speed_mph": _format_float(edge.speed_mph, 6),
                    "u": node_indices[edge.osmid_u],
                    "v": node_indices[edge.osmid_v],
                }
            )


def generate_network(
    graph: Any,
    output: Path,
    connectivity: str = "strong",
    default_speed_kph: float = 40.0,
    default_lanes: int = 1,
) -> tuple[int, int]:
    """Normalize a graph and write LPSim CSV files.

    Returns the number of exported nodes and directed edges.
    """

    if not math.isfinite(default_speed_kph) or default_speed_kph <= 0:
        raise ValueError("--default-speed-kph must be a positive finite number")
    if default_lanes < 1:
        raise ValueError("--default-lanes must be at least 1")

    graph = graph.copy()
    dropped_edges = _remove_unusable_edges(graph)
    graph = _select_component(graph, connectivity)
    _validate_node_coordinates(graph)
    edges = _normalized_edges(graph, default_speed_kph, default_lanes)
    if not edges:
        raise ValueError("the selected component contains no exportable road edges")

    _write_network(graph, edges, output)
    print(
        f"Generated LPSim network: {graph.number_of_nodes():,} nodes, "
        f"{len(edges):,} directed edges -> {output}"
    )
    if dropped_edges:
        print(f"Dropped {dropped_edges:,} self-loop or invalid-length edges")
    return graph.number_of_nodes(), len(edges)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate current-schema LPSim network CSVs from OpenStreetMap"
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--place", help="OSM place query, e.g. 'Berkeley, California, USA'"
    )
    source.add_argument(
        "--bbox",
        nargs=4,
        type=float,
        metavar=("LEFT", "BOTTOM", "RIGHT", "TOP"),
        help="OSM bounding box in longitude/latitude order",
    )
    source.add_argument("--graphml", type=Path, help="Offline GraphML input file")

    parser.add_argument(
        "--output", type=Path, required=True, help="Output network directory"
    )
    parser.add_argument(
        "--network-type",
        default="drive",
        help="OSMnx network type for live downloads (default: drive)",
    )
    parser.add_argument(
        "--connectivity",
        choices=["strong", "weak", "all"],
        default="strong",
        help="Component selection before export (default: strong)",
    )
    parser.add_argument(
        "--default-speed-kph",
        type=float,
        default=40.0,
        help="Fallback speed for edges without maxspeed (default: 40)",
    )
    parser.add_argument(
        "--default-lanes",
        type=int,
        default=1,
        help="Fallback per-direction lane count (default: 1)",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.graphml is not None:
            graph = _load_graphml(args.graphml)
        else:
            graph = _download_graph(args.place, args.bbox, args.network_type)
        generate_network(
            graph,
            output=args.output,
            connectivity=args.connectivity,
            default_speed_kph=args.default_speed_kph,
            default_lanes=args.default_lanes,
        )
    except (RuntimeError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    sys.exit(main())
