"""Acceptance tests for the OpenStreetMap network exporter proposed in #106.

These tests intentionally exercise only an offline GraphML input. Live
Overpass requests do not belong in CI: they are slow, rate-limited, and
non-deterministic. The same exporter entry point can add place/bbox download
support while keeping normalization and CSV serialization testable here.
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

import networkx as nx
import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GENERATOR = REPOSITORY_ROOT / "tools" / "generate_network.py"
DEMAND_GENERATOR = REPOSITORY_ROOT / "tools" / "generate_demand.py"
PARTITIONER = REPOSITORY_ROOT / "tools" / "partition_network.py"

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

LARGE_OSM_ID = 5_464_992_110  # larger than uint32, like current OSM data


def _run(*args: object) -> subprocess.CompletedProcess[str]:
    command = [sys.executable, *(str(arg) for arg in args)]
    return subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )


def _assert_success(result: subprocess.CompletedProcess[str]) -> None:
    assert result.returncode == 0, (
        f"command failed with exit code {result.returncode}\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )


def _write_graphml_fixture(path: Path) -> None:
    graph = nx.MultiDiGraph()
    graph.graph["crs"] = "epsg:4326"

    # The three-node component is strongly connected and should be retained.
    graph.add_node(LARGE_OSM_ID, x=-122.2700, y=37.8700, ref="", highway="")
    graph.add_node(2_002, x=-122.2680, y=37.8710, ref="", highway="traffic_signals")
    graph.add_node(3_003, x=-122.2660, y=37.8720, ref="7", highway="motorway_junction")

    # Parallel edges expose an important LPSim limitation: its C++ graph is
    # keyed by (u, v), so the exporter must select one edge deterministically.
    # The longer edge is faster in free-flow time and is therefore expected to
    # win: 150 m / 50 mph < 100 m / 30 mph.
    graph.add_edge(
        LARGE_OSM_ID,
        2_002,
        key=0,
        length=100.0,
        lanes="2",
        maxspeed="30 mph",
        highway="primary",
        oneway=True,
    )
    graph.add_edge(
        LARGE_OSM_ID,
        2_002,
        key=1,
        length=150.0,
        lanes="1",
        maxspeed="50 mph",
        highway="primary",
        oneway=True,
    )

    # OSM maxspeed without an explicit unit is kph. Missing lanes and speed
    # exercise the CLI defaults without depending on regional OSM coverage.
    graph.add_edge(
        2_002,
        3_003,
        length=80.0,
        maxspeed="80",
        highway="secondary",
        oneway=True,
    )
    graph.add_edge(
        3_003,
        LARGE_OSM_ID,
        length=90.0,
        lanes="2;3",
        highway="tertiary",
        oneway=True,
    )
    graph.add_edge(
        2_002,
        LARGE_OSM_ID,
        length=105.0,
        lanes="1",
        maxspeed="30 mph",
        highway="primary",
        oneway=True,
    )
    graph.add_edge(
        3_003,
        2_002,
        length=85.0,
        lanes="1",
        maxspeed="40 mph",
        highway="secondary",
        oneway=True,
    )
    graph.add_edge(
        LARGE_OSM_ID,
        3_003,
        length=95.0,
        lanes="1",
        maxspeed="25 mph",
        highway="tertiary",
        oneway=True,
    )

    # A smaller disconnected component must not leak into generated OD data.
    graph.add_node(9_001, x=-121.9000, y=37.5000)
    graph.add_node(9_002, x=-121.8990, y=37.5010)
    graph.add_edge(
        9_001, 9_002, length=25.0, lanes="1", maxspeed="20 mph", highway="residential"
    )
    graph.add_edge(
        9_002, 9_001, length=25.0, lanes="1", maxspeed="20 mph", highway="residential"
    )

    nx.write_graphml(graph, path)


def _generate_network(graphml: Path, output: Path) -> subprocess.CompletedProcess[str]:
    return _run(
        GENERATOR,
        "--graphml",
        graphml,
        "--output",
        output,
        "--connectivity",
        "strong",
        "--default-speed-kph",
        40,
        "--default-lanes",
        1,
    )


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


@pytest.fixture()
def generated_network(tmp_path: Path) -> Path:
    graphml = tmp_path / "source.graphml"
    output = tmp_path / "network"
    _write_graphml_fixture(graphml)
    result = _generate_network(graphml, output)
    _assert_success(result)
    return output


def test_cli_documents_download_and_offline_input_modes() -> None:
    result = _run(GENERATOR, "--help")
    _assert_success(result)

    assert "--place" in result.stdout
    assert "--bbox" in result.stdout
    assert "--graphml" in result.stdout
    assert "--output" in result.stdout


def test_exports_exact_current_schema_with_dense_ids(generated_network: Path) -> None:
    node_columns, nodes = _read_csv(generated_network / "nodes.csv")
    edge_columns, edges = _read_csv(generated_network / "edges.csv")

    assert node_columns == NODE_COLUMNS
    assert edge_columns == EDGE_COLUMNS

    node_indices = [int(row["index"]) for row in nodes]
    edge_ids = [int(row["uniqueid"]) for row in edges]
    assert node_indices == list(range(len(nodes)))
    assert edge_ids == list(range(len(edges)))

    node_index_set = set(node_indices)
    assert all(int(row["u"]) in node_index_set for row in edges)
    assert all(int(row["v"]) in node_index_set for row in edges)
    assert all(int(row["u"]) != int(row["v"]) for row in edges)

    osmids = {int(row["osmid"]) for row in nodes}
    assert LARGE_OSM_ID in osmids
    assert {int(row["osmid_u"]) for row in edges} <= osmids
    assert {int(row["osmid_v"]) for row in edges} <= osmids


def test_keeps_largest_strong_component_and_removes_parallel_edges(
    generated_network: Path,
) -> None:
    _, nodes = _read_csv(generated_network / "nodes.csv")
    _, edges = _read_csv(generated_network / "edges.csv")

    assert {int(row["osmid"]) for row in nodes} == {LARGE_OSM_ID, 2_002, 3_003}

    endpoint_pairs = [(int(row["u"]), int(row["v"])) for row in edges]
    assert len(endpoint_pairs) == len(set(endpoint_pairs))

    exported = nx.DiGraph()
    exported.add_nodes_from(int(row["index"]) for row in nodes)
    exported.add_edges_from(endpoint_pairs)
    assert nx.is_strongly_connected(exported)

    selected = next(
        row
        for row in edges
        if int(row["osmid_u"]) == LARGE_OSM_ID and int(row["osmid_v"]) == 2_002
    )
    assert float(selected["length"]) == pytest.approx(150.0)
    assert float(selected["speed_mph"]) == pytest.approx(50.0, abs=0.05)


def test_normalizes_speed_units_lanes_and_numeric_constraints(
    generated_network: Path,
) -> None:
    _, edges = _read_csv(generated_network / "edges.csv")

    metric_speed_edge = next(
        row
        for row in edges
        if int(row["osmid_u"]) == 2_002 and int(row["osmid_v"]) == 3_003
    )
    assert float(metric_speed_edge["speed_mph"]) == pytest.approx(
        80 / 1.609344,
        abs=0.05,
    )
    assert int(metric_speed_edge["lanes"]) == 1

    default_speed_edge = next(
        row
        for row in edges
        if int(row["osmid_u"]) == 3_003 and int(row["osmid_v"]) == LARGE_OSM_ID
    )
    assert float(default_speed_edge["speed_mph"]) == pytest.approx(
        40 / 1.609344,
        abs=0.05,
    )
    # Semicolon-separated OSM lane tags are ambiguous; either adjacent value
    # is acceptable, but silently falling back to the CLI default is not.
    assert int(default_speed_edge["lanes"]) in {2, 3}

    for row in edges:
        assert math.isfinite(float(row["length"])) and float(row["length"]) > 0
        assert int(row["lanes"]) >= 1
        assert math.isfinite(float(row["speed_mph"])) and float(row["speed_mph"]) > 0


def test_export_is_byte_for_byte_deterministic(tmp_path: Path) -> None:
    graphml = tmp_path / "source.graphml"
    first = tmp_path / "first"
    second = tmp_path / "second"
    _write_graphml_fixture(graphml)

    _assert_success(_generate_network(graphml, first))
    _assert_success(_generate_network(graphml, second))

    assert (first / "nodes.csv").read_bytes() == (second / "nodes.csv").read_bytes()
    assert (first / "edges.csv").read_bytes() == (second / "edges.csv").read_bytes()


def test_generated_network_works_with_existing_demand_and_partition_tools(
    generated_network: Path,
) -> None:
    demand_result = _run(
        DEMAND_GENERATOR,
        "--network",
        generated_network,
        "--num-trips",
        25,
        "--start-hour",
        5,
        "--end-hour",
        6,
        "--model",
        "uniform",
        "--seed",
        7,
    )
    _assert_success(demand_result)

    demand_columns, trips = _read_csv(generated_network / "od_demand.csv")
    assert demand_columns == ["dep_time", "origin", "destination"]
    assert len(trips) == 25
    assert all(5 * 3600 <= float(row["dep_time"]) < 6 * 3600 for row in trips)
    assert all(row["origin"] != row["destination"] for row in trips)

    partition_result = _run(
        PARTITIONER,
        "--network",
        generated_network,
        "--num-parts",
        2,
        "--method",
        "spatial",
    )
    _assert_success(partition_result)

    partitions = (generated_network / "partitions.txt").read_text().splitlines()
    assert len(partitions) == 3
    assert set(partitions) <= {"0", "1"}
