#!/usr/bin/env python3
"""
LPSim Web Visualizer: serves network + simulation state via HTTP.

Run: python viz/server.py --network data/networks/sf_bay_area --port 8080
Then open http://localhost:8080 in a browser.
"""

import argparse
import csv
import json
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path


def load_network_geojson(network_path):
    """Convert network CSV to GeoJSON for deck.gl rendering."""
    nodes_file = os.path.join(network_path, "nodes.csv")
    edges_file = os.path.join(network_path, "edges.csv")

    nodes = {}
    with open(nodes_file) as f:
        reader = csv.DictReader(f)
        has_index = "index" in reader.fieldnames
        for i, row in enumerate(reader):
            idx = int(row["index"]) if has_index else i
            nodes[idx] = {"lon": float(row["x"]), "lat": float(row["y"])}

    edges_features = []
    with open(edges_file) as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames
        has_uv = "u" in cols and "v" in cols
        for row in reader:
            if has_uv:
                u, v = int(row["u"]), int(row["v"])
            else:
                continue
            if u not in nodes or v not in nodes:
                continue
            src = nodes[u]
            dst = nodes[v]
            edges_features.append({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [src["lon"], src["lat"]],
                        [dst["lon"], dst["lat"]]
                    ]
                },
                "properties": {
                    "id": int(row.get("uniqueid", 0)),
                    "lanes": int(float(row.get("lanes", 1))),
                    "speed_mph": float(row.get("speed_mph", 30)),
                    "length": float(row.get("length", 0)),
                }
            })

    return {
        "type": "FeatureCollection",
        "features": edges_features
    }


class VizHandler(SimpleHTTPRequestHandler):
    """Serve static files from viz/ and API endpoints."""

    def __init__(self, *args, network_geojson=None, **kwargs):
        self.network_geojson = network_geojson
        super().__init__(*args, directory=str(Path(__file__).parent), **kwargs)

    def do_GET(self):
        if self.path == "/api/network":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(self.network_geojson).encode())
        elif self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ready"}).encode())
        else:
            super().do_GET()


def make_handler(network_geojson):
    def handler(*args, **kwargs):
        return VizHandler(*args, network_geojson=network_geojson, **kwargs)
    return handler


def main():
    parser = argparse.ArgumentParser(description="LPSim Web Visualizer")
    parser.add_argument("--network", default="data/networks/sf_bay_area")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    print(f"Loading network from {args.network}...")
    geojson = load_network_geojson(args.network)
    print(f"Loaded {len(geojson['features'])} edges")

    handler = make_handler(geojson)
    server = HTTPServer(("0.0.0.0", args.port), handler)
    print(f"Visualizer running at http://localhost:{args.port}")
    print("Press Ctrl+C to stop")
    server.serve_forever()


if __name__ == "__main__":
    main()
