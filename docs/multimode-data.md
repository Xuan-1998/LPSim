# Multimode supply data and OD option schema

This change is intentionally limited to the data-preparation layer. It records
the K21 aviation supply, reproducibly acquires Bay Area fixed-route schedules,
and defines the files needed to attach several candidate mode combinations to
one traveler OD. It does not add a mode-choice model or couple the mode engines.

## Included files

```text
data/multimode/
  supply/air/
    airports_k21.csv
    aircraft_k21.csv
  manifests/
    bay_area_gtfs_2026-08-24.json
    mode_supply_requirements.json
  schemas/
    od_mode_choice.schema.json
  examples/
    od_trips.csv
    od_mode_options.csv
    od_mode_legs.csv
    mode_option_templates.csv
lpsim_multimode/
  sources.json
  gtfs.py
tools/
  fetch_multimode_data.py
  build_gtfs_bundle.py
  export_gtfs_supply.py
  validate_mode_choice_inputs.py
```

Downloaded road networks, OD demand, and GTFS archives live under
`data/multimode/raw/` and are ignored by Git. This keeps large or frequently
changing agency files out of the repository while retaining their URLs,
provider namespaces, regions, license pages, service dates, and a reproducible
builder. In the supply manifests, `road_network` means the matching
`nodes.csv` and `edges.csv` pair.

## Aviation supply

`airports_k21.csv` contains 21 airport records. The important identifiers and
links for an itinerary are:

- `airport_code`: stable airport endpoint used by a UAM leg;
- `lon_decimal`, `lat_decimal`: airport location;
- `nearest_node_index`: approximate access node in the K21 road graph;
- `runway_count`, `operation_interval_s`, `capacity_ops_per_hour`: scenario
  operation inputs;
- `max_aircraft_type`: aircraft eligibility text from the source processing.

`aircraft_k21.csv` contains 9 aircraft types with `aircraft_code`, seats, and
range. It does not contain cruise speed, flight schedule, fleet count, initial
airport, turnaround time, operating cost, or fare. A UAM alternative is not
fully specified until those scenario inputs are added.

The airport-to-road mapping is geometric. It has not been validated as a road
entrance or terminal access point. See the supply-directory README for the
processing assumptions.

## Bay Area bus and rail schedules

The `bay_area_fixed_route_gtfs` source group covers all nine Bay Area counties
with 23 providers: SFMTA, AC Transit, BART, Caltrain, VTA, SamTrans, County
Connection, Golden Gate Transit, LAVTA, Marin Transit, SMART, Sonoma County
Transit, Santa Rosa CityBus, Petaluma Transit, Tri Delta Transit, WestCAT,
Union City Transit, FAST, SolTrans, Vacaville City Coach, VINE Transit, Rio
Vista Delta Breeze, and Emery Go-Round.

Fetch the source feeds and build one collision-free schedule bundle:

```bash
lpsim-fetch-data --output data/multimode/raw \
  --group bay_area_fixed_route_gtfs

lpsim-build-gtfs-bundle \
  --raw-root data/multimode/raw \
  --output data/multimode/raw/gtfs/bay_area_bundle \
  --scenario-date 2026-08-24
```

The builder assigns an operator namespace to every GTFS stop, route, trip, and
service ID. For example, `M30-2` becomes `bart:M30-2`. This prevents collisions
between agencies that both use common identifiers such as `1`.

Export the namespaced lines and scheduled stop-to-stop times into plain CSVs:

```bash
lpsim-export-gtfs-supply \
  --gtfs data/multimode/raw/gtfs/bay_area_bundle \
  --output data/multimode/raw/gtfs/bay_area_export
```

This creates `transit_routes.csv`, `transit_stops.csv`, and
`transit_connections.csv`. Each scheduled connection includes the provider,
mode, route, service, actual component service date, trip, stop pair, sequence,
and departure/arrival time in both seconds and GTFS `HH:MM:SS` form.

For each provider, the builder selects the latest active day with the same
weekday at or before the requested scenario date. It records the actual
`service_date` and `service_age_days`; it does not claim that an older schedule
is current. The committed audit snapshot records the local verification:

- 23 providers covering all 9 counties;
- 20,137 stops/platform records;
- 26,879 active trips;
- 529 active bus routes and 30 active rail routes;
- 871,601 bus connections and 70,498 rail connections.

Rio Vista Delta Breeze was the only public unauthenticated snapshot more than
one year old in that audit. A current 511 Regional GTFS archive can replace
individual feeds when `LPSIM_511_API_TOKEN` is available.

GTFS provides lines and scheduled times through `routes.txt`, `trips.txt`,
`calendar*.txt`, `stops.txt`, and `stop_times.txt`. It does not provide traveler
OD demand, reliable per-vehicle capacity, observed delay, road-congestion
feedback, or a complete inter-agency transfer model.

## Per-OD mode combinations

The schema uses three normalized CSV tables instead of one overloaded demand
file:

1. `od_trips.csv` contains each traveler request exactly once. K21 origin and
   destination values use `road_node` endpoint types.
2. `od_mode_options.csv` contains zero or more alternatives per OD, such as
   `car`, `walk>bus>walk`, or `rideshare>uam>rideshare`. Estimated attributes,
   probability, and chosen status remain empty until they are calculated.
3. `od_mode_legs.csv` expands every alternative into connected legs and stores
   supply references. Bus/rail legs can reference provider, route, and service
   trip IDs; UAM legs can reference airports and an aircraft code.

This separation preserves the traveler-demand total: adding three alternatives
to one OD does not create three travelers. Only one option may eventually be
marked `chosen=1`.

Validate the examples or a generated dataset with:

```bash
lpsim-validate-mode-choice \
  --trips data/multimode/examples/od_trips.csv \
  --options data/multimode/examples/od_mode_options.csv \
  --legs data/multimode/examples/od_mode_legs.csv
```

The validator checks foreign keys, endpoint continuity, contiguous leg order,
mode sequences, choice probabilities, and the one-choice-per-OD constraint.
The machine-readable field definitions are in
`data/multimode/schemas/od_mode_choice.schema.json`.

## Still required before mode choice

The committed `mode_supply_requirements.json` separates available data from
scenario inputs that are still absent. The main remaining inputs are:

- mode-specific travel-time and access/egress skims for every OD;
- fares, parking cost, transfer penalties, and reliability values;
- full Bay Area walking and bicycle networks;
- AV and rideshare fleet size, locations, capacity, and dispatch policy;
- transit capacity, transfer links, dwell time, and observed delay;
- UAM speed, fleet distribution, turnaround time, schedule, fare, and weather;
- observed mode shares or a calibrated utility model.

Until those inputs exist, `probability` and `chosen` should be left unassigned
rather than filled with arbitrary fixed mode shares.

## Source and redistribution notes

Acquisition metadata and provider license pages are in
`lpsim_multimode/sources.json`. Review each provider's current terms before
redistributing downloaded archives. The source catalog includes the project
Drive folder, SFMTA and AC Transit feeds, individual regional providers, and
the token-gated 511 Regional GTFS API.
