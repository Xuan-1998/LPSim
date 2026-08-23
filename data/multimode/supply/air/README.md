# K21 aviation supply snapshot

This directory contains the two small, normalized aviation-supply tables used
by the Project K21 Bay Area scenario:

- `airports_k21.csv`: 21 airport records with decimal coordinates, an
  approximate nearest K21 road node, runway count, and a scenario operations
  interval.
- `aircraft_k21.csv`: 9 aircraft types with passenger seats and range in miles
  and kilometers.

The source files came from the Project K21 Drive folder recorded in
`lpsim_multimode/sources.json`. Coordinate and nearest-node processing is
described below so these values are not mistaken for observed operations.

Airport DMS coordinates were converted to decimal degrees using west and south
as negative values. `nearest_node_index` is the geometric nearest node in the
K21 road network, calculated with haversine distance. It is not a validated
terminal entrance or airport access link. `operation_interval_s` and
`capacity_ops_per_hour` are scenario inputs, not measured airport throughput.

The aircraft table does not provide cruise speed, fleet count, home airport,
turnaround time, flight schedule, operating cost, or fare. Those fields must be
provided by a scenario before an OD option containing `uam` can be scored or
simulated.

The raw K21 road and demand files are not committed. They can be fetched using
the cataloged source IDs. Review the source terms before redistributing the
snapshot outside this project.
