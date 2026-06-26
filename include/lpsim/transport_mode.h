#pragma once
/**
 * Multi-Modal Transport Definitions
 *
 * Extends the simulator beyond cars + UAM to support:
 * - Public transit (bus, metro) with fixed routes and schedules
 * - Cycling with dedicated lanes and speed model
 * - Walking for last-mile connections
 * - Intermodal transfers (park-and-ride, bike-to-transit)
 *
 * Each mode shares the same node+edge graph but applies mode-specific
 * edge weights, capacity constraints, and movement models.
 */

#include <cstdint>

namespace lpsim {

enum class TransportMode : uint8_t {
    CAR = 0,           // Standard private vehicle (IDM car-following)
    AV = 1,            // Autonomous vehicle (dispatched by RL/LLM)
    BUS = 2,           // Fixed-route public transit
    METRO = 3,         // Rail (separate infrastructure, shared stations)
    BIKE = 4,          // Cycling (dedicated lanes where available)
    WALK = 5,          // Pedestrian (for multimodal transfers)
    UAM = 6,           // Urban air mobility (vertiport-to-vertiport)
    RIDESHARE = 7,     // Shared ride (2-4 passengers, pooled routing)
};

struct ModeCharacteristics {
    float max_speed_mps;        // mode-specific speed limit
    float acceleration;         // m/s²
    float deceleration;         // m/s²
    float min_headway_sec;      // minimum following distance in time
    float pcu_equivalent;       // passenger car unit (bus=2.5, bike=0.2)
    bool uses_dedicated_lane;   // true for bus lanes, bike lanes
    bool is_scheduled;          // true for bus/metro (follows timetable)
    uint8_t max_passengers;     // capacity (bus=60, car=4, bike=1)
};

constexpr ModeCharacteristics MODE_DEFAULTS[] = {
    // CAR:  max_spd  accel  decel  headway  PCU   dedicated  scheduled  capacity
    {33.3f,  2.5f,   4.0f,  1.5f,  1.0f,   false, false,     4},
    // AV:
    {33.3f,  3.0f,   5.0f,  1.0f,  1.0f,   false, false,     4},
    // BUS:
    {16.7f,  1.2f,   2.5f,  3.0f,  2.5f,   true,  true,      60},
    // METRO:
    {27.8f,  1.0f,   1.5f,  90.0f, 0.0f,   true,  true,      200},
    // BIKE:
    {8.3f,   1.5f,   3.0f,  1.0f,  0.2f,   true,  false,     1},
    // WALK:
    {1.4f,   0.5f,   1.0f,  0.5f,  0.1f,   true,  false,     1},
    // UAM:
    {83.3f,  5.0f,   5.0f,  30.0f, 0.0f,   true,  false,     4},
    // RIDESHARE:
    {33.3f,  2.5f,   4.0f,  1.5f,  1.0f,   false, false,     4},
};

struct TransitRoute {
    uint32_t route_id;
    TransportMode mode;         // BUS or METRO
    uint32_t* stop_nodes;       // ordered list of stop node IDs
    uint16_t num_stops;
    float headway_sec;          // frequency (e.g., every 300s = 5 min)
    float first_departure_sec;  // first service of the day
    float last_departure_sec;   // last service
};

// Intermodal transfer point (park-and-ride, bike dock, transit station)
struct TransferPoint {
    uint32_t node_id;
    TransportMode from_mode;
    TransportMode to_mode;
    float transfer_time_sec;    // time to switch modes
    uint16_t capacity;          // parking spots / bike docks
    uint16_t current_occupancy;
};

} // namespace lpsim
