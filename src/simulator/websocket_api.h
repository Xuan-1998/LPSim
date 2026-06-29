#pragma once
/**
 * Real-time WebSocket API for Live Simulation State Streaming
 *
 * Enables the web visualizer to connect to a running simulation and
 * receive vehicle positions + edge states in real-time. Supports
 * bidirectional communication for interactive control.
 *
 * Protocol:
 *   Client → Server: {"action": "set_toll", "edges": [...], "values": [...]}
 *   Server → Client: {"time": 18500, "vehicles": [[lon,lat,speed], ...], "congestion": {...}}
 *
 * Implementation options:
 *   - Standalone C++ WebSocket server (libwebsockets / uWebSockets)
 *   - Python bridge: C++ writes to shared memory, Python serves WebSocket
 *   - File-based: C++ writes JSON per frame, Python inotify + WebSocket
 */

namespace lpsim {

struct WebSocketConfig {
    int port = 8765;
    int broadcast_interval_ms = 100;  // send state every 100ms
    int max_clients = 16;
    bool compress = true;             // per-message deflate
};

struct SimulationFrame {
    float sim_time_sec;
    int num_active_vehicles;
    int num_completed_trips;
    float avg_speed_mps;
    float congestion_ratio;
    // Vehicle positions sent as packed binary: [lon, lat, speed] × N
};

} // namespace lpsim
