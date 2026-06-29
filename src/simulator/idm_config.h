#pragma once
/**
 * Configurable IDM (Intelligent Driver Model) Parameters
 *
 * Supports per-vehicle-class parameter sets for heterogeneous traffic:
 * aggressive drivers, cautious drivers, trucks, buses, AVs each have
 * distinct acceleration, braking, and headway characteristics.
 */

#include <cstdint>

namespace lpsim {

enum class VehicleClass : uint8_t {
    STANDARD = 0,
    AGGRESSIVE = 1,
    CAUTIOUS = 2,
    TRUCK = 3,
    BUS = 4,
    AUTONOMOUS = 5,
};

struct IDMParams {
    float a;        // max acceleration (m/s²)
    float b;        // comfortable deceleration (m/s²)
    float T;        // desired time headway (s)
    float s0;       // minimum gap (m)
    float v0_mult;  // desired speed as multiplier of speed limit
    float delta;    // acceleration exponent (typically 4)
};

constexpr IDMParams IDM_DEFAULTS[] = {
    // STANDARD
    {1.5f, 2.5f, 1.5f, 2.0f, 1.0f, 4.0f},
    // AGGRESSIVE
    {2.5f, 4.0f, 0.8f, 1.0f, 1.2f, 4.0f},
    // CAUTIOUS
    {1.0f, 2.0f, 2.5f, 3.0f, 0.85f, 4.0f},
    // TRUCK
    {0.8f, 1.5f, 2.0f, 4.0f, 0.75f, 4.0f},
    // BUS
    {1.0f, 2.0f, 2.5f, 3.5f, 0.7f, 4.0f},
    // AUTONOMOUS
    {2.0f, 5.0f, 0.6f, 0.5f, 1.0f, 4.0f},
};

} // namespace lpsim
