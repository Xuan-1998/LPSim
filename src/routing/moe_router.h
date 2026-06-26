#pragma once
/**
 * Mixture-of-Experts (MoE) Router for Zone-Adaptive Dispatch Policies
 *
 * Applies different routing/dispatch strategies based on geographic zone
 * and traffic regime. Inspired by MoE architectures in LLMs where different
 * expert networks handle different token types.
 *
 * Architecture:
 *   Gate(traffic_state) → selects expert policy per zone
 *   Expert_downtown:  congestion-aware, minimize queue spillback
 *   Expert_highway:   throughput-maximizing, speed priority
 *   Expert_suburban:  energy-efficient, minimize VMT
 *   Expert_airport:   time-critical, shortest-path priority
 *
 * Integration with LPSim:
 *   1. Partition network into zones (spatial or functional)
 *   2. Each zone has an assigned expert policy
 *   3. Gate function re-evaluates zone assignment periodically
 *   4. Vehicles crossing zone boundaries transition between experts
 */

#include <vector>
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace lpsim {

enum class RoutingExpert {
    DOWNTOWN_CONGESTION,  // Avoids congested links, uses dynamic tolls
    HIGHWAY_THROUGHPUT,   // Maximizes flow, prefers high-capacity roads
    SUBURBAN_EFFICIENT,   // Minimizes total VMT, eco-routing
    AIRPORT_TIMECRIT,     // Strict shortest-time, ignores cost
    UAM_AERIAL,           // Air corridor routing for vertiport-to-vertiport
};

struct ZoneState {
    int zone_id;
    float avg_speed_mps;
    float congestion_ratio;  // vehicles / capacity
    int active_vehicles;
    float demand_rate;       // requests per minute
};

struct ExpertConfig {
    RoutingExpert type;
    float speed_weight;      // how much to weight speed vs distance
    float congestion_penalty; // multiplier on congested edges
    float toll_sensitivity;  // how much tolls affect route choice
};

class MoERouter {
public:
    MoERouter(int num_zones) : num_zones_(num_zones) {
        zone_experts_.resize(num_zones, RoutingExpert::HIGHWAY_THROUGHPUT);
        zone_states_.resize(num_zones);
        initDefaultExperts();
    }

    // Gate function: assign expert to each zone based on current state
    void updateGating(const std::vector<ZoneState>& zone_states) {
        zone_states_ = zone_states;
        for (int z = 0; z < num_zones_; z++) {
            zone_experts_[z] = selectExpert(zone_states[z]);
        }
    }

    // Get the routing expert for a given zone
    RoutingExpert getExpert(int zone_id) const {
        if (zone_id >= 0 && zone_id < num_zones_) {
            return zone_experts_[zone_id];
        }
        return RoutingExpert::HIGHWAY_THROUGHPUT;
    }

    // Get edge weight multiplier based on the expert policy
    float getEdgeWeight(int zone_id, float base_time, float congestion, float toll) const {
        const ExpertConfig& cfg = expert_configs_.at(getExpert(zone_id));
        float weight = base_time * cfg.speed_weight;
        weight += congestion * cfg.congestion_penalty;
        weight += toll * cfg.toll_sensitivity;
        return weight;
    }

    // Summary for LLM context
    std::string getSummary() const {
        std::string s = "MoE Routing Policy:\n";
        for (int z = 0; z < num_zones_; z++) {
            s += "  Zone " + std::to_string(z) + ": " + expertName(zone_experts_[z]);
            s += " (congestion=" + std::to_string(zone_states_[z].congestion_ratio) + ")\n";
        }
        return s;
    }

private:
    int num_zones_;
    std::vector<RoutingExpert> zone_experts_;
    std::vector<ZoneState> zone_states_;
    std::unordered_map<RoutingExpert, ExpertConfig> expert_configs_;

    void initDefaultExperts() {
        expert_configs_[RoutingExpert::DOWNTOWN_CONGESTION] = {RoutingExpert::DOWNTOWN_CONGESTION, 0.8f, 3.0f, 0.5f};
        expert_configs_[RoutingExpert::HIGHWAY_THROUGHPUT] = {RoutingExpert::HIGHWAY_THROUGHPUT, 1.2f, 0.5f, 0.1f};
        expert_configs_[RoutingExpert::SUBURBAN_EFFICIENT] = {RoutingExpert::SUBURBAN_EFFICIENT, 0.6f, 1.0f, 0.3f};
        expert_configs_[RoutingExpert::AIRPORT_TIMECRIT] = {RoutingExpert::AIRPORT_TIMECRIT, 2.0f, 0.2f, 0.0f};
        expert_configs_[RoutingExpert::UAM_AERIAL] = {RoutingExpert::UAM_AERIAL, 1.5f, 0.0f, 0.0f};
    }

    RoutingExpert selectExpert(const ZoneState& state) const {
        if (state.congestion_ratio > 0.8f) return RoutingExpert::DOWNTOWN_CONGESTION;
        if (state.avg_speed_mps > 25.0f) return RoutingExpert::HIGHWAY_THROUGHPUT;
        if (state.demand_rate < 5.0f) return RoutingExpert::SUBURBAN_EFFICIENT;
        return RoutingExpert::HIGHWAY_THROUGHPUT;
    }

    static std::string expertName(RoutingExpert e) {
        switch (e) {
            case RoutingExpert::DOWNTOWN_CONGESTION: return "downtown-congestion-aware";
            case RoutingExpert::HIGHWAY_THROUGHPUT: return "highway-throughput";
            case RoutingExpert::SUBURBAN_EFFICIENT: return "suburban-eco";
            case RoutingExpert::AIRPORT_TIMECRIT: return "airport-time-critical";
            case RoutingExpert::UAM_AERIAL: return "uam-aerial";
        }
        return "unknown";
    }
};

} // namespace lpsim
