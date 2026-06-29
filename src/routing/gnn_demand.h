#pragma once
/**
 * GNN-Based Demand Prediction Interface
 *
 * The node+edge graph structure is natural input for Graph Neural Networks.
 * This module defines the interface for feeding current network state into
 * a GNN model that predicts short-term demand (next 15 min) from current
 * edge congestion, enabling proactive fleet positioning.
 *
 * Architecture:
 *   Current state (edge speeds, densities) → GNN encoder → demand forecast
 *   Forecast → LLM/RL dispatcher → proactive vehicle positioning
 *
 * Integration with PyTorch Geometric:
 *   - Export edge_index + node_features as tensors
 *   - Run inference via Python subprocess or TorchScript
 *   - Import predicted demand back into simulator
 */

#include <vector>
#include <cstdint>

namespace lpsim {

struct GraphFeatures {
    int num_nodes;
    int num_edges;
    std::vector<std::pair<int,int>> edge_index;  // COO format
    std::vector<float> node_features;            // [num_nodes × feature_dim]
    std::vector<float> edge_features;            // [num_edges × edge_feature_dim]
};

struct DemandPrediction {
    int horizon_minutes;
    std::vector<float> predicted_demand;  // per-node demand rate
    float confidence;
};

// Feature extraction from current simulation state
inline int getNodeFeatureDim() { return 4; }  // [degree, avg_speed, density, demand_rate]
inline int getEdgeFeatureDim() { return 3; }  // [length, speed_limit, current_flow]

} // namespace lpsim
