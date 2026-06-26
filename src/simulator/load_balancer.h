#pragma once
/**
 * Dynamic load balancer for multi-GPU traffic simulation.
 *
 * Monitors per-GPU vehicle count at runtime. When imbalance exceeds a
 * threshold (default 1.5x ratio between max and min), triggers a
 * lightweight repartition by migrating boundary nodes between partitions.
 *
 * This avoids the cost of a full METIS re-run while keeping GPUs balanced
 * as traffic patterns shift (e.g., morning rush toward downtown).
 */

#include <vector>
#include <cstdint>
#include <cstdio>

struct LoadBalancerConfig {
    float imbalance_threshold = 1.5f;   // max/min vehicle ratio to trigger
    int check_interval_steps = 600;     // check every 5 minutes of sim time (600 × 0.5s)
    int min_migrate_nodes = 10;         // minimum nodes to migrate per rebalance
    int max_migrate_nodes = 500;        // cap to avoid excessive communication
};

class MultiGPULoadBalancer {
public:
    MultiGPULoadBalancer(int num_gpus, LoadBalancerConfig config = {})
        : ngpus_(num_gpus), config_(config), step_counter_(0),
          rebalance_count_(0) {
        vehicle_counts_.resize(num_gpus, 0);
    }

    // Call every timestep with current vehicle counts per GPU
    bool shouldRebalance(const std::vector<int>& vehicle_counts) {
        step_counter_++;
        vehicle_counts_ = vehicle_counts;

        if (step_counter_ % config_.check_interval_steps != 0) {
            return false;
        }

        int max_count = *std::max_element(vehicle_counts.begin(), vehicle_counts.end());
        int min_count = *std::min_element(vehicle_counts.begin(), vehicle_counts.end());

        if (min_count == 0) return false;

        float ratio = (float)max_count / (float)min_count;
        if (ratio > config_.imbalance_threshold) {
            printf("[LoadBalancer] Imbalance detected: ratio=%.2f (max=%d, min=%d). "
                   "Triggering rebalance #%d\n", ratio, max_count, min_count, rebalance_count_ + 1);
            rebalance_count_++;
            return true;
        }
        return false;
    }

    // Compute how many boundary nodes to migrate from overloaded to underloaded GPU
    struct MigrationPlan {
        int source_gpu;
        int target_gpu;
        int num_nodes_to_migrate;
    };

    MigrationPlan computeMigrationPlan() {
        int max_gpu = 0, min_gpu = 0;
        int max_count = 0, min_count = INT32_MAX;

        for (int i = 0; i < ngpus_; i++) {
            if (vehicle_counts_[i] > max_count) {
                max_count = vehicle_counts_[i];
                max_gpu = i;
            }
            if (vehicle_counts_[i] < min_count) {
                min_count = vehicle_counts_[i];
                min_gpu = i;
            }
        }

        int excess = (max_count - min_count) / 2;
        int to_migrate = std::max(config_.min_migrate_nodes,
                         std::min(excess, config_.max_migrate_nodes));

        return {max_gpu, min_gpu, to_migrate};
    }

    int getRebalanceCount() const { return rebalance_count_; }

private:
    int ngpus_;
    LoadBalancerConfig config_;
    int step_counter_;
    int rebalance_count_;
    std::vector<int> vehicle_counts_;
};
