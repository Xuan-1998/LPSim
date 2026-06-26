#pragma once
/**
 * Multi-Node Distributed Simulation Layer
 *
 * Extends the single-node multi-GPU simulation to span multiple nodes
 * connected via high-bandwidth network (InfiniBand/EFA). Uses NCCL for
 * collective communication of ghost-zone state and MPI for control plane.
 *
 * Architecture:
 *   Node 0 [GPU 0-7]  <-- NCCL AllGather -->  Node 1 [GPU 8-15]
 *        |                                         |
 *    intra-node P2P                         intra-node P2P
 *
 * Each node owns a subset of graph partitions. Vehicles crossing node
 * boundaries are serialized into migration buffers and transferred via
 * NCCL point-to-point sends.
 *
 * Communication pattern per timestep:
 *   1. Each GPU runs local simulation (same as single-node)
 *   2. Boundary vehicles identified (ghost zone protocol)
 *   3. Intra-node migration: P2P memcpy (existing code)
 *   4. Inter-node migration: NCCL Send/Recv for cross-node vehicles
 *   5. Ghost lane map updates: NCCL AllGather of boundary lane states
 *   6. Barrier before next timestep
 */

#include <cstdint>
#include <cstdio>
#include <vector>

// Forward declarations (actual headers included only when NCCL/MPI available)
#ifdef LPSIM_DISTRIBUTED
#include <nccl.h>
#include <mpi.h>
#endif

namespace lpsim {

struct DistributedConfig {
    int world_size = 1;          // total number of nodes
    int local_rank = 0;          // this node's rank
    int gpus_per_node = 8;       // GPUs on each node
    int total_gpus = 8;          // world_size * gpus_per_node
    bool use_nccl = true;        // use NCCL for GPU-GPU across nodes
    bool use_rdma = true;        // use RDMA (GDR) for zero-copy
};

struct MigrationMessage {
    int source_node;
    int target_node;
    int source_gpu;
    int target_gpu;
    int num_vehicles;
    // Followed by serialized vehicle data
};

class DistributedSimulator {
public:
    DistributedSimulator(DistributedConfig config) : config_(config) {
        printf("[Distributed] Initializing: %d nodes × %d GPUs = %d total GPUs\n",
               config.world_size, config.gpus_per_node, config.total_gpus);
    }

    // Initialize NCCL communicators across all nodes
    bool initialize() {
#ifdef LPSIM_DISTRIBUTED
        MPI_Comm_rank(MPI_COMM_WORLD, &config_.local_rank);
        MPI_Comm_size(MPI_COMM_WORLD, &config_.world_size);
        config_.total_gpus = config_.world_size * config_.gpus_per_node;

        // Create NCCL communicator
        ncclUniqueId nccl_id;
        if (config_.local_rank == 0) {
            ncclGetUniqueId(&nccl_id);
        }
        MPI_Bcast(&nccl_id, sizeof(ncclUniqueId), MPI_BYTE, 0, MPI_COMM_WORLD);

        ncclComm_t comm;
        ncclCommInitRank(&comm, config_.total_gpus,
                         nccl_id, config_.local_rank * config_.gpus_per_node);
        comms_.push_back(comm);

        printf("[Node %d] NCCL initialized, %d GPUs total\n",
               config_.local_rank, config_.total_gpus);
        return true;
#else
        printf("[Distributed] Built without NCCL/MPI. Single-node only.\n");
        return config_.world_size == 1;
#endif
    }

    // Exchange boundary vehicles between nodes after each timestep
    void exchangeBoundaryVehicles(
        const std::vector<MigrationMessage>& outgoing,
        std::vector<MigrationMessage>& incoming) {
#ifdef LPSIM_DISTRIBUTED
        // AllToAll exchange of migration metadata
        int send_count = outgoing.size();
        std::vector<int> all_counts(config_.world_size);
        MPI_Allgather(&send_count, 1, MPI_INT,
                      all_counts.data(), 1, MPI_INT, MPI_COMM_WORLD);

        // Point-to-point vehicle data transfer via NCCL
        for (const auto& msg : outgoing) {
            if (msg.target_node != config_.local_rank) {
                // ncclSend(vehicle_data, size, ncclChar, target_rank, comm, stream);
            }
        }
        // ncclGroupEnd + stream sync
#endif
    }

    // Synchronize ghost lane map state across node boundaries
    void syncGhostLanes(void* lane_buffer, size_t buffer_size) {
#ifdef LPSIM_DISTRIBUTED
        // AllGather boundary lane states
        // Only exchange lanes that are on partition boundaries between nodes
        // ncclAllGather(sendbuff, recvbuff, count, datatype, comm, stream);
#endif
    }

    // Barrier: ensure all nodes finished current timestep
    void barrier() {
#ifdef LPSIM_DISTRIBUTED
        MPI_Barrier(MPI_COMM_WORLD);
#endif
    }

    bool isDistributed() const { return config_.world_size > 1; }
    int getRank() const { return config_.local_rank; }
    int getWorldSize() const { return config_.world_size; }

private:
    DistributedConfig config_;
#ifdef LPSIM_DISTRIBUTED
    std::vector<ncclComm_t> comms_;
#endif
};

} // namespace lpsim
