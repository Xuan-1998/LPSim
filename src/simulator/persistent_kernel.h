#pragma once
/**
 * Persistent Kernel Design for LPSim
 *
 * Instead of launching kernels every 0.5s timestep (14,400 launches for
 * a 2-hour simulation), a persistent kernel stays resident on the GPU and
 * loops internally. Host communicates via device-side flags.
 *
 * Benefits:
 *   - Eliminates all kernel launch overhead (~50us × 14,400 = 720ms saved)
 *   - Enables tighter pipelining of compute + communication
 *   - Reduces CPU-GPU synchronization points
 *
 * Architecture:
 *   Host:                           Device (persistent):
 *   ┌─────────────────┐            ┌────────────────────────┐
 *   │ write config     │───────────>│ poll config flag       │
 *   │ signal "go"      │            │ if (go):               │
 *   │ wait "done"      │<───────────│   run_timestep()       │
 *   │ read results     │            │   signal "done"        │
 *   └─────────────────┘            │   loop                 │
 *                                   └────────────────────────┘
 *
 * Constraints:
 *   - Grid size must be fixed (use maximum expected vehicle count)
 *   - Threads that have no work early-exit each iteration
 *   - Requires cooperative groups for grid-wide sync between phases
 */

#include <cstdint>

namespace lpsim {

// Device-side control flags (in pinned/managed memory)
struct PersistentKernelControl {
    volatile int go_flag;           // host sets to 1 to start next timestep
    volatile int done_flag;         // device sets to 1 when timestep complete
    volatile int terminate_flag;    // host sets to 1 to exit the kernel
    float current_time;             // updated by host each timestep
    int num_active_vehicles;        // updated by host if vehicles added/removed
    int padding[3];                 // cache line alignment
};

// Configuration for the persistent kernel
struct PersistentConfig {
    int max_vehicles;               // grid sized for this many threads
    int threads_per_block;          // typically 256 or 384
    int num_timesteps;              // total iterations before exit
    float delta_time;               // simulation timestep (0.5s default)
    bool enable_cooperative_groups; // required for grid-wide __syncthreads
};

/**
 * To use persistent kernel mode:
 *
 * 1. Allocate PersistentKernelControl in managed memory
 * 2. Launch kernel ONCE with cooperative launch API:
 *      cudaLaunchCooperativeKernel(persistent_traffic_sim, ...)
 * 3. For each timestep:
 *      control->current_time = t;
 *      control->go_flag = 1;
 *      while (!control->done_flag) {}  // spin-wait (or use event)
 *      control->done_flag = 0;
 * 4. To terminate:
 *      control->terminate_flag = 1;
 *      control->go_flag = 1;
 *      cudaDeviceSynchronize();
 *
 * Note: requires sm_70+ for cooperative groups.
 * Note: grid size limited by SM count × max_blocks_per_SM.
 */

} // namespace lpsim
