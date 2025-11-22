#include "procsim.hpp"
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <cstring>

// Global processor state
uint64_t g_r;    // Number of result buses
uint64_t g_k0;   // Number of k0 FUs
uint64_t g_k1;   // Number of k1 FUs
uint64_t g_k2;   // Number of k2 FUs
uint64_t g_f;    // Fetch rate
uint64_t g_rs_size; // Reservation station size

// Register scoreboard - tracks which instruction will write to each register
int64_t register_ready[NUM_REGS]; // -1 means ready, otherwise tag of instruction that will write

// Function unit availability
std::vector<bool> fu_k0_available;
std::vector<bool> fu_k1_available;
std::vector<bool> fu_k2_available;

// Pipeline queues
std::vector<proc_inst_t> fetch_buffer;    // Pipeline register between fetch and dispatch
std::deque<proc_inst_t> dispatch_queue;  // Dispatch queue (unlimited)
std::vector<proc_inst_t> schedule_queue; // Reservation station (limited size)

// Global counters
uint64_t next_tag = 1;
uint64_t current_cycle = 0;
bool done_fetching = false;

// Statistics tracking
uint64_t total_fired = 0;
uint64_t total_retired = 0;
uint64_t total_dispatch_size = 0;
uint64_t max_dispatch_size = 0;

/**
 * Subroutine for initializing the processor.
 */
void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f)
{
    g_r = r;
    g_k0 = k0;
    g_k1 = k1;
    g_k2 = k2;
    g_f = f;
    g_rs_size = 2 * (k0 + k1 + k2);

    // Initialize register scoreboard - all registers are initially ready
    for (int i = 0; i < NUM_REGS; i++) {
        register_ready[i] = -1;
    }

    // Initialize function units
    fu_k0_available.resize(k0, true);
    fu_k1_available.resize(k1, true);
    fu_k2_available.resize(k2, true);

    next_tag = 1;
    current_cycle = 0;
    done_fetching = false;
    total_fired = 0;
    total_retired = 0;
    total_dispatch_size = 0;
    max_dispatch_size = 0;
}

/**
 * Subroutine that simulates the processor.
 */
void run_proc(proc_stats_t* p_stats)
{
    bool all_done = false;

    while (!all_done) {
        current_cycle++;

        // Track statistics
        total_dispatch_size += dispatch_queue.size();
        if (dispatch_queue.size() > max_dispatch_size) {
            max_dispatch_size = dispatch_queue.size();
        }

        // ==================================================================
        // FIRST HALF CYCLE
        // ==================================================================

        // 1. State Update: Free FUs, update register ready bits, remove from RS
        std::vector<proc_inst_t*> completed_insts;
        for (auto& inst : schedule_queue) {
            if (inst.execution_complete && inst.state_update_cycle == 0) {
                completed_insts.push_back(&inst);
            }
        }

        // Sort by complete_cycle first (oldest), then by tag
        std::sort(completed_insts.begin(), completed_insts.end(),
            [](const proc_inst_t* a, const proc_inst_t* b) {
                if (a->complete_cycle != b->complete_cycle) {
                    return a->complete_cycle < b->complete_cycle;
                }
                return a->tag < b->tag;
            });

        // State update up to R instructions per cycle
        std::vector<uint64_t> tags_to_remove;
        for (size_t i = 0; i < completed_insts.size() && i < g_r; i++) {
            proc_inst_t* inst = completed_insts[i];

            // Free FU
            int fu_type = inst->fu_type;
            if (fu_type == 0) {
                for (size_t j = 0; j < fu_k0_available.size(); j++) {
                    // Find which FU this instruction is using (we'll just free the first non-available one)
                    if (!fu_k0_available[j]) {
                        fu_k0_available[j] = true;
                        break;
                    }
                }
            } else if (fu_type == 1) {
                for (size_t j = 0; j < fu_k1_available.size(); j++) {
                    if (!fu_k1_available[j]) {
                        fu_k1_available[j] = true;
                        break;
                    }
                }
            } else if (fu_type == 2) {
                for (size_t j = 0; j < fu_k2_available.size(); j++) {
                    if (!fu_k2_available[j]) {
                        fu_k2_available[j] = true;
                        break;
                    }
                }
            }

            // Mark register as ready
            if (inst->dest_reg != -1) {
                if (register_ready[inst->dest_reg] == (int64_t)inst->tag) {
                    register_ready[inst->dest_reg] = -1;
                }
            }

            inst->state_update_cycle = current_cycle;
            tags_to_remove.push_back(inst->tag);
            total_retired++;

            printf("%lu\tSTATE UPDATE\t%lu\n", current_cycle, inst->tag);
            fflush(stdout);
        }

        // NOTE: Do NOT remove from RS here - do it in second half after schedule stage

        // 2. Check for completed executions (latency = 1, so instructions that fired last cycle complete this cycle)
        for (auto& inst : schedule_queue) {
            if (inst.fired && !inst.execution_complete && inst.execute_cycle < current_cycle) {
                inst.complete_cycle = current_cycle;
                inst.execution_complete = true;
                printf("%lu\tEXECUTED\t%lu\n", current_cycle, inst.tag);
                fflush(stdout);
            }
        }

        // 3. Update ready bits for all instructions in RS
        // A source becomes ready when producer completes STATE UPDATE (writes via result bus)
        for (auto& inst : schedule_queue) {
            if (!inst.fired) {
                for (int i = 0; i < 2; i++) {
                    // Only update if not already ready
                    if (!inst.src_ready[i] && inst.src_producer[i] != -1) {
                        // Check if producer completed state update
                        bool producer_completed = false;
                        bool producer_found_in_rs = false;

                        // Check if producer is in RS and has completed state update
                        for (const auto& rs_inst : schedule_queue) {
                            if (rs_inst.tag == (uint64_t)inst.src_producer[i]) {
                                producer_found_in_rs = true;
                                // Ready if state update completed
                                if (rs_inst.state_update_cycle > 0) {
                                    producer_completed = true;
                                }
                                break;
                            }
                        }

                        // If not in RS, check if in DQ or already completed
                        if (!producer_found_in_rs) {
                            bool producer_in_dq = false;
                            for (const auto& dq_inst : dispatch_queue) {
                                if (dq_inst.tag == (uint64_t)inst.src_producer[i]) {
                                    producer_in_dq = true;
                                    break;
                                }
                            }

                            // If not in RS and not in DQ, it already completed
                            if (!producer_in_dq) {
                                producer_completed = true;
                            }
                        }

                        if (producer_completed) {
                            inst.src_ready[i] = true;
                        }
                    }
                }
            }
        }

        // 4. Schedule: Move READY instructions from dispatch queue to RS
        // Per spec: "The dispatch unit reserves slots in the scheduling queues" (step 4, first half)
        // Scan dispatch queue from head to tail, schedule ready instructions (out-of-order allowed)
        auto it = dispatch_queue.begin();
        while (it != dispatch_queue.end() && schedule_queue.size() < g_rs_size) {
            proc_inst_t inst = *it;
            inst.schedule_cycle = current_cycle;

            // Check if sources are ready (same timing as ready bit update above)
            bool all_ready = true;
            for (int i = 0; i < 2; i++) {
                if (inst.src_producer[i] == -1) {
                    // No producer, source is ready
                    continue;
                } else {
                    // Check if producer has completed state update
                    bool producer_completed = false;
                    bool producer_found_in_rs = false;

                    // Check if producer is in RS and has completed state update
                    for (const auto& rs_inst : schedule_queue) {
                        if (rs_inst.tag == (uint64_t)inst.src_producer[i]) {
                            producer_found_in_rs = true;
                            // Ready if state update completed (result written via result bus)
                            if (rs_inst.state_update_cycle > 0) {
                                producer_completed = true;
                            }
                            break;
                        }
                    }

                    // If not in RS, check if in DQ or already completed
                    if (!producer_found_in_rs) {
                        bool producer_in_dq = false;
                        for (const auto& dq_inst : dispatch_queue) {
                            if (dq_inst.tag == (uint64_t)inst.src_producer[i]) {
                                producer_in_dq = true;
                                break;
                            }
                        }

                        // If not in RS and not in DQ, it completed in a previous cycle
                        if (!producer_in_dq) {
                            producer_completed = true;
                        }
                    }

                    if (!producer_completed) {
                        all_ready = false;
                        break;
                    }
                }
            }

            if (all_ready) {
                // Schedule this instruction (sources are ready)
                inst.src_ready[0] = true;
                inst.src_ready[1] = true;

                schedule_queue.push_back(inst);
                printf("%lu\tSCHEDULED\t%lu\n", current_cycle, inst.tag);
                fflush(stdout);

                it = dispatch_queue.erase(it);  // Remove and advance iterator
            } else {
                // Not ready - skip to next in DQ
                ++it;
            }
        }

        // 5. Fire ready instructions to function units (in tag order)
        // First half: happens after scheduling so newly scheduled instructions can fire same cycle
        std::vector<proc_inst_t*> ready_to_fire;
        for (auto& inst : schedule_queue) {
            if (!inst.fired && inst.src_ready[0] && inst.src_ready[1]) {
                ready_to_fire.push_back(&inst);
            }
        }

        std::sort(ready_to_fire.begin(), ready_to_fire.end(),
            [](const proc_inst_t* a, const proc_inst_t* b) { return a->tag < b->tag; });

        for (auto* inst : ready_to_fire) {
            bool fired = false;
            int fu_type = inst->fu_type;

            if (fu_type == 0) {
                for (size_t i = 0; i < fu_k0_available.size(); i++) {
                    if (fu_k0_available[i]) {
                        fu_k0_available[i] = false;
                        fired = true;
                        break;
                    }
                }
            } else if (fu_type == 1) {
                for (size_t i = 0; i < fu_k1_available.size(); i++) {
                    if (fu_k1_available[i]) {
                        fu_k1_available[i] = false;
                        fired = true;
                        break;
                    }
                }
            } else if (fu_type == 2) {
                for (size_t i = 0; i < fu_k2_available.size(); i++) {
                    if (fu_k2_available[i]) {
                        fu_k2_available[i] = false;
                        fired = true;
                        break;
                    }
                }
            }

            if (fired) {
                inst->fired = true;
                inst->execute_cycle = current_cycle;
                total_fired++;

                // With latency=1, instruction will complete next cycle
            }
        }

        // ==================================================================
        // SECOND HALF CYCLE
        // ==================================================================

        // 6. Dispatch: Move instructions from fetch buffer to dispatch queue
        for (auto& inst : fetch_buffer) {
            inst.dispatch_cycle = current_cycle;

            // Save producer tags at dispatch time
            // At this point, register_ready shows the most recent previous writer
            for (int i = 0; i < 2; i++) {
                if (inst.src_reg[i] == -1) {
                    // No source register
                    inst.src_producer[i] = -1;
                } else if (inst.src_reg[i] == inst.dest_reg) {
                    // Self-dependency: instruction reads and writes same register
                    // This is always ready (no actual dependency)
                    inst.src_producer[i] = -1;
                } else {
                    // Save which instruction will produce this value
                    inst.src_producer[i] = register_ready[inst.src_reg[i]];
                }
            }

            // Mark destination register as not ready (update scoreboard)
            if (inst.dest_reg != -1) {
                register_ready[inst.dest_reg] = inst.tag;
            }

            dispatch_queue.push_back(inst);
            printf("%lu\tDISPATCHED\t%lu\n", current_cycle, inst.tag);
            fflush(stdout);
        }
        fetch_buffer.clear();

        // 7. Remove state-updated instructions from RS (second half cycle)
        for (uint64_t tag : tags_to_remove) {
            schedule_queue.erase(
                std::remove_if(schedule_queue.begin(), schedule_queue.end(),
                    [tag](const proc_inst_t& inst) { return inst.tag == tag; }),
                schedule_queue.end()
            );
        }

        // 8. Fetch: Read instructions from stdin into fetch buffer
        if (!done_fetching) {
            for (uint64_t i = 0; i < g_f; i++) {
                proc_inst_t inst;
                if (read_instruction(&inst)) {
                    inst.tag = next_tag++;
                    inst.fetch_cycle = current_cycle;
                    inst.fired = false;
                    inst.execution_complete = false;
                    inst.state_update_cycle = 0;

                    // Handle function unit type -1 -> use type 1
                    if (inst.op_code == -1) {
                        inst.fu_type = 1;
                    } else {
                        inst.fu_type = inst.op_code;
                    }

                    fetch_buffer.push_back(inst);
                    printf("%lu\tFETCHED\t%lu\n", current_cycle, inst.tag);
                    fflush(stdout);
                } else {
                    done_fetching = true;
                    break;
                }
            }
        }

        // Check if done
        all_done = done_fetching &&
                   fetch_buffer.empty() &&
                   dispatch_queue.empty() &&
                   schedule_queue.empty();

        // Progress indicator
        if (current_cycle % 10000 == 0) {
            fprintf(stderr, "Cycle %lu: RS=%lu/%lu, DQ=%lu\n",
                    current_cycle, schedule_queue.size(), g_rs_size,
                    dispatch_queue.size());
        }
    }

    p_stats->cycle_count = current_cycle;
}

/**
 * Subroutine for cleaning up and calculating statistics
 */
void complete_proc(proc_stats_t *p_stats)
{
    p_stats->retired_instruction = total_retired;
    // Use the cycle_count that was set in run_proc for consistency
    p_stats->avg_inst_fired = (float)total_fired / (float)p_stats->cycle_count;
    p_stats->avg_inst_retired = (float)total_retired / (float)p_stats->cycle_count;
    p_stats->avg_disp_size = (float)total_dispatch_size / (float)p_stats->cycle_count;
    p_stats->max_disp_size = max_dispatch_size;
}
