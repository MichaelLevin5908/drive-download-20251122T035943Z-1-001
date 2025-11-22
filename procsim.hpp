#ifndef PROCSIM_HPP
#define PROCSIM_HPP

#include <cstdint>
#include <cstdio>
#include <vector>
#include <queue>

#define DEFAULT_K0 1
#define DEFAULT_K1 2
#define DEFAULT_K2 3
#define DEFAULT_R 8
#define DEFAULT_F 4
#define NUM_REGS 128

typedef struct _proc_inst_t
{
    uint32_t instruction_address;
    int32_t op_code;
    int32_t src_reg[2];
    int32_t dest_reg;

    // Additional fields for simulation
    uint64_t tag;                // Instruction tag/sequence number
    uint64_t fetch_cycle;        // Cycle when instruction entered fetch
    uint64_t dispatch_cycle;     // Cycle when instruction entered dispatch
    uint64_t schedule_cycle;     // Cycle when instruction entered schedule
    uint64_t execute_cycle;      // Cycle when instruction entered execute
    uint64_t state_update_cycle; // Cycle when instruction entered state update
    bool src_ready[2];           // Ready bits for source registers
    bool fired;                  // Has this instruction been fired to FU?
    int32_t fu_type;             // Actual FU type to use (handling -1 case)
    uint64_t complete_cycle;     // Cycle when execution completes
    bool execution_complete;     // Has execution finished (waiting for result bus)?

} proc_inst_t;

typedef struct _proc_stats_t
{
    float avg_inst_retired;
    float avg_inst_fired;
    float avg_disp_size;
    unsigned long max_disp_size;
    unsigned long retired_instruction;
    unsigned long cycle_count;
} proc_stats_t;

bool read_instruction(proc_inst_t* p_inst);

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f);
void run_proc(proc_stats_t* p_stats);
void complete_proc(proc_stats_t* p_stats);

#endif /* PROCSIM_HPP */
