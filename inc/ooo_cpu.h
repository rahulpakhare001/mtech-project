#ifndef OOO_CPU_H
#define OOO_CPU_H

#include "cache.h"
#include "page_table_walker.h"
#include <set>
#include <unordered_map>

#ifdef CRC2_COMPILE
#define STAT_PRINTING_PERIOD 1000000
#else
#define STAT_PRINTING_PERIOD 1000000
#endif
#define DEADLOCK_CYCLE 10000000

using namespace std;

// CORE PROCESSOR
#define FETCH_WIDTH 6
#define IFETCH_WIDTH 12
#define DECODE_WIDTH 6
#define EXEC_WIDTH 6
#define LQ_WIDTH 2
#define SQ_WIDTH 2
#define RETIRE_WIDTH 4
#define SCHEDULER_SIZE 128
#define BRANCH_MISPREDICT_PENALTY 1 //@Vishal: Updated from 20 to be same as new ChampSim, anyway penalty is simulated because of queues
//#define SCHEDULING_LATENCY 6
//#define EXEC_LATENCY 1

#define STA_SIZE (ROB_SIZE*NUM_INSTR_DESTINATIONS_SPARC)

#define BTB_SET 1024
#define BTB_WAY 4

#define PARTIAL_TAG 64

#ifdef ARKA_DP_PRED
extern uint64_t mispred_stlb_load, corrpred_stlb_load, mispred_stlb_bypass, corrpred_stlb_bypass;
extern uint64_t bypass_stlb;
#endif
extern uint32_t SCHEDULING_LATENCY, EXEC_LATENCY, DECODE_LATENCY;
extern uint8_t TRACE_ENDS_STOP;
//extern uint8_t threads[NUM_CPUS];
extern uint8_t highest_register_num;
extern uint64_t stlb_evict_crit_trans;
extern set<uint64_t> critical_ips[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> crit_ip_stlb_miss[NUM_CPUS][MAX_THREADS];
extern set<uint64_t> critical_translation[NUM_CPUS][MAX_THREADS];

extern set<uint64_t> all_pscl2_entry[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> per_crit_ip_stall[MAX_THREADS];
extern map<uint64_t, uint64_t> per_crit_trans_stall[MAX_THREADS];

struct miss_stats{
	uint16_t occurence;
	uint16_t miss;
};

extern map<uint64_t, struct miss_stats> freq_critical[NUM_CPUS][MAX_THREADS]; 

extern map<uint64_t, uint64_t> critical_trans_0_10[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> critical_trans_10_50[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> critical_trans_50_100[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> critical_trans_100_200[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> critical_trans_200_500[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, uint64_t> critical_trans_500_[NUM_CPUS][MAX_THREADS];

struct evicted_stats{
	uint64_t translation;
	uint64_t count;
};
#ifdef BYPASS_TLB
extern map<uint64_t, struct evicted_stats> bypass_stats[NUM_CPUS][MAX_THREADS];
extern uint64_t bypass_count;
#endif

extern set<uint64_t> total_translation[NUM_CPUS][MAX_THREADS];
extern set<uint64_t> total_ips[NUM_CPUS][MAX_THREADS];
extern uint64_t reused_translation;
extern uint64_t total_load_rob_stall_cycles[NUM_CPUS][MAX_THREADS];
extern uint64_t total_rob_stall_cycles[NUM_CPUS];
extern uint64_t rob_stall_count[NUM_CPUS];
extern uint64_t load_rob_stall_count[NUM_CPUS];

extern uint64_t load_rob_stall_prob_distri[NUM_CPUS][MAX_THREADS][6];

struct find_reuse{
	uint64_t last_cycle_accessed;
	uint64_t unique_accesses;
	uint8_t  criticalOrNot;
	uint64_t reuse_1_4, reuse_5_12, reuse_13_100, reuse_101_500, reuse_501_1000, reuse_1001;
};

extern uint64_t dead_reuse_1_4[2];
extern uint64_t dead_reuse_5_12[2];
extern uint64_t dead_reuse_13_50[2];
extern uint64_t dead_reuse_51_100[2];
extern uint64_t dead_reuse_101_500[2];
extern uint64_t dead_reuse_501_1000[2];
extern uint64_t dead_reuse_1001[2];

extern uint64_t nc[2][2], c[2][2];	
// [cip/ncip] [dtlb/stlb], [used/unused] 

extern uint64_t dead_pred_crit_dtlb[MAX_THREADS][3];	// how many translations brought by critical IPs were doa, dead, used in DTLB? 
extern uint64_t dead_pred_crit_stlb[MAX_THREADS][3];	// in STLB?

extern map<uint64_t, struct find_reuse> critical_reuse[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, struct find_reuse> non_critical_reuse[NUM_CPUS][MAX_THREADS];
extern map<uint64_t, struct find_reuse> both_reuse[NUM_CPUS][MAX_THREADS];

//extern unordered_map <uint64_t, set<uint64_t>> trans_per_cycle[DTLB_SET];
extern vector<uint64_t> trans_per_cycle[DTLB_SET];
extern map <uint64_t, uint64_t> evict_dead_trans[MAX_THREADS][2][DTLB_SET];


struct dead_stats{
	uint64_t doa;
	uint64_t dead;
	uint64_t used;
	uint16_t critical;
	uint64_t doa_crit;
	uint64_t doa_non_crit;
};
extern map <uint64_t, struct dead_stats> doa_dtlb[MAX_THREADS];
extern map <uint64_t, struct dead_stats> doa_stlb[MAX_THREADS];
extern map <uint64_t, struct dead_stats> doa_pscl2[MAX_THREADS];

extern uint64_t check_shadow_table(PACKET *packet);
extern uint64_t lookup_pHIST(PACKET *packet);
extern void modify_pHIST(BLOCK *block);

// cpu
class O3_CPU {
  public:
    uint32_t cpu;
    
    uint32_t thread;
    // trace
    FILE* trace_file[MAX_THREADS];	//@Vasudha:SMT: Multiple traces running on different threads in a single core
    char trace_string[MAX_THREADS][1024];
    char gunzip_command[MAX_THREADS][1024];
    int context_switch, operating_index, total_drop_from_PTW;
   
    // instruction
    input_instr next_instr;
    input_instr current_instr;
    cloudsuite_instr current_cloudsuite_instr;

    //@Vasudha:SMT: Keeping track of individual thread
    uint64_t instr_unique_id[MAX_THREADS];
    
    //@Vasudha:SMT: Keep a track of which thread was last to decode and execute
    int last_read_trace_thread, last_fetched_thread, last_decoded_thread, last_retired_thread, last_scheduled_thread;
    uint64_t completed_executions, begin_sim_cycle, begin_sim_instr[MAX_THREADS], 
             last_sim_cycle[MAX_THREADS], last_sim_instr[MAX_THREADS], finish_sim_cycle[MAX_THREADS], finish_sim_instr[MAX_THREADS],
             warmup_instructions, simulation_instructions, instrs_to_read_this_cycle, instrs_to_fetch_this_cycle,
             next_print_instruction[MAX_THREADS], num_retired[MAX_THREADS];
    uint32_t inflight_reg_executions, inflight_mem_executions, num_searched;
    uint32_t next_ITLB_fetch;

    // reorder buffer, load/store queue, register file
    CORE_BUFFER IFETCH_BUFFER[4]{CORE_BUFFER("IFETCH_BUFFER", FETCH_WIDTH*2), CORE_BUFFER("IFETCH_BUFFER", FETCH_WIDTH*2), CORE_BUFFER("IFETCH_BUFFER", FETCH_WIDTH*2), CORE_BUFFER("IFETCH_BUFFER", FETCH_WIDTH*2)};
    CORE_BUFFER DECODE_BUFFER{"DECODE_BUFFER", DECODE_WIDTH*3};
    PARTITIONED_CORE_BUFFER ROB{"ROB", ROB_SIZE};
    LOAD_STORE_QUEUE LQ{"LQ", LQ_SIZE}, SQ{"SQ", SQ_SIZE};
    
    // store array, this structure is required to properly handle store instructions
    uint64_t STA[STA_SIZE], STA_head[4], STA_tail[4], STA_partition[4]; 

    // Ready-To-Execute
    uint32_t RTE0[ROB_SIZE], RTE0_head, RTE0_tail, 
             RTE1[ROB_SIZE], RTE1_head, RTE1_tail;  

    // Ready-To-Load
    uint32_t RTL0[LQ_SIZE], RTL0_head, RTL0_tail, 
             RTL1[LQ_SIZE], RTL1_head, RTL1_tail;  

    // Ready-To-Store
    uint32_t RTS0[SQ_SIZE], RTS0_head, RTS0_tail,
             RTS1[SQ_SIZE], RTS1_head, RTS1_tail;

    // branch
    int branch_mispredict_stall_fetch; // flag that says that we should stall because a branch prediction was wrong
    int mispredicted_branch_iw_index; // index in the instruction window of the mispredicted branch.  fetch resumes after the instruction at this index executes
    bool read_instr[MAX_THREADS];		      //@Vasudha:SMT: Records if further instructions of the trace can be read 
    uint8_t  fetch_stall[MAX_THREADS];
    uint64_t fetch_resume_cycle[MAX_THREADS];
    uint64_t num_branch[MAX_THREADS], branch_mispredictions[MAX_THREADS];
    uint64_t total_rob_occupancy_at_branch_mispredict[MAX_THREADS];
    uint64_t total_branch_types[MAX_THREADS][8];

    //@Vishal
    uint64_t sim_RAW_hits[MAX_THREADS], roi_RAW_hits[MAX_THREADS], //Loads hitting in Store queue
	     sim_load_gen[MAX_THREADS], roi_load_gen[MAX_THREADS], //Loads generated by cpu
	     sim_load_sent[MAX_THREADS], roi_load_sent[MAX_THREADS], //Loads sent to L1D cache
    	     sim_store_gen[MAX_THREADS], roi_store_gen[MAX_THREADS], //Stores generated by cpu
             sim_store_sent[MAX_THREADS], roi_store_sent[MAX_THREADS]; //Stores sent to L1D cache

    // TLBs and caches
    CACHE ITLB{"ITLB", ITLB_SET, ITLB_WAY, ITLB_SET*ITLB_WAY, ITLB_WQ_SIZE, ITLB_RQ_SIZE, ITLB_PQ_SIZE, ITLB_MSHR_SIZE},
          DTLB{"DTLB", DTLB_SET, DTLB_WAY, DTLB_SET*DTLB_WAY, DTLB_WQ_SIZE, DTLB_RQ_SIZE, DTLB_PQ_SIZE, DTLB_MSHR_SIZE},
          STLB{"STLB", STLB_SET, STLB_WAY, STLB_SET*STLB_WAY, STLB_WQ_SIZE, STLB_RQ_SIZE, STLB_PQ_SIZE, STLB_MSHR_SIZE},
          L1I{"L1I", L1I_SET, L1I_WAY, L1I_SET*L1I_WAY, L1I_WQ_SIZE, L1I_RQ_SIZE, L1I_PQ_SIZE, L1I_MSHR_SIZE},
          L1D{"L1D", L1D_SET, L1D_WAY, L1D_SET*L1D_WAY, L1D_WQ_SIZE, L1D_RQ_SIZE, L1D_PQ_SIZE, L1D_MSHR_SIZE},
          L2C{"L2C", L2C_SET, L2C_WAY, L2C_SET*L2C_WAY, L2C_WQ_SIZE, L2C_RQ_SIZE, L2C_PQ_SIZE, L2C_MSHR_SIZE},
	  BTB{"BTB", BTB_SET, BTB_WAY, BTB_SET*BTB_WAY, 0, 0, 0, 0};

    #ifdef ARKA_DP_PRED
    CACHE VICTIM_ST{"VICTIM_ST", VICTIM_ST_SET, VICTIM_ST_WAY, VICTIM_ST_SET*VICTIM_ST_WAY, 0, 0, 0, 0};
    #endif

    #ifdef PUSH_DTLB_PB
    CACHE DTLB_PB{"DTLB_PB", DTLB_PB_SET, DTLB_PB_WAY, DTLB_PB_SET*DTLB_PB_WAY, DTLB_PB_WQ_SIZE, DTLB_PB_RQ_SIZE, DTLB_PB_PQ_SIZE, DTLB_PB_MSHR_SIZE};
    #endif
    
    #ifdef PUSH_VICTIMS_DTLB_VB
    CACHE DTLB_VB{"DTLB_VB", DTLB_VB_SET, DTLB_VB_WAY, DTLB_VB_SET*DTLB_VB_WAY, DTLB_VB_WQ_SIZE, DTLB_VB_RQ_SIZE, DTLB_VB_PQ_SIZE, DTLB_VB_MSHR_SIZE};
    #endif

    #ifdef PUSH_DTLB_UB
    CACHE DTLB_UB{"DTLB_UB", DTLB_UB_SET, DTLB_UB_WAY, DTLB_UB_SET*DTLB_UB_WAY, DTLB_UB_WQ_SIZE, DTLB_UB_RQ_SIZE, DTLB_UB_PQ_SIZE, DTLB_UB_MSHR_SIZE};
    #endif

    #ifdef DTLB_NC_BUFFER
    CACHE DTLB_NC{"DTLB_NC", DTLB_NC_SET, DTLB_NC_WAY, DTLB_NC_SET*DTLB_NC_WAY, DTLB_NC_WQ_SIZE, DTLB_NC_RQ_SIZE, DTLB_NC_PQ_SIZE, DTLB_NC_MSHR_SIZE};
    #endif
    
    PAGE_TABLE_WALKER PTW{"PTW"}; 

#ifdef ROB_STALL_STATS
    unordered_set<uint64_t> total_stall[MAX_THREADS], stlb_miss_stall[MAX_THREADS],
                            l1d_tr_miss_stall[MAX_THREADS], l1d_load_miss_stall[MAX_THREADS],
                            l2c_tr_miss_stall[MAX_THREADS], l2c_load_miss_stall[MAX_THREADS],
                            llc_tr_miss_stall[MAX_THREADS], llc_load_miss_stall[MAX_THREADS];
#endif

    // constructor
    O3_CPU() {
        cpu = 0;
	thread = 0;
        // trace
        for(int i=0; i<MAX_THREADS; i++)
		trace_file[i] = NULL;
	context_switch = 0;
	operating_index = -1;
	total_drop_from_PTW = 0;

	
        // instruction
	for(int thread_num = 0; thread_num < MAX_THREADS; thread_num++)
	{
		instr_unique_id[thread_num] = 0;
        	fetch_stall[thread_num] = 0;
		fetch_resume_cycle[thread_num] = 0;
		read_instr[thread_num] = true;
        	last_sim_cycle[thread_num] = 0;
        	last_sim_instr[thread_num] = 0;
        	finish_sim_cycle[thread_num] = 0;
        	finish_sim_instr[thread_num] = 0;
		num_retired[thread_num] = 0;
        	num_branch[thread_num] = 0;
		STA_head[thread_num] = 0;
		STA_tail[thread_num] = 0;
		STA_partition[thread_num] = 0;
        	branch_mispredictions[thread_num] = 0;
        	next_print_instruction[thread_num] = STAT_PRINTING_PERIOD;
        	begin_sim_instr[thread_num] = 0;
       		for(uint32_t type=0; type<8; type++)
			total_branch_types[thread_num][type] = 0;
	}

	last_read_trace_thread = -1;
	last_fetched_thread = -1;
	last_decoded_thread = -1;
	last_retired_thread = -1;	
	last_scheduled_thread = -1;
	completed_executions = 0;
        begin_sim_cycle = 0;
        warmup_instructions = 0;
        simulation_instructions = 0;
        instrs_to_read_this_cycle = 0;
        instrs_to_fetch_this_cycle = 0;


        inflight_reg_executions = 0;
        inflight_mem_executions = 0;
        num_searched = 0;

        next_ITLB_fetch = 0;

        // branch
        branch_mispredict_stall_fetch = 0;
        mispredicted_branch_iw_index = 0;


        for (uint32_t i=0; i<STA_SIZE; i++)
            STA[i] = UINT64_MAX;

        for (uint32_t i=0; i<ROB_SIZE; i++) {
            RTE0[i] = ROB_SIZE;
            RTE1[i] = ROB_SIZE;
        }
        RTE0_head = 0;
        RTE1_head = 0;
        RTE0_tail = 0;
        RTE1_tail = 0;

        for (uint32_t i=0; i<LQ_SIZE; i++) {
            RTL0[i] = LQ_SIZE;
            RTL1[i] = LQ_SIZE;
        }
        RTL0_head = 0;
        RTL1_head = 0;
        RTL0_tail = 0;
        RTL1_tail = 0;

        for (uint32_t i=0; i<SQ_SIZE; i++) {
            RTS0[i] = SQ_SIZE;
            RTS1[i] = SQ_SIZE;
        }
        RTS0_head = 0;
        RTS1_head = 0;
        RTS0_tail = 0;
        RTS1_tail = 0;
    }

    // functions
    void read_from_trace(),
	    //handle_branch(), Neelu: Now it is read_from_trace.
         fetch_instruction(),
	 decode_and_dispatch(),
         schedule_instruction(),
         execute_instruction(),
         schedule_memory_instruction(),
         execute_memory_instruction(),
         do_scheduling(uint32_t rob_index),  
         reg_dependency(uint32_t rob_index),
         do_execution(uint32_t rob_index),
         do_memory_scheduling(uint32_t rob_index),
         operate_lsq(),
         complete_execution(uint32_t rob_index),
         reg_RAW_dependency(uint32_t prior, uint32_t current, uint32_t source_index),
         reg_RAW_release(uint32_t rob_index),
         mem_RAW_dependency(uint32_t prior, uint32_t current, uint32_t data_index, uint32_t lq_index),
         //handle_o3_fetch(PACKET *current_packet, uint32_t cache_type), //@Vishal: This function is not used anywhere
         handle_merged_translation(PACKET *provider), 
         handle_merged_load(PACKET *provider),
         release_load_queue(uint32_t lq_index),
         complete_instr_fetch(PACKET_QUEUE *queue, uint8_t is_it_tlb),
         complete_data_fetch(PACKET_QUEUE *queue, uint8_t is_it_tlb);

    void initialize_core();
    void add_load_queue(uint32_t rob_index, uint32_t data_index),
         add_store_queue(uint32_t rob_index, uint32_t data_index),
         execute_store(uint32_t rob_index, uint32_t sq_index, uint32_t data_index);
    int  execute_load(uint32_t rob_index, uint32_t sq_index, uint32_t data_index);
    void check_dependency(int prior, int current);
    void operate_cache();
    void update_rob();
    void retire_rob();

    uint32_t  add_to_rob(ooo_model_instr *arch_instr),
              check_rob(uint64_t instr_id);

	uint32_t add_to_ifetch_buffer(ooo_model_instr *arch_instr);
	uint32_t add_to_decode_buffer(ooo_model_instr *arch_instr);

    uint32_t check_and_add_lsq(uint32_t rob_index);

    // branch predictor
    uint8_t predict_branch(uint64_t ip, uint64_t instr_id);
    void    initialize_branch_predictor(),
            last_branch_result(uint64_t ip, uint8_t taken, uint64_t instr_id); 

     void l1i_prefetcher_initialize();
     void l1i_prefetcher_branch_operate(uint64_t ip, uint8_t branch_type, uint64_t branch_target);
     void l1i_prefetcher_cache_operate(uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit);
     void l1i_prefetcher_cycle_operate();
     void l1i_prefetcher_cache_fill(uint64_t v_addr, uint32_t set, uint32_t way, uint8_t prefetch, uint64_t evicted_v_addr);
     void l1i_prefetcher_final_stats();
     int prefetch_code_line(uint64_t pf_v_addr);
	void core_final_stats();
void fill_btb(uint64_t trigger, uint64_t target, uint64_t instr_id);

};

extern O3_CPU ooo_cpu[NUM_CPUS];

#endif
