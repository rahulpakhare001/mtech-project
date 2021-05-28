
#include "ooo_cpu.h"
#include "set.h"
#include <map>
#include <set>
#include <iterator>
#include "uncore.h"


#if defined(DETECT_CRITICAL_TRANSLATIONS) || defined(DETECT_CRITICAL_IPS)
#define MIN_CRIT_CYCLE 0
//#define MAX_CRIT_CYCLE 10
#define CRIT_RECUR 1
#endif

// out-of-order core
O3_CPU ooo_cpu[NUM_CPUS]; 
uint64_t current_core_cycle[NUM_CPUS], stall_cycle[NUM_CPUS];
uint32_t SCHEDULING_LATENCY = 0, EXEC_LATENCY = 0, DECODE_LATENCY = 0;
uint8_t TRACE_ENDS_STOP = 0;
uint8_t UNIQUE_ASID[5];
int asid_index=0;
uint8_t highest_register_num = 0;
int reg_instruction_pointer = REG_INSTRUCTION_POINTER, reg_flags = REG_FLAGS, reg_stack_pointer = REG_STACK_POINTER;

uint64_t reused_translation = 0;
set<uint64_t> total_ips[NUM_CPUS][MAX_THREADS];
set<uint64_t> critical_ips[NUM_CPUS][MAX_THREADS];
map<uint64_t, uint64_t> crit_ip_stlb_miss[NUM_CPUS][MAX_THREADS];

set<uint64_t> total_translation[NUM_CPUS][MAX_THREADS];
set<uint64_t> critical_translation[NUM_CPUS][MAX_THREADS];
map<uint64_t, uint64_t> critical_trans_0_10[NUM_CPUS][MAX_THREADS]; 
map<uint64_t, uint64_t> critical_trans_10_50[NUM_CPUS][MAX_THREADS]; 
map<uint64_t, uint64_t> critical_trans_50_100[NUM_CPUS][MAX_THREADS]; 
map<uint64_t, uint64_t> critical_trans_100_200[NUM_CPUS][MAX_THREADS];
map<uint64_t, uint64_t> critical_trans_200_500[NUM_CPUS][MAX_THREADS];
map<uint64_t, uint64_t> critical_trans_500_[NUM_CPUS][MAX_THREADS];

map<uint64_t, uint64_t> per_crit_ip_stall[MAX_THREADS];
map<uint64_t, uint64_t> per_crit_trans_stall[MAX_THREADS];

uint64_t global_stall_cycles_count[NUM_CPUS], rob_stall_count[NUM_CPUS], load_rob_stall_count[NUM_CPUS], total_rob_stall_cycles[NUM_CPUS], total_load_rob_stall_cycles[NUM_CPUS][MAX_THREADS] = {0}, load_rob_stall_prob_distri[NUM_CPUS][MAX_THREADS][6];
uint64_t translation_stalled[NUM_CPUS][MAX_THREADS];
//uint64_t cycle_stalled[NUM_CPUS][MAX_THREADS];
uint32_t way_stalled[NUM_CPUS][MAX_THREADS];
uint64_t total_rob_occupancy[NUM_CPUS];

void O3_CPU::initialize_core()
{
}




void O3_CPU::read_from_trace()
{
    // actual processors do not work like this but for easier implementation,
    // we read instruction traces and virtually add them in the ROB
    // note that these traces are not yet translated and fetched 

    uint8_t continue_reading = 1;
    uint32_t num_reads = 0;
    instrs_to_read_this_cycle = FETCH_WIDTH;
    uint32_t fetch_thread = -1, min_occupancy = UINT8_MAX;

/*    //@Vasudha:SMT: In case of corrent branch prediction, further instructions in a trace file can be resumed
    for (uint16_t thread_num = 0; thread_num < thread; thread_num++)
    {
	    //@Vasudha:SMT: Every thread has a separate instruction fetch buffer. Select the one which has least occupancy assuming that it is utilizing the pipeline efficiently
	   // if (IFETCH_BUFFER[thread_num].occupancy < min_occupancy && fetch_stall[thread_num] == 0 && simulation_complete[cpu][thread_num] == 0)
	    if (IFETCH_BUFFER[thread_num].occupancy < min_occupancy && fetch_stall[thread_num] == 0)
	    {
		    //@Vasudha:SMT: TODO:handle end of trace thriugh num_retired
		    min_occupancy = IFETCH_BUFFER[thread_num].occupancy;
		    fetch_thread = thread_num;
	    }
    }
*/
    //@Vasudha:SMT: In case of corrent branch prediction, further instructions in a trace file can be resumed
    uint32_t thread_num = last_read_trace_thread + 1;
    uint32_t count = 0;
    while(count < ooo_cpu[cpu].thread)
    {
            if (thread_num == ooo_cpu[cpu].thread)
                thread_num = 0;

            //@Vasudha:SMT: Every thread has a separate instruction fetch buffer. Select the one which has least occupancy assuming that it is utilizing the pipeline efficiently
            if (IFETCH_BUFFER[thread_num].occupancy < min_occupancy && fetch_stall[thread_num] == 0)
            {
                    min_occupancy = IFETCH_BUFFER[thread_num].occupancy;
                    fetch_thread = thread_num;
                    last_read_trace_thread = thread_num;
	    }

            thread_num++;
            count++;
    }

    assert(fetch_thread < ooo_cpu[cpu].thread && fetch_thread >= 0);
    assert(IFETCH_BUFFER[fetch_thread].occupancy < IFETCH_BUFFER[fetch_thread].SIZE);

    //cout << " Cycle = " << current_core_cycle[cpu] << " thread active= " << fetch_thread << endl;
    // first, read PIN trace
    while (continue_reading) {

        size_t instr_size = knob_cloudsuite ? sizeof(cloudsuite_instr) : sizeof(input_instr);
	
        if (knob_cloudsuite) {
            if (!fread(&current_cloudsuite_instr, instr_size, 1, trace_file[fetch_thread])) {
                // reached end of file for this trace
                cout << "*** Reached end of trace for Core: " << cpu << " Repeating trace: " << trace_string << endl; 
		//TRACE_ENDS_STOP = 1; /*@Vasudha - STOP simulation once trace file ends*/
                // close the trace file and re-open it
                pclose(trace_file[fetch_thread]);
		//return; /*@Vasudha */
                trace_file[fetch_thread] = popen(gunzip_command[fetch_thread], "r");
                if (trace_file[fetch_thread] == NULL) {
                    cerr << endl << "*** CANNOT REOPEN TRACE FILE: " << trace_string[fetch_thread] << " ***" << endl;
                    assert(0);
                }
            } else { // successfully read the trace

                // copy the instruction into the performance model's instruction format
                ooo_model_instr arch_instr;
                int num_reg_ops = 0, num_mem_ops = 0;

		//@Vasudha:SMT: append THREAD_ID with instr_id
                arch_instr.instr_id = (instr_unique_id[fetch_thread] << LOG2_THREADS) | fetch_thread;
                arch_instr.ip = current_cloudsuite_instr.ip;
                arch_instr.is_branch = current_cloudsuite_instr.is_branch;
                arch_instr.branch_taken = current_cloudsuite_instr.branch_taken;

                arch_instr.asid[0] = current_cloudsuite_instr.asid[0];
                arch_instr.asid[1] = current_cloudsuite_instr.asid[1];


		//cout << fetch_thread << "-" << (arch_instr.instr_id>>LOG2_THREADS) <<"-"<<arch_instr.ip<<"-"<< arch_instr.is_branch<<"-"<<arch_instr.branch_taken<<"-"<<cpu<<endl;

		total_ips[cpu][0].insert(arch_instr.ip);

                for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
		//@Vasudha:SMT: append THREAD-ID with register numbers to differentiate between same regsters of different threads while checking data dpendency
                    arch_instr.destination_registers[i] = (current_cloudsuite_instr.destination_registers[i] << LOG2_THREADS) | fetch_thread;
		    arch_instr.destination_memory[i] = current_cloudsuite_instr.destination_memory[i];
                    arch_instr.destination_virtual_address[i] = current_cloudsuite_instr.destination_memory[i];

                    if (arch_instr.destination_registers[i])
                        num_reg_ops++;
                    if (arch_instr.destination_memory[i]) {
                        num_mem_ops++;

                        // update STA, this structure is required to execute store instructios properly without deadlock
                        if (num_mem_ops > 0) {
#ifdef SANITY_CHECK
                            if (STA[STA_tail[fetch_thread]] < UINT64_MAX) {
                                if (STA_head[fetch_thread] != STA_tail[fetch_thread])
                                    assert(0);
                            }
#endif
                            STA[STA_tail[fetch_thread]] = instr_unique_id[fetch_thread];
                            STA_tail[fetch_thread]++;

                            if (STA_tail[fetch_thread] == STA_partition[fetch_thread] + 1)
                                STA_tail[fetch_thread] = fetch_thread == 0 ? 0 : STA_partition[fetch_thread-1] + 1;;
                        }
                    }
                }

                for (int i=0; i<NUM_INSTR_SOURCES; i++) {
		//@Vasudha:SMT: append THREAD-ID with register numbers to differentiate between same regsters of different threads while checking data dpendency
                    arch_instr.source_registers[i] = (current_cloudsuite_instr.source_registers[i] << LOG2_THREADS) | fetch_thread;
                    arch_instr.source_memory[i] = current_cloudsuite_instr.source_memory[i];
                    arch_instr.source_virtual_address[i] = current_cloudsuite_instr.source_memory[i];
		    if (knob_cloudsuite)
		   	    total_translation[cpu][0].insert(arch_instr.source_virtual_address[i] >> LOG2_PAGE_SIZE);
		    else
			    total_translation[cpu][fetch_thread].insert(arch_instr.source_virtual_address[i] >> LOG2_PAGE_SIZE);
		    if (arch_instr.source_registers[i])
                        num_reg_ops++;
                    if (arch_instr.source_memory[i])
                        num_mem_ops++;
                }

                arch_instr.num_reg_ops = num_reg_ops;
                arch_instr.num_mem_ops = num_mem_ops;
                if (num_mem_ops > 0) 
                    arch_instr.is_memory = 1;

//Neelu: Addition begin

		// add this instruction to the IFETCH_BUFFER
		if (IFETCH_BUFFER[fetch_thread].occupancy < IFETCH_BUFFER[fetch_thread].SIZE) {
			uint32_t ifetch_buffer_index = add_to_ifetch_buffer(&arch_instr);
			num_reads++;
							                      // handle branch prediction

			if (IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].is_branch) {
				//DP( if (warmup_complete[cpu][fetch_thread]) {
				//	cout << "[BRANCH] instr_id: " << instr_unique_id[fetch_thread] << " thread: " << fetch_thread << " ip: " << hex << arch_instr.ip;
				  //     	cout << dec << " taken: " << +arch_instr.branch_taken << endl; });
			
				num_branch[fetch_thread]++;	
				// handle branch prediction & branch predictor update

				uint8_t branch_prediction = predict_branch(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].ip, IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].instr_id );
	    			if(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_taken != branch_prediction)
	   			{
	       				branch_mispredictions[fetch_thread]++;
					total_rob_occupancy_at_branch_mispredict[fetch_thread] += ROB.occupancy[fetch_thread];
		 			if(warmup_complete[cpu][fetch_thread])
      					{
  						fetch_stall[fetch_thread] = 1;
						//@Vasudha:SMT:Commenting as instructions from different threads can be read on the same cycle 
						instrs_to_read_this_cycle = 0;
						//read_instr[fetch_thread] = false;
						IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_mispredicted = 1;
					}
	  			}
	    			else
				{
					//correct prediction
					if(branch_prediction == 1)
					{
						// if correctly predicted taken, then we can't fetch anymore instructions this cycle
		                                //@Vasudha:SMT: Commenting as instructions from different threads can be read on the same cycle
						instrs_to_read_this_cycle = 0;
					}
				}

			last_branch_result(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].ip, IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_taken, 
					IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].instr_id);
			}


				

                    //if ((num_reads == FETCH_WIDTH) || (ROB.occupancy == ROB.SIZE))
                    if ((num_reads >= instrs_to_read_this_cycle) || (IFETCH_BUFFER[fetch_thread].occupancy == IFETCH_BUFFER[fetch_thread].SIZE))
                        continue_reading = 0;
                }
                instr_unique_id[fetch_thread]++;
            }
        }
	else
	  {
	   	/* context-switch code*/
		if(ooo_cpu[cpu].context_switch)
		{
			////cout << "context_switch = " << cpu <<" cycle="<<current_core_cycle[cpu]<<"ROB-occupancy = "<< ROB.occupancy <<  endl;
	               
			break;
		} 
		input_instr trace_read_instr;
            if (!fread(&trace_read_instr, instr_size, 1, trace_file[fetch_thread])) {
                // reached end of file for this trace
                //cout << "*** Reached end of trace for Core: " << cpu << " Repeating trace: " << trace_string << endl; 

                // close the trace file and re-open it
                pclose(trace_file[fetch_thread]);
                trace_file[fetch_thread] = popen(gunzip_command[fetch_thread], "r");
                if (trace_file[fetch_thread] == NULL) {
                    cerr << endl << "*** CANNOT REOPEN TRACE FILE: " << trace_string[fetch_thread] << " ***" << endl;
                    assert(0);
                }
            } else { // successfully read the trace

//	            current_instr = trace_read_instr;
        	    if(instr_unique_id[fetch_thread] == 0)
		    {
			    current_instr = next_instr = trace_read_instr;
		    }
		    else
		    {
			    current_instr = next_instr;
			    next_instr = trace_read_instr;
		    }



                // copy the instruction into the performance model's instruction format
                ooo_model_instr arch_instr;
                int num_reg_ops = 0, num_mem_ops = 0;

		//@Vasudha:SMT: append THREAD_ID with instr_id
                arch_instr.instr_id = (instr_unique_id[fetch_thread] << LOG2_THREADS) | fetch_thread;
                arch_instr.ip = current_instr.ip;
		arch_instr.is_branch = current_instr.is_branch;
                arch_instr.branch_taken = current_instr.branch_taken;

                arch_instr.asid[0] = cpu;
                arch_instr.asid[1] = cpu;

		bool reads_sp = false;
		bool writes_sp = false;
		bool reads_flags = false;
		bool reads_ip = false;
		bool writes_ip = false;
		bool reads_other = false;

		//cout << fetch_thread << "-" << (arch_instr.instr_id>>LOG2_THREADS) <<"-"<<arch_instr.ip<<"-"<< arch_instr.is_branch<<"-"<<arch_instr.branch_taken<<"-"<<cpu<<endl;
       	
		total_ips[cpu][fetch_thread].insert(arch_instr.ip);
       		
		for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
		//@Vasudha:SMT: append THREAD-ID with register numbers to differentiate between same regsters of different threads while checking data dpendency
                    arch_instr.destination_registers[i] = ((uint16_t)current_instr.destination_registers[i] << LOG2_THREADS) | fetch_thread;
                    arch_instr.destination_memory[i] = current_instr.destination_memory[i];
                    arch_instr.destination_virtual_address[i] = current_instr.destination_memory[i];

		if(arch_instr.destination_registers[i] >> LOG2_THREADS == reg_stack_pointer)
                writes_sp = true;
		else if(arch_instr.destination_registers[i] >> LOG2_THREADS == reg_instruction_pointer)
                writes_ip = true;


                    if (arch_instr.destination_registers[i])
                        num_reg_ops++;
                    if (arch_instr.destination_memory[i]) {
                        num_mem_ops++;
                        // update STA, this structure is required to execute store instructios properly without deadlock
                        if (num_mem_ops > 0) {
#ifdef SANITY_CHECK
                            if (STA[STA_tail[fetch_thread]] < UINT64_MAX) {
                                if (STA_head[fetch_thread] != STA_tail[fetch_thread])
                                    assert(0);
                            }
#endif
                            STA[STA_tail[fetch_thread]] = instr_unique_id[fetch_thread];
                            STA_tail[fetch_thread]++;

                            if (STA_tail[fetch_thread] == STA_partition[fetch_thread] + 1)
                                STA_tail[fetch_thread] = fetch_thread == 0 ? 0 : STA_partition[fetch_thread-1] + 1 ;
                        }
                    }
                }

                for (int i=0; i<NUM_INSTR_SOURCES; i++) {
		//@Vasudha:SMT: append THREAD-ID with register numbers to differentiate between same regsters of different threads while checking data dpendency
                    arch_instr.source_registers[i] = ((uint16_t)current_instr.source_registers[i] << LOG2_THREADS) | fetch_thread;
                    arch_instr.source_memory[i] = current_instr.source_memory[i];
                    arch_instr.source_virtual_address[i] = current_instr.source_memory[i];
		    //if (total_translation[cpu][fetch_thread].find(arch_instr.source_virtual_address[i] >> LOG2_PAGE_SIZE) != total_translation[cpu][fetch_thread].end())
			//    ++reused_translation;
		    total_translation[cpu][fetch_thread].insert(arch_instr.source_virtual_address[i] >> LOG2_PAGE_SIZE);

			if(arch_instr.source_registers[i] >> LOG2_THREADS == reg_stack_pointer)
				reads_sp = true;
			else if(arch_instr.source_registers[i] >> LOG2_THREADS == reg_flags)
                		reads_flags = true;
			else if(arch_instr.source_registers[i] >> LOG2_THREADS == reg_instruction_pointer)
                		reads_ip = true;
			else if(arch_instr.source_registers[i] >> LOG2_THREADS != 0)
                		reads_other = true;


                    if (arch_instr.source_registers[i])
                        num_reg_ops++;
                    if (arch_instr.source_memory[i])
                        num_mem_ops++;
                }

                arch_instr.num_reg_ops = num_reg_ops;
                arch_instr.num_mem_ops = num_mem_ops;
                if (num_mem_ops > 0) 
                    arch_instr.is_memory = 1;


		// determine what kind of branch this is, if any
		if(!reads_sp && !reads_flags && writes_ip && !reads_other)
		{
			// direct jump
			arch_instr.is_branch = 1; 
			arch_instr.branch_taken = 1;
			arch_instr.branch_type = BRANCH_DIRECT_JUMP;
		}
		else if(!reads_sp && !reads_flags && writes_ip && reads_other)
		{
			// indirect branch
			arch_instr.is_branch = 1; 
			arch_instr.branch_taken = 1;
			arch_instr.branch_type = BRANCH_INDIRECT;
		}
		else if(!reads_sp && reads_ip && !writes_sp && writes_ip && reads_flags && !reads_other)
		{
			// conditional branch
			arch_instr.is_branch = 1;
			arch_instr.branch_taken = arch_instr.branch_taken; // don't change this
			arch_instr.branch_type = BRANCH_CONDITIONAL;
		}
		else if(reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && !reads_other)
		{
			// direct call
			arch_instr.is_branch = 1;
    			arch_instr.branch_taken = 1;
    			arch_instr.branch_type = BRANCH_DIRECT_CALL;

  		}
                else if(reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && reads_other)
		{
			// indirect call
			arch_instr.is_branch = 1;
			arch_instr.branch_taken = 1;
			arch_instr.branch_type = BRANCH_INDIRECT_CALL;
		}
		else if(reads_sp && !reads_ip && writes_sp && writes_ip)
		{
			//return
			arch_instr.is_branch = 1;
			arch_instr.branch_taken = 1;
			arch_instr.branch_type = BRANCH_RETURN;
		}
		else if(writes_ip)
		{
			// some other branch type that doesn't fit the above categories
			arch_instr.is_branch = 1;
			arch_instr.branch_taken = arch_instr.branch_taken; // don't change this
			arch_instr.branch_type = BRANCH_OTHER;
		}

		total_branch_types[fetch_thread][arch_instr.branch_type]++;

		if((arch_instr.is_branch == 1) && (arch_instr.branch_taken == 1))
		{
			arch_instr.branch_target = next_instr.ip;
		}


		//Neelu: Inserting instruction to IFETCH BUFFER

		// add this instruction to the IFETCH_BUFFER
		if (IFETCH_BUFFER[fetch_thread].occupancy < IFETCH_BUFFER[fetch_thread].SIZE) {
			uint32_t ifetch_buffer_index = add_to_ifetch_buffer(&arch_instr);
			num_reads++;
	
	 		// handle branch prediction

			if (IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].is_branch) {

				//DP( if (warmup_complete[cpu][fetch_thread]) {
				//cout << "[BRANCH] instr_id: " << instr_unique_id[fetch_thread] << " thread: " << fetch_thread << " ip: " << hex << arch_instr.ip;
			        //cout << dec << " taken: " << +arch_instr.branch_taken << endl; });
		                        num_branch[fetch_thread]++;
			// handle branch prediction & branch predictor update
			uint8_t branch_prediction = predict_branch(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].ip, IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].instr_id);
			uint64_t predicted_branch_target = IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_target;
			if(branch_prediction == 0)
			{
				predicted_branch_target = 0;
			}
									                        // call code prefetcher every time the branch predictor is used
			l1i_prefetcher_branch_operate(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].ip, IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_type, predicted_branch_target);


#ifndef PERFECT_BTB
			if(arch_instr.branch_taken)
			{
					uint32_t btb_set = BTB.get_set(arch_instr.ip);
					int btb_way = BTB.get_way(arch_instr.ip, btb_set, arch_instr.instr_id);
					if(btb_way == BTB_WAY)
					{
						BTB.sim_miss[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
						BTB.sim_access[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
						if(warmup_complete[cpu][fetch_thread])
						{
							fetch_stall[fetch_thread] = 1;
							instrs_to_read_this_cycle = 0;
							IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].btb_miss = 1;
						}
					}
					else
					{
						if(BTB.block[btb_set][btb_way].data == IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_target)
						{
								BTB.sim_hit[cpu][fetch_thread][ arch_instr. branch_type - 1]++;
								BTB.sim_access[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
								(BTB.*BTB.update_replacement_state)(cpu, btb_set, btb_way, arch_instr.ip, arch_instr.ip, 0, 0, 1);
						}
						else
						{
							BTB.sim_miss[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
							BTB.sim_access[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
							if(warmup_complete[cpu][fetch_thread])
								{
									fetch_stall[fetch_thread] = 1;
		                                			//@Vasudha:SMT: Commenting as instructions from different threads can be read on the same cycle
									instrs_to_read_this_cycle = 0;
									IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].btb_miss = 1;
								}
						}
					}
			}
			else
			{
				uint32_t btb_set = BTB.get_set(arch_instr.ip);
                int btb_way = BTB.get_way(arch_instr.ip, btb_set, arch_instr.instr_id);

				if(btb_way != BTB_WAY)
				{
					BTB.sim_hit[cpu][fetch_thread][ arch_instr. branch_type - 1]++;
                    			BTB.sim_access[cpu][fetch_thread][ arch_instr.branch_type - 1]++;
                    			(BTB.*BTB.update_replacement_state)(cpu, btb_set, btb_way, arch_instr.ip, arch_instr.ip, 0, 0, 1);
				}
			}

#endif





			if(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_taken != branch_prediction)
			{
				branch_mispredictions[fetch_thread]++;
				total_rob_occupancy_at_branch_mispredict[fetch_thread] += ROB.occupancy[fetch_thread];
				if(warmup_complete[cpu][fetch_thread])
				{
					fetch_stall[fetch_thread] = 1;
					//@Vasudha:SMT: Commenting as instructions from different threads can be read on the same cycle
					instrs_to_read_this_cycle = 0;
					IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_mispredicted = 1;
				}
			}
			else
			{
				// correct prediction

				if(branch_prediction == 1)
				{
					// if correctly predicted taken, then we can't fetch anymore instructions this cycle
					//@Vasudha:SMT: Commenting as instructions from different threads can be read on the same cycle
					instrs_to_read_this_cycle = 0;
				}
			}

			last_branch_result(IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].ip, IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].branch_taken, 
					IFETCH_BUFFER[fetch_thread].entry[ifetch_buffer_index].instr_id);
			}

                    //if ((num_reads == FETCH_WIDTH) || (ROB.occupancy == ROB.SIZE))
                    if ((num_reads >= instrs_to_read_this_cycle) || (IFETCH_BUFFER[fetch_thread].occupancy == IFETCH_BUFFER[fetch_thread].SIZE))
                        continue_reading = 0;
                }
                instr_unique_id[fetch_thread]++;
            }
        }
    }
//////cout<<endl;
    //instrs_to_fetch_this_cycle = num_reads;
}

uint32_t O3_CPU::add_to_rob(ooo_model_instr *arch_instr)
{
	int thread_num = ((1 << LOG2_THREADS) - 1) & (*arch_instr).instr_id;
    	uint32_t index = ROB.tail[thread_num];

    	// sanity check
    	if (ROB.entry[index].instr_id != 0) {
        	cerr << "[ROB_ERROR] " << __func__ << " is not empty index: " << index;
        	cerr << " instr_id: " << ROB.entry[index].instr_id << endl;
         	// print ROB entry
    		//cout << endl << "ROB Entry" << endl;
    		//for (uint32_t j=0; j < ROB_SIZE; j++)
        		//cout << "[ROB] entry: " << j << " instr_id: " << (ROB.entry[j].instr_id >> LOG2_THREADS) << " is_memory = " << ROB.entry[j].is_memory << " scheduled: " << ROB.entry[j].scheduled << " executed = " << ROB.entry[j].executed << " ip: " << ROB.entry[j].ip << " event_cycle = " << ROB.entry[j].event_cycle << endl;
		//assert(0);
    	}

    	ROB.entry[index] = *arch_instr;
    	ROB.entry[index].event_cycle = current_core_cycle[cpu];

    	ROB.occupancy[thread_num]++;
    	ROB.tail[thread_num]++;
	if (ROB.tail[thread_num] == ROB.partition[thread_num] + 1)
	{
		if (thread_num == 0)
			ROB.tail[thread_num] = 0;
		else
        		ROB.tail[thread_num] = ROB.partition[thread_num-1] + 1;
	}

    	//DP ( if (warmup_complete[cpu][thread_num]) {
    	//cout << "[ROB] " <<  __func__ << " instr_id: " << (ROB.entry[index].instr_id >> LOG2_THREADS);
    	//cout << " ip: " << hex << ROB.entry[index].ip << dec;
    	//cout << " head: " << ROB.head[thread_num] << " tail: " << ROB.tail[thread_num] << " occupancy: " << ROB.occupancy[thread_num];
    	//cout << " event: " << ROB.entry[index].event_cycle << " current: " << current_core_cycle[cpu] << endl; });

#ifdef SANITY_CHECK
    if (ROB.entry[index].ip == 0) {
        cerr << "[ROB_ERROR] " << __func__ << " ip is zero index: " << index;
        cerr << " instr_id: " << ROB.entry[index].instr_id << " ip: " << ROB.entry[index].ip << endl;
        assert(0);
    }
#endif

    return index;
}


uint32_t O3_CPU::add_to_ifetch_buffer(ooo_model_instr *arch_instr)
{
	uint16_t fetch_thread = ((1 << LOG2_THREADS) - 1) & (*arch_instr).instr_id;

	uint32_t index = IFETCH_BUFFER[fetch_thread].tail;
	if(IFETCH_BUFFER[fetch_thread].entry[index].instr_id != 0)
	{
		  cerr << "[IFETCH_BUFFER_ERROR] " << __func__ << " is not empty index: " << index;
		  cerr << " instr_id: " << IFETCH_BUFFER[fetch_thread].entry[index].instr_id << endl;
		  assert(0);
	}

	IFETCH_BUFFER[fetch_thread].entry[index] = *arch_instr;
	IFETCH_BUFFER[fetch_thread].entry[index].event_cycle = current_core_cycle[cpu];

	//Neelu: Instead of translating instructions magically, translating them as usual to pay ITLB access penalty. 
	IFETCH_BUFFER[fetch_thread].entry[index].instruction_pa = 0;
        IFETCH_BUFFER[fetch_thread].entry[index].translated = 0;
	IFETCH_BUFFER[fetch_thread].entry[index].fetched = 0;

	IFETCH_BUFFER[fetch_thread].occupancy++;
	IFETCH_BUFFER[fetch_thread].tail++;

	if(IFETCH_BUFFER[fetch_thread].tail >= IFETCH_BUFFER[fetch_thread].SIZE)
	{
		IFETCH_BUFFER[fetch_thread].tail = 0;
	}
        //DP ( if (warmup_complete[cpu][fetch_thread]) {
        //cout << "[IFETCH_BUFFER["<<fetch_thread<<"]] " << __func__ << " inserted instr_id: " << (IFETCH_BUFFER[fetch_thread].entry[index].instr_id >> 3) << endl; });

      return index;
}

uint32_t O3_CPU::add_to_decode_buffer(ooo_model_instr *arch_instr)
{
	  uint32_t index = DECODE_BUFFER.tail;

	  if(DECODE_BUFFER.entry[index].instr_id != 0)
	  {
		  cerr << "[DECODE_BUFFER_ERROR] " << __func__ << " is not empty index: " << index;
		  cerr << " instr_id: " << IFETCH_BUFFER[arch_instr->instr_id & ((1<<LOG2_THREADS) - 1)].entry[index].instr_id << endl;
		  assert(0);
	  }

	  DECODE_BUFFER.entry[index] = *arch_instr;
	  DECODE_BUFFER.entry[index].event_cycle = current_core_cycle[cpu];

	  DECODE_BUFFER.occupancy++;
	  DECODE_BUFFER.tail++;
	  if(DECODE_BUFFER.tail >= DECODE_BUFFER.SIZE)
	  {
		  DECODE_BUFFER.tail = 0;
	  }

	  uint16_t thread_num = ((1 << LOG2_THREADS) - 1) & DECODE_BUFFER.entry[index].instr_id;
          //DP ( if (warmup_complete[cpu][thread_num]) {
            //    cout << "[DECODE_BUFFER]-T" << thread_num << " " << __func__ <<  " inserted instr_id: " << (DECODE_BUFFER.entry[index].instr_id >> 3) << endl;});
	  return index;
}



uint32_t O3_CPU::check_rob(uint64_t instr_id)
{
    int thread_num = ((1 << LOG2_THREADS) - 1) & instr_id;
    if ((ROB.head[thread_num] == ROB.tail[thread_num]) && ROB.occupancy[thread_num] == 0)
        return ROB.SIZE;

    if (ROB.head[thread_num] < ROB.tail[thread_num]) {
        for (uint32_t i=ROB.head[thread_num]; i<ROB.tail[thread_num]; i++) {
            if (ROB.entry[i].instr_id == instr_id) {
                //DP ( if (warmup_complete[ROB.cpu][thread_num]) {
                ////cout << "[ROB] " << __func__ << " same instr_id: " << ROB.entry[i].instr_id;
                ////cout << " rob_index: " << i << endl; });
                return i;
            }
        }
    }
    else {
        for (uint32_t i=ROB.head[thread_num]; i<=ROB.partition[thread_num]; i++) {
            if (ROB.entry[i].instr_id == instr_id) {
                //DP ( if (warmup_complete[cpu][thread_num]) {
                ////cout << "[ROB] " << __func__ << " same instr_id: " << ROB.entry[i].instr_id;
                ////cout << " rob_index: " << i << endl; });
                return i;
            }
        }
        for (uint32_t i = (thread_num == 0) ? 0 : ROB.partition[thread_num-1] + 1; i<ROB.tail[thread_num]; i++) {
            if (ROB.entry[i].instr_id == instr_id) {
                //DP ( if (warmup_complete[cpu][thread_num]) {
                ////cout << "[ROB] " << __func__ << " same instr_id: " << ROB.entry[i].instr_id;
                ////cout << " rob_index: " << i << endl; });
                return i;
            }
        }
    }
    
    cerr << "[ROB_ERROR] " << __func__ << " does not have any matching index! " << " thread: " << thread_num <<" head=" << ROB.head[thread_num] << " tail="<<ROB.tail[thread_num]<<endl;
    cerr << " instr_id: " << instr_id << endl;
    assert(0);

    return ROB.SIZE;
}

void O3_CPU::fetch_instruction()
{
    // TODO: can we model wrong path execusion?

	
  // if we had a branch mispredict, turn fetching back on after the branch mispredict penalty
  //////////cout <<"prev, fetch_stall="<<fetch_stall<<" "<<current_core_cycle[cpu]<<" "<<fetch_resume_cycle<<endl;

	//@Vasudha:SMT: Update all threads
	for(uint16_t thread_num=0; thread_num < thread; thread_num++)
	{
      		if((fetch_stall[thread_num] == 1) && (current_core_cycle[cpu] >= fetch_resume_cycle[thread_num]) && (fetch_resume_cycle[thread_num] != 0))
    		{
	   		//cout<<"fetch_resume at "<< fetch_resume_cycle<<" " ;
      			fetch_stall[thread_num] = 0;
			//read_instr[thread_num] = true;
      			fetch_resume_cycle[thread_num] = 0;
      			//////cout <<"****noww, fetch_stall="<<fetch_stall<<" "<<current_core_cycle[cpu]<<" "<<fetch_resume_cycle<<endl;
    		}
	}



	uint32_t ifetch_buffer_index;
	uint32_t thread_index = last_fetched_thread + 1;
	if (thread_index == ooo_cpu[cpu].thread)
		thread_index = 0;
		//cout << "Caught : INDEX  " << thread_index << " \n\n ";
	uint64_t count = 0;
	//@Vasudha:SMT: Fetch instruction from all thread's IFETCH_BUFFER in round-robin fashion
	for(uint32_t i=0; i < IFETCH_WIDTH; i++)
	{
		count++;
		//check for infinite loop
		if(count > IFETCH_WIDTH * thread)
		{
			//cout << "Breaking fetch_instruction because of going for infinite loop\n";
			break;	
		}

		if (IFETCH_BUFFER[thread_index].occupancy == 0)
		{
			i--;
			thread_index++;
			if (thread_index == thread) // && thread != 1)
			{
				thread_index = 0;
			}
			continue;
			
		}

		//Check for packet in IFETCH_BUFFER with a valid IP that is not fetched 
		ifetch_buffer_index = IFETCH_BUFFER[thread_index].head;
		if (IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip == 0 || IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].fetched != 0)
		{
			ifetch_buffer_index++;
			if(ifetch_buffer_index == IFETCH_BUFFER[thread_index].SIZE)
				ifetch_buffer_index = 0;
			uint8_t flag = 0;
			while (ifetch_buffer_index != IFETCH_BUFFER[thread_index].head)
			{
				if (IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip != 0)
				{
					if (IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].fetched == 0)
					{
						flag = 1;
						break;
					}
				}
				ifetch_buffer_index++;
				if (ifetch_buffer_index == IFETCH_BUFFER[thread_index].SIZE)
					ifetch_buffer_index = 0;
			}
			if (flag == 0)	//No packet available in this IFETCH_BUFFER, search other thread's IFETCH_BUFFER
			{
				i--;
				thread_index++;
				if (thread_index == thread && thread != 1)
				{
					thread_index = 0;
					continue;
				}
				else if (thread_index == thread && thread == 1)
				{
					thread_index = 0;
					break;
				}
				else
					continue;	//New thread, check conditions again
			}
			
		}

		//Neelu: From IFETCH buffer, requests will be sent to L1-I and it will take care of sending the translation request to ITLB as L1 caches are VIPT in this version. 

		if((IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].fetched == 0))
		{
			// add it to the L1-I's read queue
			PACKET fetch_packet;
			fetch_packet.instruction = 1;
			fetch_packet.is_data = 0;
			fetch_packet.fill_level = FILL_L1;
			fetch_packet.fill_l1i = 1;
			fetch_packet.cpu = cpu;
			fetch_packet.address = IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip >> 6;
			fetch_packet.full_addr = IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip;
			fetch_packet.full_virtual_address = IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip;
			//@Vasudha:SMT: Thread-ID should be communicated, so instr_id needs to be send to L1I_RQ
			fetch_packet.instr_id = IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].instr_id;
			//fetch_packet.instr_id = 0;
			//Neelu: Is assigning rob_index = 0 going to cause any problems? 
			fetch_packet.rob_index = 0;
			fetch_packet.producer = 0;
			fetch_packet.ip = IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip;
			fetch_packet.type = LOAD;
			fetch_packet.asid[0] = 0;
			fetch_packet.asid[1] = 0;
			fetch_packet.event_cycle = current_core_cycle[cpu];

			int rq_index = L1I.add_rq(&fetch_packet);
			if(rq_index != -2)
			{
				// mark all instructions from this cache line as having been fetched
				for(uint32_t j=0; j<IFETCH_BUFFER[thread_index].SIZE; j++)
				{
					if(((IFETCH_BUFFER[thread_index].entry[j].ip)>>6) == ((IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].ip)>>6))
					{
						IFETCH_BUFFER[thread_index].entry[j].translated = COMPLETED;
						IFETCH_BUFFER[thread_index].entry[j].fetched = INFLIGHT;
					}
				}
			}
		}
		thread_index++;
		if(thread_index == thread)
			thread_index = 0;
	}	//Neelu: For loop ending. 
	last_fetched_thread = thread_index;

	//send to decode stage
	//@Vasudha:SMT: Select instruction fetch buffer of a thread in round-robin fashion
	bool decode_full = false;
	uint16_t num_of_iterations;
	thread_index = last_decoded_thread + 1;
	if (thread_index == ooo_cpu[cpu].thread)
		thread_index = 0;
	count = 0;
	for(uint32_t i=0; i<DECODE_WIDTH; i++)
	{
		count++;
		//@Vasudha: Check for infinite loop
		if (count > DECODE_WIDTH * thread)
			break;
		if(decode_full)
		{
			break;
		}
	
		num_of_iterations = 0;
		while (IFETCH_BUFFER[thread_index].entry[IFETCH_BUFFER[thread_index].head].ip == 0)
		{
			thread_index++;
			if (thread_index == thread)
				thread_index = 0;
			if (num_of_iterations == thread)
			{
				return;
			}
			num_of_iterations++;
		}

		ifetch_buffer_index = IFETCH_BUFFER[thread_index].head;

		if((IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].translated == COMPLETED) && (IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index].fetched == COMPLETED))
		{
			if(DECODE_BUFFER.occupancy < DECODE_BUFFER.SIZE)
			{
				uint32_t decode_index = add_to_decode_buffer(&IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index]);
				DECODE_BUFFER.entry[decode_index].event_cycle = 0;
				//cout << "DECODING THREAD " << thread_index << endl;
				ooo_model_instr empty_entry;
				IFETCH_BUFFER[thread_index].entry[ifetch_buffer_index] = empty_entry;
				IFETCH_BUFFER[thread_index].head++;
				if(IFETCH_BUFFER[thread_index].head >= IFETCH_BUFFER[thread_index].SIZE)
				{
					IFETCH_BUFFER[thread_index].head = 0;
				}
				IFETCH_BUFFER[thread_index].occupancy--;
			}
			else
			{
				decode_full = true;
			}

		}
		else
			i--;

		thread_index++;
		if(thread_index == thread)
		{
			thread_index = 0;
		}
	}
	last_decoded_thread = thread_index;
	
	//if (last_decoded_thread < 0)
	//	last_decoded_thread = thread - 1;
} //Ending of fetch_instruction()



    // add this request to ITLB
    // @Vishal: Transalation will be handled by L1D cache (VIPT)

	//Neelu: Comment started from here. 
    /*uint32_t read_index = (ROB.last_read == (ROB.SIZE-1)) ? 0 : (ROB.last_read + 1);
    for (uint32_t i=0; i<FETCH_WIDTH; i++) {

        if (ROB.entry[read_index].ip == 0)
            break;

#ifdef SANITY_CHECK
        // sanity check
        if (ROB.entry[read_index].translated) {
            if (read_index == ROB.head)
                break;
            else {
                ////cout << "read_index: " << read_index << " ROB.head: " << ROB.head << " ROB.tail: " << ROB.tail << endl;
                assert(0);
            }
        }
#endif

        PACKET trace_packet;
        trace_packet.instruction = 1;
        trace_packet.tlb_access = 1;
        trace_packet.fill_level = FILL_L1;
        trace_packet.cpu = cpu;
        trace_packet.address = ROB.entry[read_index].ip >> LOG2_PAGE_SIZE;
        if (knob_cloudsuite)
            trace_packet.address = ((ROB.entry[read_index].ip >> LOG2_PAGE_SIZE) << 9) | ( 256 + ROB.entry[read_index].asid[0]);
        else
            trace_packet.address = ROB.entry[read_index].ip >> LOG2_PAGE_SIZE;
        trace_packet.full_addr = ROB.entry[read_index].ip;
        trace_packet.instr_id = ROB.entry[read_index].instr_id;
        trace_packet.rob_index = read_index;
        trace_packet.producer = 0; // TODO: check if this guy gets used or not
        trace_packet.ip = ROB.entry[read_index].ip;
        trace_packet.type = LOAD; 
        trace_packet.asid[0] = ROB.entry[read_index].asid[0];
        trace_packet.asid[1] = ROB.entry[read_index].asid[1];
        trace_packet.event_cycle = current_core_cycle[cpu];

        int rq_index = ITLB.add_rq(&trace_packet);

        if (rq_index == -2)
            break;
        else {
            
            //if (rq_index >= 0) {
            //    uint32_t producer = ITLB.RQ.entry[rq_index].rob_index;
            //    ROB.entry[read_index].fetch_producer = producer;
            //    ROB.entry[read_index].is_consumer = 1;
	    //	
            //    ROB.entry[producer].memory_instrs_depend_on_me[read_index] = 1;
            //    ROB.entry[producer].is_producer = 1; // producer for fetch
            //}
            

            ROB.last_read = read_index;
            read_index++;
            if (read_index == ROB.SIZE)
                read_index = 0;
        }
    }
    */

/*    uint32_t fetch_index = (ROB.last_fetch == (ROB.SIZE-1)) ? 0 : (ROB.last_fetch + 1);
	////DP ( if (warmup_complete[cpu]) {	
              //  ////cout << " and fetch instr id:"<< ROB.entry[fetch_index].instr_id << endl; });
    for (uint32_t i=0; i<FETCH_WIDTH; i++) {

        // fetch is in-order so it should be break
	// @Vishal: TODO see if we can make out of order fetch as schedule_instruction is in order.
        // @Vishal: Removed transalation condition
	//if ((ROB.entry[fetch_index].translated != COMPLETED) || (ROB.entry[fetch_index].event_cycle > current_core_cycle[cpu]))
	 if(ROB.entry[fetch_index].event_cycle > current_core_cycle[cpu]) 
            break;
	
        //@Vishal: check if empty ROB slot
	if(ROB.entry[fetch_index].ip == 0)
		break;

        // sanity check
        if (ROB.entry[fetch_index].fetched) {
            if (fetch_index == ROB.head)
                break;
            else {
                ////cout << "fetch_index: " << fetch_index << " ROB.head: " << ROB.head << " ROB.tail: " << ROB.tail << endl;
                assert(0);
            }
        }

        // add it to L1I
        PACKET fetch_packet;
        fetch_packet.instruction = 1;
        fetch_packet.fill_level = FILL_L1;
        fetch_packet.cpu = cpu;
	
	//@Vishal: VIPT virtual address will be sent to L1I instead of physical address
        fetch_packet.address = ROB.entry[fetch_index].instruction_pa >> 6;
        fetch_packet.instruction_pa = ROB.entry[fetch_index].instruction_pa;
        fetch_packet.full_addr = ROB.entry[fetch_index].instruction_pa; //Neelu: Removed comment ending.

	fetch_packet.address = ROB.entry[fetch_index].ip >> 6;
        fetch_packet.full_addr = ROB.entry[fetch_index].ip;	
	fetch_packet.full_virtual_address =  ROB.entry[fetch_index].ip;

        fetch_packet.instr_id = ROB.entry[fetch_index].instr_id;
        fetch_packet.rob_index = fetch_index;
        fetch_packet.producer = 0;
        fetch_packet.ip = ROB.entry[fetch_index].ip;
        fetch_packet.type = LOAD; 
        fetch_packet.asid[0] = ROB.entry[fetch_index].asid[0];
        fetch_packet.asid[1] = ROB.entry[fetch_index].asid[1];
        fetch_packet.event_cycle = current_core_cycle[cpu];

        int rq_index = L1I.add_rq(&fetch_packet);

        if (rq_index == -2)
            break;
        else {
            if (rq_index >= 0) {
                uint32_t producer = L1I.RQ.entry[rq_index].rob_index;
                ROB.entry[fetch_index].fetch_producer = producer;
                ROB.entry[fetch_index].is_consumer = 1;

                ROB.entry[producer].memory_instrs_depend_on_me[fetch_index] = 1;
                ROB.entry[producer].is_producer = 1;
            }
            //Neelu: Removed comment ending.

            ROB.entry[fetch_index].fetched = INFLIGHT;
            ROB.last_fetch = fetch_index;
            fetch_index++;
            if (fetch_index == ROB.SIZE)
                fetch_index = 0;
        }
    }*/
	//Neelu: Commented until here. 
	
    //DP ( if (warmup_complete[cpu]) {
      //          ////cout << "Exit fetch_instruction() with read instr id:"<< ROB.entry[read_index].instr_id << "fetch instr_id:"<< ROB.entry[fetch_index].instr_id<< endl; });

//}	ending of fetch_instruction


//@Vasudha:SMT: Store instr_id which comprises of thread-ID appended to it. 
void O3_CPU::fill_btb(uint64_t trigger, uint64_t target, uint64_t instr_id)
{
	uint32_t btb_set = BTB.get_set(trigger);
	int btb_way = BTB.get_way(trigger, btb_set, instr_id);
	
	if(btb_way == BTB_WAY)
	{
		btb_way = (BTB.*BTB.find_victim)(cpu, 0, btb_set, BTB.block[btb_set], trigger, trigger, 0);
		(BTB.*BTB.update_replacement_state)(cpu, btb_set, btb_way, trigger, trigger, BTB.block[btb_set][btb_way].address, 0, 0);
		BLOCK &entry = BTB.block[btb_set][btb_way];
		if(entry.valid == 0)
			entry.valid = 1;
		entry.dirty = 0;
		entry.tag = trigger;
		entry.address = trigger;
		entry.full_addr = trigger;
		entry.data = target;
		entry.ip = trigger;
		entry.cpu = cpu;
		entry.instr_id = instr_id;
	}
	else
	{
		BTB.block[btb_set][btb_way].data = target;
	}			
}



void O3_CPU::decode_and_dispatch()
{
	  // dispatch DECODE_WIDTH instructions that have decoded into the ROB
	  uint32_t count_dispatches = 0;
	  for(uint32_t i=0; i<DECODE_BUFFER.SIZE; i++)
	  {
		  if(DECODE_BUFFER.entry[DECODE_BUFFER.head].ip == 0)
		  {
			  break;
		  }

		  int thread_num = ((1 << LOG2_THREADS) - 1) & DECODE_BUFFER.entry[DECODE_BUFFER.head].instr_id;
		  //if(((!warmup_complete[cpu]) && (ROB.occupancy < ROB.SIZE)) || ((DECODE_BUFFER.entry[DECODE_BUFFER.head].event_cycle != 0) && (DECODE_BUFFER.entry[DECODE_BUFFER.head].event_cycle < current_core_cycle[cpu]) && (ROB.occupancy < ROB.SIZE)))
		  if ((!warmup_complete[cpu][thread_num]) || (DECODE_BUFFER.entry[DECODE_BUFFER.head].event_cycle != 0 && DECODE_BUFFER.entry[DECODE_BUFFER.head].event_cycle < current_core_cycle[cpu]))
		  {
			  //@Vasudha:SMT: Check the occupancy of one partition of ROB
			  uint32_t size_of_partition = ROB.SIZE/thread;
			  if (ROB.occupancy[thread_num] < size_of_partition)
			  {
				if(DECODE_BUFFER.entry[DECODE_BUFFER.head].btb_miss == 1 && DECODE_BUFFER.entry[DECODE_BUFFER.head].branch_mispredicted == 0)
				{
					uint8_t branch_type = DECODE_BUFFER.entry[DECODE_BUFFER.head].branch_type;
					if(branch_type == BRANCH_DIRECT_JUMP || branch_type == BRANCH_DIRECT_CALL || (branch_type == BRANCH_CONDITIONAL))
					{
						if(warmup_complete[cpu][thread_num])
						{
							fetch_resume_cycle[thread_num] = current_core_cycle[cpu] + 1; //Resume fetch from next cycle.
						}
						DECODE_BUFFER.entry[DECODE_BUFFER.head].btb_miss = 0;
						//if(branch_type == BRANCH_CONDITIONAL)
						//assert(DECODE_BUFFER.entry[DECODE_BUFFER.head].ip + 4 != DECODE_BUFFER.entry[DECODE_BUFFER.head].branch_target);		
						fill_btb(DECODE_BUFFER.entry[DECODE_BUFFER.head].ip, DECODE_BUFFER.entry[DECODE_BUFFER.head].branch_target, DECODE_BUFFER.entry[DECODE_BUFFER.head].instr_id);		
					}
				}


			  	// move this instruction to the ROB if there's space
				uint32_t rob_index = add_to_rob(&DECODE_BUFFER.entry[DECODE_BUFFER.head]);
				ROB.entry[rob_index].event_cycle = current_core_cycle[cpu];

				ooo_model_instr empty_entry;
				DECODE_BUFFER.entry[DECODE_BUFFER.head] = empty_entry;

				DECODE_BUFFER.head++;
				if(DECODE_BUFFER.head >= DECODE_BUFFER.SIZE)
				{
					DECODE_BUFFER.head = 0;
				}
				DECODE_BUFFER.occupancy--;

				count_dispatches++;
				if(count_dispatches >= DECODE_WIDTH)
				{
					break;
				}
		  	}
	  	  }
		  else
		  {
			  break;
		  }
	  }

	  // make new instructions pay decode penalty if they miss in the decoded instruction cache
	uint32_t decode_index = DECODE_BUFFER.head;
      	uint32_t count_decodes = 0;
      	for(uint32_t i=0; i<DECODE_BUFFER.SIZE; i++)
	{
		if(DECODE_BUFFER.entry[DECODE_BUFFER.head].ip == 0)
		{
			break;
		}

  		if(DECODE_BUFFER.entry[decode_index].event_cycle == 0)
  		{
			// apply decode latency
	  		DECODE_BUFFER.entry[decode_index].event_cycle = current_core_cycle[cpu] + DECODE_LATENCY;
			count_decodes++;
		}

		if(decode_index == DECODE_BUFFER.tail)
		{
			break;
		}
		decode_index++;
		if(decode_index >= DECODE_BUFFER.SIZE)
		{
			decode_index = 0;
		}

		if(count_decodes >= DECODE_WIDTH)
		{
			break;
		}
	}
}//Ending of decode_and_dispatch()

int O3_CPU::prefetch_code_line(uint64_t pf_v_addr)
{
	  if(pf_v_addr == 0)
	  {
		  cerr << "Cannot prefetch code line 0x0 !!!" << endl;
		  assert(0);
	  }

	  L1I.pf_requested++;

	  if (L1I.PQ.occupancy < L1I.PQ.SIZE)
	  {
		  //Neelu: Cannot magically translate prefetches, need to get realistic and access the TLBs. 
		  // magically translate prefetches
		/*  uint64_t pf_pa = (va_to_pa(cpu, 0, pf_v_addr, pf_v_addr>>LOG2_PAGE_SIZE, 1) & (~((1 << LOG2_PAGE_SIZE) - 1))) | (pf_v_addr & ((1 << LOG2_PAGE_SIZE) - 1)); */

		  PACKET pf_packet;
		  pf_packet.instruction = 1; // this is a code prefetch
		  pf_packet.is_data = 0;
		  pf_packet.fill_level = FILL_L1;
		  pf_packet.fill_l1i = 1;
		  pf_packet.pf_origin_level = FILL_L1;
		  pf_packet.cpu = cpu;		 
		  //Neelu: assigning virtual addresses.  
		  //pf_packet.address = pf_pa >> LOG2_BLOCK_SIZE;
		  //pf_packet.full_addr = pf_pa;

		  pf_packet.address = pf_v_addr >> LOG2_BLOCK_SIZE;
		  pf_packet.full_addr = pf_v_addr;

		  //Neelu: Marking translated = 0
		  pf_packet.translated = 0;
		  pf_packet.full_physical_address = 0;

		  pf_packet.ip = pf_v_addr;
		  pf_packet.type = PREFETCH;
		  pf_packet.event_cycle = current_core_cycle[cpu];

		  L1I.add_pq(&pf_packet);
		  L1I.pf_issued++;
		
		  return 1;
	  }

	  return 0;
}



// TODO: When should we update ROB.schedule_event_cycle?
// I. Instruction is fetchd
// II. Instruction is completed
// III. Instruction is retired
void O3_CPU::schedule_instruction()
{
	//@Vasudha:SMT: Schedule instructions from every thread's ROB (logically partitioned)
	//for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
	//{
	uint32_t thread_num = last_scheduled_thread + 1;
        
	//@Rahul
 	uint32_t start_index[thread];

	if (thread_num == ooo_cpu[cpu].thread)
		thread_num = 0;
	uint32_t cannot_schedule[thread], total_cannot_schedule = 0;
	for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
	{
		cannot_schedule[thread_num] = 0;
		total_cannot_schedule += cannot_schedule[thread_num];

		//@Rahul
		start_index[thread_num] = ROB.head[thread_num];
	}
    	num_searched = 0;

	while (total_cannot_schedule < thread)
	{
		if (ROB.head[thread_num] == ROB.tail[thread_num] && ROB.occupancy[thread_num] == 0)
		{
			cannot_schedule[thread_num] = 1;
			total_cannot_schedule = 0;
			for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
				total_cannot_schedule += cannot_schedule[thread_num];

			thread_num++;
			if (thread_num ==  thread)
				thread_num = 0;
			continue;
		}
    		// execution is out-of-order but we have an in-order scheduling algorithm to detect all RAW dependencies
		uint32_t limit = ROB.tail[thread_num];
    		//num_searched = 0;
		
		//@Rahul
    		//if (ROB.head[thread_num] < limit) {
                if (start_index[thread_num] < limit) {
		
        	//	for (uint32_t i=ROB.head[thread_num]; i<limit; i++) { 
		        for (uint32_t i=start_index[thread_num]; i<limit; i++) {
                                start_index[thread_num] = i+1;

            			if (cannot_schedule[thread_num] == 1)
					break;	
				if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
				{
					cannot_schedule[thread_num] = 1;
					break;
				}

				if (i == limit-1)
                                        cannot_schedule[thread_num] = 1;
				num_searched++;
				
				if (ROB.entry[i].scheduled == 0)
				{
					do_scheduling(i);
				//	num_searched++;
	                		break;
				}
				//if (i == limit-1)
				//	cannot_schedule[thread_num] = 1;
				//num_searched++;
				
        		}
    		}
    		else 
		{
        		//for (uint32_t i=ROB.head[thread_num]; i<=ROB.partition[thread_num]; i++) {
                        for (uint32_t i=start_index[thread_num]; i<=ROB.partition[thread_num]; i++) {
				if(i == ROB.partition[thread_num]){
                                        start_index[thread_num] = (thread_num==0 ? 0 : ROB.partition[thread_num-1] + 1);
                                }else{
                                        start_index[thread_num] = i+1;
                                }

            			if (cannot_schedule[thread_num] == 1)
					break;
				if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
				{
					cannot_schedule[thread_num] = 1;
					break;
				}

				if (i == ROB.partition[thread_num])
                                        cannot_schedule[thread_num] = 1;
			       	num_searched++;

            			if (ROB.entry[i].scheduled == 0)
				{
					do_scheduling(i);
            			//	num_searched++;
					break;
				}
				//if (i == ROB.partition[thread_num])
				//	cannot_schedule[thread_num] = 1;
				//num_searched++;
				
        		}
                        for (uint32_t i=start_index[thread_num]; i<limit; i++) {		
        		//for (uint32_t i=(thread_num == 0 ? 0 : ROB.partition[thread_num-1] + 1); i<limit; i++) { 
        			start_index[thread_num] = i+1;

				if (cannot_schedule[thread_num] == 1)
					break;
				if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
				{
					cannot_schedule[thread_num] = 1;
					break;
				}

				if (i == limit-1)
                                        cannot_schedule[thread_num] = 1;
				num_searched++;

            			if (ROB.entry[i].scheduled == 0)
				{
					do_scheduling(i);
				//	num_searched++;
					break;
				}
				//if (i == limit - 1)
				//	cannot_schedule[thread_num] = 1;
            			//num_searched++;
				
        		}
    		}

		thread_num++;
		if (thread_num == thread)
			thread_num = 0;

		total_cannot_schedule = 0;
		for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
			total_cannot_schedule += cannot_schedule[thread_num];
	
	}
	last_scheduled_thread = thread_num;

}


void O3_CPU::do_scheduling(uint32_t rob_index)
{
    ROB.entry[rob_index].reg_ready = 1; // reg_ready will be reset to 0 if there is RAW dependency 

    reg_dependency(rob_index);
    int thread_num = ((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id;
 //  ROB.next_schedule[thread_num] = (rob_index == ROB.partition[thread_num] + 1) ? (thread_num == 0 ? 0 : ROB.partition[thread_num-1]+1) : (rob_index + 1);
     ROB.next_schedule[thread_num] = (rob_index == ROB.partition[thread_num]) ? (thread_num == 0 ? 0 : ROB.partition[thread_num-1]+1) : (rob_index + 1);


    //cout << "next-schedule = " << ROB.next_schedule[thread_num] << endl;
    if (ROB.entry[rob_index].is_memory)
        ROB.entry[rob_index].scheduled = INFLIGHT;
    else {
        ROB.entry[rob_index].scheduled = COMPLETED;
        // ADD LATENCY
        if (ROB.entry[rob_index].event_cycle < current_core_cycle[cpu])
            ROB.entry[rob_index].event_cycle = current_core_cycle[cpu] + SCHEDULING_LATENCY;
        else
            ROB.entry[rob_index].event_cycle += SCHEDULING_LATENCY;
	////if(all_warmup_complete > thread)
	////cout << "Cycle-" << current_core_cycle[cpu] <<" Scheduling complete(Non-mem) - thread: " << thread_num << " instr_id: " << ROB.entry[rob_index].instr_id << " ROB-index: " << rob_index << endl;
        if (ROB.entry[rob_index].reg_ready) {

#ifdef SANITY_CHECK
            if (RTE1[RTE1_tail] < ROB_SIZE)
                assert(0);
#endif
            // remember this rob_index in the Ready-To-Execute array 1
            RTE1[RTE1_tail] = rob_index;

            //DP (if (warmup_complete[cpu][thread_num]) {
            //cout << "[RTE1] " << __func__ << " instr_id: " << (ROB.entry[rob_index].instr_id >>  LOG2_THREADS) << " rob_index: " << rob_index << " is added to RTE1";
            //cout << " head: " << RTE1_head << " tail: " << RTE1_tail << endl; }); 

            RTE1_tail++;
            if (RTE1_tail == ROB_SIZE)
                RTE1_tail = 0;
        }
    }
}

void O3_CPU::reg_dependency(uint32_t rob_index)
{
    // print out source/destination registers
    /*DP (if (warmup_complete[cpu]) {
    for (uint32_t i=0; i<NUM_INSTR_SOURCES; i++) {
        if (ROB.entry[rob_index].source_registers[i]) {
            ////cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[rob_index].instr_id << " is_memory: " << +ROB.entry[rob_index].is_memory;
            ////cout << " load  reg_index: " << +ROB.entry[rob_index].source_registers[i] << endl;
        }
    }
    for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        if (ROB.entry[rob_index].destination_registers[i]) {
            ////cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[rob_index].instr_id << " is_memory: " << +ROB.entry[rob_index].is_memory;
            ////cout << " store reg_index: " << +ROB.entry[rob_index].destination_registers[i] << endl;
        }
    } }); */

    // check RAW dependency
    int prior = (int)rob_index - 1;
    uint8_t thread_num = ((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id;

    //@Vasudha:SMT: check dependency in the thread's ROB partition only
    if (prior < 0 || ( prior >= 0 && ((((1 << LOG2_THREADS) - 1) & ROB.entry[prior].instr_id) != thread_num)))
    {
	prior = ROB.partition[thread_num] ;	
    }

    
    if (rob_index != ROB.head[thread_num]) {
        if ((int)ROB.head[thread_num] <= prior) {
            for (int i=prior; i>=(int)ROB.head[thread_num]; i--) if (ROB.entry[i].executed != COMPLETED) {
		for (uint32_t j=0; j<NUM_INSTR_SOURCES; j++) {
			if (ROB.entry[rob_index].source_registers[j] && (ROB.entry[rob_index].reg_RAW_checked[j] == 0))
				reg_RAW_dependency(i, rob_index, j);
		}
	    }
        } else {
		/*uint32_t limit;
		if (thread_num == 0)
			limit = 0;
		else
			limit = ROB.partition[thread_num-1]+1;*/
            for (int i=prior; i>=(int)(thread_num == 0 ? 0 : ROB.partition[thread_num-1] + 1); i--) if (ROB.entry[i].executed != COMPLETED) {
		for (uint32_t j=0; j<NUM_INSTR_SOURCES; j++) {
			if (ROB.entry[rob_index].source_registers[j] && (ROB.entry[rob_index].reg_RAW_checked[j] == 0))
				reg_RAW_dependency(i, rob_index, j);
		}
	    }
            for (int i=ROB.partition[thread_num]; i>=(int)ROB.head[thread_num]; i--) if (ROB.entry[i].executed != COMPLETED) {
		for (uint32_t j=0; j<NUM_INSTR_SOURCES; j++) {
			if (ROB.entry[rob_index].source_registers[j] && (ROB.entry[rob_index].reg_RAW_checked[j] == 0))
				reg_RAW_dependency(i, rob_index, j);
		}
	    }
        }
    }
}

void O3_CPU::reg_RAW_dependency(uint32_t prior, uint32_t current, uint32_t source_index)
{
	//@Vasudha:SMT: Both rob_index (prior, current) should belong to same partition
	if ((((1 << LOG2_THREADS) - 1) & ROB.entry[prior].instr_id) != (((1 << LOG2_THREADS) - 1) & ROB.entry[current].instr_id))
	{
		cerr << "ERROR: register dependency cannot be checked among different threads\n" ;
		assert(0);
	}
    for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        if (ROB.entry[prior].destination_registers[i] == 0)
            continue;

        if (ROB.entry[prior].destination_registers[i] == ROB.entry[current].source_registers[source_index]) {

            // we need to mark this dependency in the ROB since the producer might not be added in the store queue yet
            ROB.entry[prior].registers_instrs_depend_on_me.insert (current);   // this load cannot be executed until the prior store gets executed
            ROB.entry[prior].registers_index_depend_on_me[source_index].insert (current);   // this load cannot be executed until the prior store gets executed
            ROB.entry[prior].reg_RAW_producer = 1;

            ROB.entry[current].reg_ready = 0;
            ROB.entry[current].producer_id = ROB.entry[prior].instr_id; 
            ROB.entry[current].num_reg_dependent++;
            ROB.entry[current].reg_RAW_checked[source_index] = 1;

            //DP (if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[current].instr_id]) {
            //cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[current].instr_id << " is_memory: " << +ROB.entry[current].is_memory;
            //cout << " RAW reg_index: " << +ROB.entry[current].source_registers[source_index];
            //cout << " producer_id: " << ROB.entry[prior].instr_id << endl; });

            return;
        }
    }
}

void O3_CPU::execute_instruction()
{
	int count_thread = 0;
	for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
		if (ROB.occupancy[thread_num])
			count_thread++;
	
	//cout << "Enter execute_insr() with " << count_thread << "  thread(s) RTE[RTE1.head] = " << RTE1[RTE1_head] <<  endl ;
	if (count_thread == 0)
		return;

    // out-of-order execution for non-memory instructions
    // memory instructions are handled by memory_instruction()
    uint32_t exec_issued = 0, num_iteration = 0;

    while (exec_issued < EXEC_WIDTH) {
        if (RTE0[RTE0_head] < ROB_SIZE) {
    		uint32_t exec_index = RTE0[RTE0_head];
            if (ROB.entry[exec_index].event_cycle <= current_core_cycle[cpu]) {
                do_execution(exec_index);

                RTE0[RTE0_head] = ROB_SIZE;
                RTE0_head++;
                if (RTE0_head == ROB_SIZE)
                    RTE0_head = 0;
                exec_issued++;
            }
        }
        else {
            //DP (if (warmup_complete[cpu][((1<<LOG2_THREADS) - 1) & ROB.entry[RTE0[RTE0_head]].instr_id]) {
            	//cout << "[RTE0] is empty head: " << RTE0_head << " tail: " << RTE0_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (ROB_SIZE-1))
            break;
    }

    
    num_iteration = 0;
    while (exec_issued < EXEC_WIDTH) {
        if (RTE1[RTE1_head] < ROB_SIZE) {
    	uint32_t exec_index = RTE1[RTE1_head];
            if (ROB.entry[exec_index].event_cycle <= current_core_cycle[cpu]) {
                do_execution(exec_index);

                RTE1[RTE1_head] = ROB_SIZE;
                RTE1_head++;
                if (RTE1_head == ROB_SIZE)
                    RTE1_head = 0;
                exec_issued++;
            }
        }
        else {
            //DP (if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[RTE1[RTE1_head]].instr_id]) {
            //cout << "[RTE1] is empty head: " << RTE1_head << " tail: " << RTE1_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (ROB_SIZE-1))
            break;
    }
}

void O3_CPU::do_execution(uint32_t rob_index)
{
    //if (ROB.entry[rob_index].reg_ready && (ROB.entry[rob_index].scheduled == COMPLETED) && (ROB.entry[rob_index].event_cycle <= current_core_cycle[cpu])) {

        ROB.entry[rob_index].executed = INFLIGHT;

        // ADD LATENCY
        if (ROB.entry[rob_index].event_cycle < current_core_cycle[cpu])
            ROB.entry[rob_index].event_cycle = current_core_cycle[cpu] + EXEC_LATENCY;
        else
            ROB.entry[rob_index].event_cycle += EXEC_LATENCY;

        inflight_reg_executions++;

        //DP (if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1 ) & ROB.entry[rob_index].instr_id]) {
        //cout << "[ROB] " << __func__ << " non-memory instr_id: " << ROB.entry[rob_index].instr_id; 
        //cout << " event_cycle: " << ROB.entry[rob_index].event_cycle << endl;});
    //}
}

void O3_CPU::schedule_memory_instruction()
{
    	//DP ( if (warmup_complete[cpu]) {	
               //cout << "Entered schedule_memory_instruction() with  next_schedule instr_id: "<< (ROB.entry[limit].instr_id >> LOG2_THREADS) << endl;// });
	int count_thread = 0;

	//@Rahul
        uint32_t start_index[thread];

	for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
		if (ROB.occupancy[thread_num])
			count_thread++;

	if (count_thread == 0)
		return;

	uint32_t thread_num = last_scheduled_thread + 1;
	if (thread_num == ooo_cpu[cpu].thread)
		thread_num = 0;
	uint32_t cannot_schedule[thread], total_cannot_schedule = 0;
	for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
	{
		cannot_schedule[thread_num] = 0;
		total_cannot_schedule += cannot_schedule[thread_num];

                //@Rahul
                start_index[thread_num] = ROB.head[thread_num];
	}
    	num_searched = 0;

	while (total_cannot_schedule < thread)
	{
		if (ROB.head[thread_num] == ROB.tail[thread_num] && ROB.occupancy[thread_num] == 0)
		{
			cannot_schedule[thread_num] = 1;
			total_cannot_schedule = 0;
			for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
				total_cannot_schedule += cannot_schedule[thread_num];

			thread_num++;
			if (thread_num ==  thread)
				thread_num = 0;
			continue;
		}
    		// execution is out-of-order but we have an in-order scheduling algorithm to detect all RAW dependencies
    		uint32_t limit = ROB.next_schedule[thread_num];
    		//num_searched = 0;
		
		//@Rahul
    		//if (ROB.head[thread_num] < limit) {
                if (start_index[thread_num] < limit) {

        		//for (uint32_t i=ROB.head[thread_num]; i<limit; i++) {
                        for (uint32_t i=start_index[thread_num]; i<limit; i++) {
                                start_index[thread_num] = i+1;
			
				if (cannot_schedule[thread_num] == 1)
					break;
            			if (ROB.entry[i].is_memory == 0)
				{
					if (i == limit - 1)
						cannot_schedule[thread_num] = 1;	
                			continue;
				}

            			if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
                		{
					cannot_schedule[thread_num] = 1;
					break;
				}

            			if (ROB.entry[i].is_memory && ROB.entry[i].reg_ready && (ROB.entry[i].scheduled == INFLIGHT))
				{
					do_memory_scheduling(i);
					//num_searched++;
					break;
				}
				if (i == limit - 1)
					cannot_schedule[thread_num] = 1;
        			//num_searched++;
			}
    		}
    		else {
        		//for (uint32_t i=ROB.head[thread_num]; i<=ROB.partition[thread_num]; i++) {
                        for (uint32_t i=start_index[thread_num]; i<=ROB.partition[thread_num]; i++) {

				if(i == ROB.partition[thread_num]){
                                        start_index[thread_num] = (thread_num==0 ? 0 : ROB.partition[thread_num-1] + 1);
                                }else{
                                        start_index[thread_num] = i+1;
                                }

				if (cannot_schedule[thread_num] == 1)
					break;
            			if (ROB.entry[i].is_memory == 0)
				{
					if (i == ROB.partition[thread_num])
						cannot_schedule[thread_num] = 1;
					continue;
				}

            			if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
				{
					cannot_schedule[thread_num] = 1;
					break;
				}

            			if (ROB.entry[i].is_memory && ROB.entry[i].reg_ready && (ROB.entry[i].scheduled == INFLIGHT))
				{
					do_memory_scheduling(i);
					//num_searched++;
					break;
				}
        			if (i == ROB.partition[thread_num])
					cannot_schedule[thread_num] = 1;
				//num_searched++;
			}
        		//for (uint32_t i=(thread_num==0 ? 0 : ROB.partition[thread_num-1] + 1); i<limit; i++) {
                        for (uint32_t i=start_index[thread_num]; i<limit; i++) {
                                start_index[thread_num] = i+1;		
			
				if (cannot_schedule[thread_num] == 1)
					break;
           	 		if (ROB.entry[i].is_memory == 0)
				{
					if (i == limit - 1)
						cannot_schedule[thread_num] = 1;
					continue;
				}

            			if ((ROB.entry[i].fetched != COMPLETED) || (ROB.entry[i].event_cycle > current_core_cycle[cpu]) || (num_searched >= SCHEDULER_SIZE))
                			break;

            			if (ROB.entry[i].is_memory && ROB.entry[i].reg_ready && (ROB.entry[i].scheduled == INFLIGHT))
				{
					do_memory_scheduling(i);
					//num_searched++;
					break;
				}
				if( i == limit - 1)
					cannot_schedule[thread_num] = 1;
        			//num_searched++;
			}
    		}

		thread_num++;
		if (thread_num == thread)
			thread_num = 0;

		total_cannot_schedule = 0;
		for (uint32_t thread_num = 0; thread_num < thread; thread_num++)
			total_cannot_schedule += cannot_schedule[thread_num]; 	
	}
	last_scheduled_thread = thread_num;
   // //DP ( if (warmup_complete[cpu]) {	
     //           ////cout << "Exit execute_instruction() "<< endl; });
}

void O3_CPU::execute_memory_instruction()
{
	////DP ( if (warmup_complete[cpu]) {	//*******************************************************************************************
          //      ////cout << "Entered execute_memory_instruction() "<< endl; });
    operate_lsq();
    operate_cache();
    ////DP ( if (warmup_complete[cpu]) {	//*******************************************************************************************
      //          ////cout << "Exit execute_memory_instruction() "<< endl; });
}

void O3_CPU::do_memory_scheduling(uint32_t rob_index)
{
	//cout << "do_memory_Scheduling for rob_index - " << rob_index << endl;
    uint32_t not_available = check_and_add_lsq(rob_index);
    if (not_available == 0) {
        ROB.entry[rob_index].scheduled = COMPLETED;
        //cout << "ROB rob_index = " << rob_index << " memory scheduling completed " << endl;
	if (ROB.entry[rob_index].executed == 0) // it could be already set to COMPLETED due to store-to-load forwarding
            ROB.entry[rob_index].executed  = INFLIGHT;

	////if(all_warmup_complete > thread)
	////cout << "Cycle- " << current_core_cycle[cpu] << " Scheduling complete(Mem) - thread: " << (((1<<LOG2_THREADS)-1) & ROB.entry[rob_index].instr_id) << " instr_id: " << ROB.entry[rob_index].instr_id << " ROB-index: " << rob_index << endl;
        
	//DP (if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id]) {
        //cout << "[ROB] " << __func__ << " instr_id: " << (ROB.entry[rob_index].instr_id  >> LOG2_THREADS)<< " rob_index: " << rob_index;
        //cout << " scheduled all num_mem_ops: " << ROB.entry[rob_index].num_mem_ops << endl; });
    }

    num_searched++;
}

uint32_t O3_CPU::check_and_add_lsq(uint32_t rob_index) 
{
    uint32_t num_mem_ops = 0, num_added = 0;

    // load
    for (uint32_t i=0; i<NUM_INSTR_SOURCES; i++) {
        if (ROB.entry[rob_index].source_memory[i]) {
            num_mem_ops++;
            if (ROB.entry[rob_index].source_added[i])
                num_added++;
            else if (LQ.occupancy < LQ.SIZE) {
                add_load_queue(rob_index, i);
                num_added++;
            }
            else {
               // DP(if(warmup_complete[cpu]) {
               // cout << "[LQ] " << __func__ << " instr_id: " << (ROB.entry[rob_index].instr_id >> LOG2_THREADS);
               // cout << " cannot be added in the load queue occupancy: " << LQ.occupancy << " cycle: " << current_core_cycle[cpu] << endl; });
            }
        }
    }

    // store
    for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        if (ROB.entry[rob_index].destination_memory[i]) {
            num_mem_ops++;
            if (ROB.entry[rob_index].destination_added[i])
                num_added++;
            else if (SQ.occupancy < SQ.SIZE) {
                if (STA[STA_head[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id]] == (ROB.entry[rob_index].instr_id >> LOG2_THREADS)) {
                    add_store_queue(rob_index, i);
                    num_added++;
                }
                //add_store_queue(rob_index, i);
                //num_added++;
            }
            else {
                //DP(if(warmup_complete[cpu]) {
                //cout << "[SQ] " << __func__ << " instr_id: " << (ROB.entry[rob_index].instr_id >> LOG2_THREADS);
                //cout << " cannot be added in the store queue occupancy: " << SQ.occupancy << " cycle: " << current_core_cycle[cpu] << endl; });
            }
        }
    }

    if (num_added == num_mem_ops)
        return 0;

    uint32_t not_available = num_mem_ops - num_added;
    if (not_available > num_mem_ops) {
        cerr << "instr_id: " << ROB.entry[rob_index].instr_id << endl;
        assert(0);
    }

    return not_available;
}

void O3_CPU::add_load_queue(uint32_t rob_index, uint32_t data_index)
{
    // search for an empty slot 
    uint32_t lq_index = LQ.SIZE;
    for (uint32_t i=0; i<LQ.SIZE; i++) {
        if (LQ.entry[i].virtual_address == 0) {
            lq_index = i;
            break;
        }
    }

    // sanity check
    if (lq_index == LQ.SIZE) {
        cerr << "instr_id: " << ROB.entry[rob_index].instr_id << " no empty slot in the load queue!!!" << endl;
        assert(0);
    }

    //cout << "add_load_queue - ROB index = " << rob_index << " instr_id = " << (ROB.entry[rob_index].instr_id >> LOG2_THREADS) << endl; 
    sim_load_gen[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id]++;

    // add it to the load queue
    ROB.entry[rob_index].lq_index[data_index] = lq_index;
    LQ.entry[lq_index].instr_id = ROB.entry[rob_index].instr_id;
    LQ.entry[lq_index].virtual_address = ROB.entry[rob_index].source_memory[data_index];
    LQ.entry[lq_index].ip = ROB.entry[rob_index].ip;
    LQ.entry[lq_index].data_index = data_index;
    LQ.entry[lq_index].rob_index = rob_index;
    LQ.entry[lq_index].asid[0] = ROB.entry[rob_index].asid[0];
    LQ.entry[lq_index].asid[1] = ROB.entry[rob_index].asid[1];
    LQ.entry[lq_index].event_cycle = current_core_cycle[cpu] + SCHEDULING_LATENCY;
    LQ.occupancy++;

    // check RAW dependency
    int prior = rob_index - 1;
    uint8_t thread_num = ((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id;
 
    //@Vasudha:SMT: check RAW dependency in the thread'sROB partition only
    if (prior < 0 || (prior >= 0 && ((((1 << LOG2_THREADS) - 1) & ROB.entry[prior].instr_id) != thread_num)))
    {
	prior = ROB.partition[thread_num];
    }

    if (rob_index != ROB.head[thread_num]) {
        if (ROB.head[thread_num] <= prior) {
            for (int i=prior; i>=(int)ROB.head[thread_num]; i--) {
                if (LQ.entry[lq_index].producer_id != UINT64_MAX)
                    break;
                mem_RAW_dependency(i, rob_index, data_index, lq_index);
            }
        }
        else {
            for (int i=prior; i>=(int)(thread_num == 0 ? 0 : ROB.partition[thread_num - 1] + 1); i--) {
                if (LQ.entry[lq_index].producer_id != UINT64_MAX)
                    break;
                mem_RAW_dependency(i, rob_index, data_index, lq_index);
            }
            for (int i=ROB.partition[thread_num]; i>=(int)ROB.head[thread_num]; i--) { 
                if (LQ.entry[lq_index].producer_id != UINT64_MAX)
                    break;

                    mem_RAW_dependency(i, rob_index, data_index, lq_index);
            }
        }
    }

    // check
    // 1) if store-to-load forwarding is possible
    // 2) if there is WAR that are not correctly executed
    uint32_t forwarding_index = SQ.SIZE;
    for (uint32_t i=0; i<SQ.SIZE; i++) {

        // skip empty slot
        if (SQ.entry[i].virtual_address == 0)
            continue;

	//@Vasudha:SMT: RAW dependency should be checked for instructions from same thread
	if ((((1 << LOG2_THREADS) - 1) & SQ.entry[i].instr_id) == (((1 << LOG2_THREADS) - 1) & LQ.entry[lq_index].instr_id)) {
        // forwarding should be done by the SQ entry that holds the same producer_id from RAW dependency check
        if (SQ.entry[i].virtual_address == LQ.entry[lq_index].virtual_address) { // store-to-load forwarding check

            // forwarding store is in the SQ
            if ((rob_index != ROB.head[((1 << LOG2_THREADS) - 1) & SQ.entry[i].instr_id]) && (LQ.entry[lq_index].producer_id == SQ.entry[i].instr_id)) { // RAW
                forwarding_index = i;
                break; // should be break
            }

            if ((LQ.entry[lq_index].producer_id == UINT64_MAX) && (LQ.entry[lq_index].instr_id <= SQ.entry[i].instr_id)) { // WAR 
                // a load is about to be added in the load queue and we found a store that is 
                // "logically later in the program order but already executed" => this is not correctly executed WAR
                // due to out-of-order execution, this case is possible, for example
                // 1) application is load intensive and load queue is full
                // 2) we have loads that can't be added in the load queue
                // 3) subsequent stores logically behind in the program order are added in the store queue first

                // thanks to the store buffer, data is not written back to the memory system until retirement
                // also due to in-order retirement, this "already executed store" cannot be retired until we finish the prior load instruction 
                // if we detect WAR when a load is added in the load queue, just let the load instruction to access the memory system
                // no need to mark any dependency because this is actually WAR not RAW

                // do not forward data from the store queue since this is WAR
                // just read correct data from data cache

                LQ.entry[lq_index].physical_address = 0;
                LQ.entry[lq_index].translated = 0;
                LQ.entry[lq_index].fetched = 0;
                
                //DP(if(warmup_complete[cpu]) {
                //cout << "[LQ] " << __func__ << " instr_id: " << (LQ.entry[lq_index].instr_id >> LOG2_THREADS) << " reset fetched: " << +LQ.entry[lq_index].fetched;
                //cout << " to obey WAR store instr_id: " << SQ.entry[i].instr_id << " cycle: " << current_core_cycle[cpu] << endl; });
            }
        }
	}
    }

    if (forwarding_index != SQ.SIZE) { // we have a store-to-load forwarding

        if ((SQ.entry[forwarding_index].fetched == COMPLETED) && (SQ.entry[forwarding_index].event_cycle <= current_core_cycle[cpu])) {
           
	    //@Vishal: count RAW forwarding
	    sim_RAW_hits[((1 << LOG2_THREADS) - 1) & LQ.entry[lq_index].instr_id]++;

            //@Vishal: VIPT, translation is not required, Just mark the entry as fetched
            //LQ.entry[lq_index].physical_address = (SQ.entry[forwarding_index].physical_address & ~(uint64_t) ((1 << LOG2_BLOCK_SIZE) - 1)) | (LQ.entry[lq_index].virtual_address & ((1 << LOG2_BLOCK_SIZE) - 1));
            LQ.entry[lq_index].translated = COMPLETED;
            LQ.entry[lq_index].fetched = COMPLETED;	
            uint32_t fwr_rob_index = LQ.entry[lq_index].rob_index;
            ROB.entry[fwr_rob_index].num_mem_ops--;
            ROB.entry[fwr_rob_index].event_cycle = current_core_cycle[cpu];
            if (ROB.entry[fwr_rob_index].num_mem_ops < 0) {
                cerr << "instr_id: " << ROB.entry[fwr_rob_index].instr_id << endl;
                assert(0);
            }
            if (ROB.entry[fwr_rob_index].num_mem_ops == 0)
                inflight_mem_executions++;

            //DP(if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & LQ.entry[lq_index].instr_id]) {
            //cout << "[LQ] " << __func__ << " instr_id: " << (LQ.entry[lq_index].instr_id  >> LOG2_THREADS) << hex;
            //cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " is forwarded by store instr_id: ";
            //cout << SQ.entry[forwarding_index].instr_id << " remain_num_ops: " << ROB.entry[fwr_rob_index].num_mem_ops << " cycle: " << current_core_cycle[cpu] << endl; });

            release_load_queue(lq_index);
        }
        else
            ; // store is not executed yet, forwarding will be handled by execute_store()
    }

    // succesfully added to the load queue
    ROB.entry[rob_index].source_added[data_index] = 1;

    if (LQ.entry[lq_index].virtual_address && (LQ.entry[lq_index].producer_id == UINT64_MAX)) { // not released and no forwarding
        RTL0[RTL0_tail] = lq_index;
        RTL0_tail++;
        if (RTL0_tail == LQ_SIZE)
            RTL0_tail = 0;

        //DP (if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1 ) & LQ.entry[lq_index].instr_id]) {
        //cout << "[RTL0] " << __func__ << " instr_id: " << (LQ.entry[lq_index].instr_id >> LOG2_THREADS) << " rob_index: " << LQ.entry[lq_index].rob_index << " is added to RTL0";
        //cout << " head: " << RTL0_head << " tail: " << RTL0_tail << endl; }); 
    }

    //DP(if(warmup_complete[cpu][((1<<LOG2_THREADS) - 1) & LQ.entry[lq_index].instr_id]) {
    //cout << "[LQ] " << __func__ << " instr_id: " << (LQ.entry[lq_index].instr_id >> LOG2_THREADS);
    //cout << " is added in the LQ address: " << hex << LQ.entry[lq_index].virtual_address << dec << " translated: " << +LQ.entry[lq_index].translated;
    //cout << " fetched: " << +LQ.entry[lq_index].fetched << " index: " << lq_index << " occupancy: " << LQ.occupancy << " cycle: " << current_core_cycle[cpu] << endl; });
}

void O3_CPU::mem_RAW_dependency(uint32_t prior, uint32_t current, uint32_t data_index, uint32_t lq_index)
{
	//@Vasudha:SMT: Both rob_index (prior, current) should belong to same partition
	if ((((1 << LOG2_THREADS) - 1) & ROB.entry[prior].instr_id) != (((1 << LOG2_THREADS) - 1) & ROB.entry[current].instr_id))
	{
		cerr << "ERROR: Memory RAW dependency cannot be checked among different threads\n" ;
		assert(0);
	}
    for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        if (ROB.entry[prior].destination_memory[i] == 0)
            continue;

        if (ROB.entry[prior].destination_memory[i] == ROB.entry[current].source_memory[data_index]) { //  store-to-load forwarding check

            // we need to mark this dependency in the ROB since the producer might not be added in the store queue yet
            ROB.entry[prior].memory_instrs_depend_on_me.insert (current);   // this load cannot be executed until the prior store gets executed
            ROB.entry[prior].is_producer = 1;
            LQ.entry[lq_index].producer_id = ROB.entry[prior].instr_id; 
            LQ.entry[lq_index].translated = INFLIGHT;

            //DP (if(warmup_complete[cpu]) {
            ////cout << "[LQ] " << __func__ << " RAW producer instr_id: " << ROB.entry[prior].instr_id << " consumer_id: " << ROB.entry[current].instr_id << " lq_index: " << lq_index;
            ////cout << hex << " address: " << ROB.entry[prior].destination_memory[i] << dec << endl; });

            return;
        }
    }
}

void O3_CPU::add_store_queue(uint32_t rob_index, uint32_t data_index)
{
    uint32_t sq_index = SQ.tail;
#ifdef SANITY_CHECK
    if (SQ.entry[sq_index].virtual_address)
    {
	    cout << "INSTR_ID : " << (SQ.entry[sq_index].instr_id >> LOG2_THREADS) << " not getting store queue " << endl;
	    assert(0);
    }
#endif

    /*
    // search for an empty slot 
    uint32_t sq_index = SQ.SIZE;
    for (uint32_t i=0; i<SQ.SIZE; i++) {
        if (SQ.entry[i].virtual_address == 0) {
            sq_index = i;
            break;
        }
    }

    // sanity check
    if (sq_index == SQ.SIZE) {
        cerr << "instr_id: " << ROB.entry[rob_index].instr_id << " no empty slot in the store queue!!!" << endl;
        assert(0);
    }
    */

    sim_store_gen[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id]++;

    //cout << "add_store_queue() ROB index: " << rob_index << " instr_id: " << (ROB.entry[rob_index].instr_id >> LOG2_THREADS) << endl;
    // add it to the store queue
    ROB.entry[rob_index].sq_index[data_index] = sq_index;
    SQ.entry[sq_index].instr_id = ROB.entry[rob_index].instr_id;
    SQ.entry[sq_index].virtual_address = ROB.entry[rob_index].destination_memory[data_index];
    SQ.entry[sq_index].ip = ROB.entry[rob_index].ip;
    SQ.entry[sq_index].data_index = data_index;
    SQ.entry[sq_index].rob_index = rob_index;
    SQ.entry[sq_index].asid[0] = ROB.entry[rob_index].asid[0];
    SQ.entry[sq_index].asid[1] = ROB.entry[rob_index].asid[1];
    SQ.entry[sq_index].event_cycle = current_core_cycle[cpu] + SCHEDULING_LATENCY;

    SQ.occupancy++;
    SQ.tail++;
    if (SQ.tail == SQ.SIZE)
        SQ.tail = 0;

    // succesfully added to the store queue
    ROB.entry[rob_index].destination_added[data_index] = 1;
    
    uint32_t thread_index = ((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id;
    STA[STA_head[thread_index]] = UINT64_MAX;
    STA_head[thread_index]++;
    if (STA_head[thread_index] == STA_partition[thread_index] + 1)
        STA_head[thread_index] = thread_index == 0 ? 0 : STA_partition[thread_index - 1] + 1;

    RTS0[RTS0_tail] = sq_index;
    RTS0_tail++;
    if (RTS0_tail == SQ_SIZE)
        RTS0_tail = 0;

    //DP(if(warmup_complete[cpu]) {
    ////cout << "[SQ] " << __func__ << " instr_id: " << SQ.entry[sq_index].instr_id;
    ////cout << " is added in the SQ translated: " << +SQ.entry[sq_index].translated << " fetched: " << +SQ.entry[sq_index].fetched << " is_producer: " << +ROB.entry[rob_index].is_producer;
    ////cout << " cycle: " << current_core_cycle[cpu] << endl; });
}

void O3_CPU::operate_lsq()
{
    // handle store
    uint32_t store_issued = 0, num_iteration = 0;


    //@Vishal: VIPT Execute store without sending translation request to DTLB.
    while (store_issued < SQ_WIDTH) {
        if (RTS0[RTS0_head] < SQ_SIZE) {
            uint32_t sq_index = RTS0[RTS0_head];
            if (SQ.entry[sq_index].event_cycle <= current_core_cycle[cpu]) {
            	execute_store(SQ.entry[sq_index].rob_index, sq_index, SQ.entry[sq_index].data_index);

            	RTS0[RTS0_head] = SQ_SIZE;
                RTS0_head++;
                if (RTS0_head == SQ_SIZE)
                    RTS0_head = 0;

                store_issued++;
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTS0] is empty head: " << RTS0_head << " tail: " << RTS0_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (SQ_SIZE-1))
            break;
    }

    /*while (store_issued < SQ_WIDTH) {
        if (RTS0[RTS0_head] < SQ_SIZE) {
            uint32_t sq_index = RTS0[RTS0_head];
            if (SQ.entry[sq_index].event_cycle <= current_core_cycle[cpu]) {

                // add it to DTLB
                PACKET data_packet;

                data_packet.tlb_access = 1;
                data_packet.fill_level = FILL_L1;
                data_packet.cpu = cpu;
                data_packet.data_index = SQ.entry[sq_index].data_index;
                data_packet.sq_index = sq_index;
                if (knob_cloudsuite)
                    data_packet.address = ((SQ.entry[sq_index].virtual_address >> LOG2_PAGE_SIZE) << 9) | SQ.entry[sq_index].asid[1];
                else
                    data_packet.address = SQ.entry[sq_index].virtual_address >> LOG2_PAGE_SIZE;
                data_packet.full_addr = SQ.entry[sq_index].virtual_address;
                data_packet.instr_id = SQ.entry[sq_index].instr_id;
                data_packet.rob_index = SQ.entry[sq_index].rob_index;
                data_packet.ip = SQ.entry[sq_index].ip;
                data_packet.type = RFO;
                data_packet.asid[0] = SQ.entry[sq_index].asid[0];
                data_packet.asid[1] = SQ.entry[sq_index].asid[1];
                data_packet.event_cycle = SQ.entry[sq_index].event_cycle;

                //DP (if (warmup_complete[cpu]) {
                ////cout << "[RTS0] " << __func__ << " instr_id: " << SQ.entry[sq_index].instr_id << " rob_index: " << SQ.entry[sq_index].rob_index << " is popped from to RTS0";
                ////cout << " head: " << RTS0_head << " tail: " << RTS0_tail << endl; }); 

                int rq_index = DTLB.add_rq(&data_packet);

                if (rq_index == -2)
                    break; 
                else 
                    SQ.entry[sq_index].translated = INFLIGHT;

                RTS0[RTS0_head] = SQ_SIZE;
                RTS0_head++;
                if (RTS0_head == SQ_SIZE)
                    RTS0_head = 0;

                store_issued++;
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTS0] is empty head: " << RTS0_head << " tail: " << RTS0_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (SQ_SIZE-1))
            break;
    }

    num_iteration = 0;
    while (store_issued < SQ_WIDTH) {
        if (RTS1[RTS1_head] < SQ_SIZE) {
            uint32_t sq_index = RTS1[RTS1_head];
            if (SQ.entry[sq_index].event_cycle <= current_core_cycle[cpu]) {
                execute_store(SQ.entry[sq_index].rob_index, sq_index, SQ.entry[sq_index].data_index);

                RTS1[RTS1_head] = SQ_SIZE;
                RTS1_head++;
                if (RTS1_head == SQ_SIZE)
                    RTS1_head = 0;

                store_issued++;
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTS1] is empty head: " << RTS1_head << " tail: " << RTS1_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (SQ_SIZE-1))
            break;
    }*/

    unsigned load_issued = 0;
    num_iteration = 0;

    //@Vishal: VIPT. Send request to L1D.

    while (load_issued < LQ_WIDTH) {
        if (RTL0[RTL0_head] < LQ_SIZE) {
            uint32_t lq_index = RTL0[RTL0_head];
            if (LQ.entry[lq_index].event_cycle <= current_core_cycle[cpu]) {

            	int rq_index = execute_load(LQ.entry[lq_index].rob_index, lq_index, LQ.entry[lq_index].data_index);

            	if (rq_index != -2) {
                    RTL0[RTL0_head] = LQ_SIZE;
	                RTL0_head++;
	                if (RTL0_head == LQ_SIZE)
	                    RTL0_head = 0;

                    load_issued++;
                }
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTL1] is empty head: " << RTL1_head << " tail: " << RTL1_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (LQ_SIZE-1))
            break;
    }

    /*while (load_issued < LQ_WIDTH) {
        if (RTL0[RTL0_head] < LQ_SIZE) {
            uint32_t lq_index = RTL0[RTL0_head];
            if (LQ.entry[lq_index].event_cycle <= current_core_cycle[cpu]) {

                // add it to DTLB
                PACKET data_packet;
                data_packet.fill_level = FILL_L1;
                data_packet.cpu = cpu;
                data_packet.data_index = LQ.entry[lq_index].data_index;
                data_packet.lq_index = lq_index;
                if (knob_cloudsuite)
                    data_packet.address = ((LQ.entry[lq_index].virtual_address >> LOG2_PAGE_SIZE) << 9) | LQ.entry[lq_index].asid[1];
                else
                    data_packet.address = LQ.entry[lq_index].virtual_address >> LOG2_PAGE_SIZE;
                data_packet.full_addr = LQ.entry[lq_index].virtual_address;
                data_packet.instr_id = LQ.entry[lq_index].instr_id;
                data_packet.rob_index = LQ.entry[lq_index].rob_index;
                data_packet.ip = LQ.entry[lq_index].ip;
                data_packet.type = LOAD;
                data_packet.asid[0] = LQ.entry[lq_index].asid[0];
                data_packet.asid[1] = LQ.entry[lq_index].asid[1];
                data_packet.event_cycle = LQ.entry[lq_index].event_cycle;

                //DP (if (warmup_complete[cpu]) {
                ////cout << "[RTL0] " << __func__ << " instr_id: " << LQ.entry[lq_index].instr_id << " rob_index: " << LQ.entry[lq_index].rob_index << " is popped to RTL0";
                ////cout << " head: " << RTL0_head << " tail: " << RTL0_tail << endl; }); 

                int rq_index = DTLB.add_rq(&data_packet);

                if (rq_index == -2)
                    break; // break here
                else  
                    LQ.entry[lq_index].translated = INFLIGHT;

                RTL0[RTL0_head] = LQ_SIZE;
                RTL0_head++;
                if (RTL0_head == LQ_SIZE)
                    RTL0_head = 0;

                load_issued++;
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTL0] is empty head: " << RTL0_head << " tail: " << RTL0_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (LQ_SIZE-1))
            break;
    }

    num_iteration = 0;
    while (load_issued < LQ_WIDTH) {
        if (RTL1[RTL1_head] < LQ_SIZE) {
            uint32_t lq_index = RTL1[RTL1_head];
            if (LQ.entry[lq_index].event_cycle <= current_core_cycle[cpu]) {
                int rq_index = execute_load(LQ.entry[lq_index].rob_index, lq_index, LQ.entry[lq_index].data_index);

                if (rq_index != -2) {
                    RTL1[RTL1_head] = LQ_SIZE;
                    RTL1_head++;
                    if (RTL1_head == LQ_SIZE)
                        RTL1_head = 0;

                    load_issued++;
                }
            }
        }
        else {
            ////DP (if (warmup_complete[cpu]) {
            //////cout << "[RTL1] is empty head: " << RTL1_head << " tail: " << RTL1_tail << endl; });
            break;
        }

        num_iteration++;
        if (num_iteration == (LQ_SIZE-1))
            break;
    }*/
}

void O3_CPU::execute_store(uint32_t rob_index, uint32_t sq_index, uint32_t data_index)
{
    SQ.entry[sq_index].fetched = COMPLETED;
    SQ.entry[sq_index].event_cycle = current_core_cycle[cpu];
   
    ROB.entry[rob_index].num_mem_ops--;
    ROB.entry[rob_index].event_cycle = current_core_cycle[cpu];
    if (ROB.entry[rob_index].num_mem_ops < 0) {
        cerr << "instr_id: " << ROB.entry[rob_index].instr_id << endl;
        assert(0);
    }

    if (ROB.entry[rob_index].num_mem_ops == 0)	
    	inflight_mem_executions++;
    
       //DP (if (warmup_complete[cpu]) {
       //	cout << "[SQ1] " << __func__ << " instr_id: " << SQ.entry[sq_index].instr_id << hex;
       //cout << " full_address: " << SQ.entry[sq_index].physical_address << dec << " remain_mem_ops: " << ROB.entry[rob_index].num_mem_ops;
    	//cout << " event_cycle: " << SQ.entry[sq_index].event_cycle << endl; });

    // resolve RAW dependency after DTLB access
    // check if this store has dependent loads
    if (ROB.entry[rob_index].is_producer) {
	ITERATE_SET(dependent,ROB.entry[rob_index].memory_instrs_depend_on_me, ROB_SIZE) {
            // check if dependent loads are already added in the load queue
            for (uint32_t j=0; j<NUM_INSTR_SOURCES; j++) { // which one is dependent?
                if (ROB.entry[dependent].source_memory[j] && ROB.entry[dependent].source_added[j]) {
                    if (ROB.entry[dependent].source_memory[j] == SQ.entry[sq_index].virtual_address) { // this is required since a single instruction can issue multiple loads
		
		     	 if ((((1 << LOG2_THREADS) - 1) & ROB.entry[dependent].instr_id) != (((1 << LOG2_THREADS) - 1) & SQ.entry[sq_index].instr_id)) 
			 {
				 //cout << "ERROR : Dependency between different threads " << endl;
				assert(0);
			 }		 
                         //@Vishal: count RAW forwarding
			 sim_RAW_hits[((1 << LOG2_THREADS) - 1) & ROB.entry[dependent].instr_id]++;

                        // now we can resolve RAW dependency
                        uint32_t lq_index = ROB.entry[dependent].lq_index[j];
#ifdef SANITY_CHECK
                        if (lq_index >= LQ.SIZE)
                            assert(0);
                        if (LQ.entry[lq_index].producer_id != SQ.entry[sq_index].instr_id) {
                            cerr << "[SQ2] " << __func__ << " lq_index: " << lq_index << " producer_id: " << LQ.entry[lq_index].producer_id;
                            cerr << " does not match to the store instr_id: " << SQ.entry[sq_index].instr_id << endl;
                            assert(0);
                        }
#endif
                        // update correspodning LQ entry
                        // @Vishal: Dependent load can now get the data, translation is not required
                        //LQ.entry[lq_index].physical_address = (SQ.entry[sq_index].physical_address & ~(uint64_t) ((1 << LOG2_BLOCK_SIZE) - 1)) | (LQ.entry[lq_index].virtual_address & ((1 << LOG2_BLOCK_SIZE) - 1));
                        LQ.entry[lq_index].translated = COMPLETED;
                        LQ.entry[lq_index].fetched = COMPLETED;
                        LQ.entry[lq_index].event_cycle = current_core_cycle[cpu];

                        uint32_t fwr_rob_index = LQ.entry[lq_index].rob_index;
                        ROB.entry[fwr_rob_index].num_mem_ops--;
                        ROB.entry[fwr_rob_index].event_cycle = current_core_cycle[cpu];
#ifdef SANITY_CHECK
                        if (ROB.entry[fwr_rob_index].num_mem_ops < 0) {
                            cerr << "instr_id: " << ROB.entry[fwr_rob_index].instr_id << endl;
                            assert(0);
                        }
#endif
                        if (ROB.entry[fwr_rob_index].num_mem_ops == 0)
                            inflight_mem_executions++;

                        /*DP(if(warmup_complete[cpu]) {
                        cout << "[LQ3] " << __func__ << " instr_id: " << LQ.entry[lq_index].instr_id << hex;
                        cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " is forwarded by store instr_id: ";
                        cout << SQ.entry[sq_index].instr_id << " remain_num_ops: " << ROB.entry[fwr_rob_index].num_mem_ops << " cycle: " << current_core_cycle[cpu] << endl; });
			*/
                        release_load_queue(lq_index);

                        // clear dependency bit
                        if (j == (NUM_INSTR_SOURCES-1))
                            ROB.entry[rob_index].memory_instrs_depend_on_me.insert (dependent);
                    }
                }
            }
        }
    }
}

int O3_CPU::execute_load(uint32_t rob_index, uint32_t lq_index, uint32_t data_index)
{
    // add it to L1D
    PACKET data_packet;
    data_packet.fill_level = FILL_L1;
    data_packet.fill_l1d = 1;
    data_packet.cpu = cpu;
    data_packet.data_index = LQ.entry[lq_index].data_index;
    data_packet.lq_index = lq_index;

    //@Vishal: VIPT send virtual address instead of physical address
    //data_packet.address = LQ.entry[lq_index].physical_address >> LOG2_BLOCK_SIZE;
    //data_packet.full_addr = LQ.entry[lq_index].physical_address;
    data_packet.address = LQ.entry[lq_index].virtual_address >> LOG2_BLOCK_SIZE;
    data_packet.full_addr = LQ.entry[lq_index].virtual_address;
    data_packet.full_virtual_address = LQ.entry[lq_index].virtual_address;

    data_packet.instr_id = LQ.entry[lq_index].instr_id;
    data_packet.rob_index = LQ.entry[lq_index].rob_index;
    data_packet.ip = LQ.entry[lq_index].ip;

    //Neelu: Setting critical ip flag in packet if ip is identified as critical
        uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & data_packet.instr_id;;
	if (knob_cloudsuite)
		thread_index = 0;
#ifdef DETECT_CRITICAL_IPS
	if(critical_ips[cpu][thread_index].find(LQ.entry[lq_index].ip & ((1 << PARTIAL_TAG) - 1)) != critical_ips[cpu][thread_index].end())
		data_packet.critical_ip_flag = 1;
#endif 

    data_packet.type = LOAD;
    data_packet.asid[0] = LQ.entry[lq_index].asid[0];
    data_packet.asid[1] = LQ.entry[lq_index].asid[1];
    data_packet.event_cycle = LQ.entry[lq_index].event_cycle;
#ifdef DETECT_CRITICAL_TRANSLATIONS
	if (critical_translation[cpu][thread_index].find(data_packet.full_addr >> LOG2_PAGE_SIZE) != critical_translation[cpu][thread_index].end())
		data_packet.critical_ip_flag = 1;
#endif
    
    int rq_index = L1D.add_rq(&data_packet);

    if (rq_index == -2)
        return rq_index;
    else 
    {
        LQ.entry[lq_index].fetched = INFLIGHT;
	
	sim_load_sent[((1 << LOG2_THREADS) - 1) & LQ.entry[lq_index].instr_id]++;
    }

    return rq_index;
}

void O3_CPU::complete_execution(uint32_t rob_index)
{
    if (ROB.entry[rob_index].is_memory == 0) {
        if ((ROB.entry[rob_index].executed == INFLIGHT) && (ROB.entry[rob_index].event_cycle <= current_core_cycle[cpu])) {

            ROB.entry[rob_index].executed = COMPLETED; 
	    inflight_reg_executions--;
            completed_executions++;

            if (ROB.entry[rob_index].reg_RAW_producer)
                reg_RAW_release(rob_index);

            if (ROB.entry[rob_index].branch_mispredicted)
	      	{
				if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id])
				{
					//@Vasudha:SMT: Find thread_ID from instr_id
					fetch_resume_cycle[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id] = current_core_cycle[cpu] + 1;
					//cout << "fetch_Resume cycle decided for rob_index - " << rob_index << endl;
				}
				if(ROB.entry[rob_index].branch_taken)
				{
					fill_btb(ROB.entry[rob_index].ip, ROB.entry[rob_index].branch_target, ROB.entry[rob_index].instr_id);
				}
	     	}

			if(ROB.entry[rob_index].btb_miss && ROB.entry[rob_index].branch_mispredicted == 0)
        	{
            	uint8_t branch_type = ROB.entry[rob_index].branch_type;
            	assert(branch_type != BRANCH_DIRECT_JUMP && branch_type != BRANCH_DIRECT_CALL && branch_type != BRANCH_CONDITIONAL);
				if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id])
				{ 
                			fetch_resume_cycle[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id]  = current_core_cycle[cpu] + 1; //Resume fetch from next cycle.
				}
				fill_btb(ROB.entry[rob_index].ip, ROB.entry[rob_index].branch_target, ROB.entry[rob_index].instr_id);
        	}


            //DP(if(warmup_complete[cpu]) {
            //cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[rob_index].instr_id;
            //cout << " branch_mispredicted: " << +ROB.entry[rob_index].branch_mispredicted << " fetch_stall: " << +fetch_stall;
            //cout << " event: " << ROB.entry[rob_index].event_cycle << endl; });
        }
    }
    else {
        if (ROB.entry[rob_index].num_mem_ops == 0) {
            if ((ROB.entry[rob_index].executed == INFLIGHT) && (ROB.entry[rob_index].event_cycle <= current_core_cycle[cpu])) {
                ROB.entry[rob_index].executed = COMPLETED;
                inflight_mem_executions--;
                completed_executions++;
                
                if (ROB.entry[rob_index].reg_RAW_producer)
                    reg_RAW_release(rob_index);

			if (ROB.entry[rob_index].branch_mispredicted)
	      	{
				if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id])
				{
					fetch_resume_cycle[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id] = current_core_cycle[cpu] + 1;
				}
				if(ROB.entry[rob_index].branch_taken)
				{
					fill_btb(ROB.entry[rob_index].ip, ROB.entry[rob_index].branch_target, ROB.entry[rob_index].instr_id);
				}
	     	}

		if(ROB.entry[rob_index].btb_miss && ROB.entry[rob_index].branch_mispredicted == 0)
        	{
            		uint8_t branch_type = ROB.entry[rob_index].branch_type;
            		assert(branch_type != BRANCH_DIRECT_JUMP && branch_type != BRANCH_DIRECT_CALL && branch_type != BRANCH_CONDITIONAL);
				if(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id])
				{ 
                			fetch_resume_cycle[((1 << LOG2_THREADS) - 1) & ROB.entry[rob_index].instr_id] = current_core_cycle[cpu] + 1; //Resume fetch from next cycle.
				}
				fill_btb(ROB.entry[rob_index].ip, ROB.entry[rob_index].branch_target, ROB.entry[rob_index].instr_id);
        	}

                //DP(if(warmup_complete[cpu]) {
                //cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[rob_index].instr_id;
                //cout << " is_memory: " << +ROB.entry[rob_index].is_memory << " branch_mispredicted: " << +ROB.entry[rob_index].branch_mispredicted;
                //cout << " fetch_stall: " << +fetch_stall << " event: " << ROB.entry[rob_index].event_cycle << " current: " << current_core_cycle[cpu] << endl; });
            }
        }
    }
}

void O3_CPU::reg_RAW_release(uint32_t rob_index)
{
    // if (!ROB.entry[rob_index].registers_insers_depend_on_me.empty()) 

    ITERATE_SET(i,ROB.entry[rob_index].registers_instrs_depend_on_me, ROB_SIZE) {
        for (uint32_t j=0; j<NUM_INSTR_SOURCES; j++) {
            if (ROB.entry[rob_index].registers_index_depend_on_me[j].search (i)) {
                ROB.entry[i].num_reg_dependent--;

                if (ROB.entry[i].num_reg_dependent == 0) {
                    ROB.entry[i].reg_ready = 1;
                    if (ROB.entry[i].is_memory)
                        ROB.entry[i].scheduled = INFLIGHT;
                    else {
                        ROB.entry[i].scheduled = COMPLETED;

#ifdef SANITY_CHECK
                        if (RTE0[RTE0_tail] < ROB_SIZE)
                            assert(0);
#endif
                        // remember this rob_index in the Ready-To-Execute array 0
                        RTE0[RTE0_tail] = i;

                        //DP (if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[i].instr_id]) {
                        //cout << "[RTE0] " << __func__ << " instr_id: " << ROB.entry[i].instr_id << " rob_index: " << i << " is added to RTE0";
                        //cout << " head: " << RTE0_head << " tail: " << RTE0_tail << endl; }); 

                        RTE0_tail++;
                        if (RTE0_tail == ROB_SIZE)
                            RTE0_tail = 0;

                    }
                }

                //DP (if (warmup_complete[cpu]) {
                //cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[rob_index].instr_id << " releases instr_id: ";
                //cout << ROB.entry[i].instr_id << " reg_index: " << +ROB.entry[i].source_registers[j] << " num_reg_dependent: " << ROB.entry[i].num_reg_dependent << " cycle: " << current_core_cycle[cpu] << endl; });
            }
        }
    }
}

void O3_CPU::operate_cache()
{
    ITLB.operate();
    DTLB.operate();
    STLB.operate();
#ifdef INS_PAGE_TABLE_WALKER
    PTW.operate();
#endif
    L1I.operate();
    L1D.operate();
    L2C.operate();

    // also handle per-cycle prefetcher operation
    l1i_prefetcher_cycle_operate();
}

void O3_CPU::update_rob()
{
    //@Vishal: VIPT ITLB processed entries will be handled by L1I cache.
    //if (ITLB.PROCESSED.occupancy && (ITLB.PROCESSED.entry[ITLB.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
    //    complete_instr_fetch(&ITLB.PROCESSED, 1);

    if (L1I.PROCESSED.occupancy && (L1I.PROCESSED.entry[L1I.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
        complete_instr_fetch(&L1I.PROCESSED, 0);

    //@Vishal: VIPT DTLB processed entries will be handled by L1D cache
    //if (DTLB.PROCESSED.occupancy && (DTLB.PROCESSED.entry[DTLB.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
    //    complete_data_fetch(&DTLB.PROCESSED, 1);

    if (L1D.PROCESSED.occupancy && (L1D.PROCESSED.entry[L1D.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
        complete_data_fetch(&L1D.PROCESSED, 0);

    // update ROB entries with completed executions
    if ((inflight_reg_executions > 0) || (inflight_mem_executions > 0)) {
        for (uint32_t thread_num = 0; thread_num < ooo_cpu[cpu].thread; thread_num++)
	{
		//cout << " reg inflight: " << inflight_reg_executions << " mem: " << inflight_mem_executions << " head= " << ROB.head[thread_num] << " tail= " << ROB.tail[thread_num] << " core cycle: " << current_core_cycle[cpu] << endl ;
	    	if (ROB.head[thread_num] < ROB.tail[thread_num]) {
            		for (uint32_t i=ROB.head[thread_num]; i<ROB.tail[thread_num]; i++) 
                		complete_execution(i);
        	}
        	else {
            		for (uint32_t i=ROB.head[thread_num]; i<=ROB.partition[thread_num]; i++)
                		complete_execution(i);
            		for (uint32_t i=(thread_num==0 ? 0 : ROB.partition[thread_num - 1] + 1); i<ROB.tail[thread_num]; i++)
                		complete_execution(i);
        	}
	}
    }
}

void O3_CPU::complete_instr_fetch(PACKET_QUEUE *queue, uint8_t is_it_tlb)
{
	//@Vishal: VIPT, TLB request should not be handled here
	assert(is_it_tlb == 0);


//Neelu: Todo: IFETCH_BUFFER entries, mark translated and fetched. Since TLB requests will not be handled here due to VIPT, I am not
// inserting code for is_it_tlb condition.

    uint32_t index = queue->head,
             rob_index = queue->entry[index].rob_index,
	     num_fetched = 0;

    uint64_t complete_ip = queue->entry[index].ip;

//Neelu: Marking IFETCH_BUFFER entries translated and fetched and then returning. 

     for (uint16_t thread_index = 0; thread_index < thread; thread_index++)
     {
     	if(!knob_cloudsuite)
	       	thread_index = ((1 << LOG2_THREADS) - 1) & queue->entry[index].instr_id;
     	for(uint32_t j=0; j<IFETCH_BUFFER[thread_index].SIZE; j++)
     	{
		     if(((IFETCH_BUFFER[thread_index].entry[j].ip)>>6) == ((complete_ip)>>6))
		     {
			     IFETCH_BUFFER[thread_index].entry[j].translated = COMPLETED;
			     IFETCH_BUFFER[thread_index].entry[j].fetched = COMPLETED;
		     }
     	}
	if (!knob_cloudsuite)
		break;
     }

     // remove this entry
     queue->remove_queue(&queue->entry[index]);

     return;

//old function

#ifdef SANITY_CHECK
	//DP (if (warmup_complete[cpu]) {
    ////cout << "**(complete_instr_fetch)queue->entry[index].full_addr = "<< hex << queue->entry[index].full_addr << "  instr_id = "<< queue->entry[index].instr_id << " index="<<index<< " rob_index="<<queue->entry[index].rob_index<< endl; });
    uint32_t a;
    if (rob_index != (a = check_rob(queue->entry[index].instr_id)))
    {
	    ////cout << "complete_instr_fetch, rob_index ="<<rob_index<< " a = " << a << endl; 
	    assert(0);
    }
#endif

    // update ROB entry
    if (is_it_tlb) {
        ROB.entry[rob_index].translated = COMPLETED;
        ROB.entry[rob_index].instruction_pa = (queue->entry[index].instruction_pa << LOG2_PAGE_SIZE) | (ROB.entry[rob_index].ip & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
    }
    else
        ROB.entry[rob_index].fetched = COMPLETED;
    ROB.entry[rob_index].event_cycle = current_core_cycle[cpu];
    num_fetched++;

    //DP ( if (warmup_complete[cpu]) {
    ////cout << "[" << queue->NAME << "] " << __func__ << " cpu: " << cpu <<  " instr_id: " << ROB.entry[rob_index].instr_id;
    ////cout << " ip: " << hex << ROB.entry[rob_index].ip << " address: " << ROB.entry[rob_index].instruction_pa << dec;
    ////cout << " translated: " << +ROB.entry[rob_index].translated << " fetched: " << +ROB.entry[rob_index].fetched;
    ////cout << " event_cycle: " << ROB.entry[rob_index].event_cycle << endl; });

    // check if other instructions were merged
    if (queue->entry[index].instr_merged) {
	ITERATE_SET(i,queue->entry[index].rob_index_depend_on_me, ROB_SIZE) {
	    //@Vasudha:SMT: check if both entries are from same thread
	    if ((((1 << LOG2_THREADS) - 1) & queue->entry[index].instr_id) != (((1 << LOG2_THREADS) - 1) & ROB.entry[i].instr_id))
	    {
		    //cout << "ERROR: No communication is allowed among different threads\n";
		    assert(0);
	    }		    

            // update ROB entry
            if (is_it_tlb) {
                ROB.entry[i].translated = COMPLETED;
                ROB.entry[i].instruction_pa = (queue->entry[index].instruction_pa << LOG2_PAGE_SIZE) | (ROB.entry[i].ip & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            }
            else
                ROB.entry[i].fetched = COMPLETED;
            ROB.entry[i].event_cycle = current_core_cycle[cpu] + (num_fetched / FETCH_WIDTH);
            num_fetched++;

            //DP ( if (warmup_complete[cpu]) {
            ////cout << "[" << queue->NAME << "] " << __func__ << " cpu: " << cpu <<  " instr_id: " << ROB.entry[i].instr_id;
            ////cout << " ip: " << hex << ROB.entry[i].ip << " address: " << ROB.entry[i].instruction_pa << dec;
            ////cout << " translated: " << +ROB.entry[i].translated << " fetched: " << +ROB.entry[i].fetched << " provider: " << ROB.entry[rob_index].instr_id;
            ////cout << " event_cycle: " << ROB.entry[i].event_cycle << endl; });
        }
    }

    // remove this entry
    queue->remove_queue(&queue->entry[index]);
    
}

void O3_CPU::complete_data_fetch(PACKET_QUEUE *queue, uint8_t is_it_tlb)
{

	//@Vishal: VIPT, TLB request should not be handled here
	assert(is_it_tlb == 0);


    uint32_t index = queue->head,
             rob_index = queue->entry[index].rob_index,
             sq_index = queue->entry[index].sq_index,
             lq_index = queue->entry[index].lq_index;

#ifdef SANITY_CHECK
    if (queue->entry[index].type != RFO) {
    	//DP (if (warmup_complete[cpu]) {
            ////cout << "queue->entry[index].full_addr = "<< queue->entry[index].full_addr << endl; });
        if (rob_index != check_rob(queue->entry[index].instr_id))
        {
            assert(0);
        }
    }
#endif

    // update ROB entry
    if (is_it_tlb) { // DTLB

        if (queue->entry[index].type == RFO) {
            SQ.entry[sq_index].physical_address = (queue->entry[index].data_pa << LOG2_PAGE_SIZE) | (SQ.entry[sq_index].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            SQ.entry[sq_index].translated = COMPLETED;
            SQ.entry[sq_index].event_cycle = current_core_cycle[cpu];

            RTS1[RTS1_tail] = sq_index;
            RTS1_tail++;
            if (RTS1_tail == SQ_SIZE)
                RTS1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " RFO instr_id: " << SQ.entry[sq_index].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +SQ.entry[sq_index].translated << hex << " page: " << (SQ.entry[sq_index].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << SQ.entry[sq_index].physical_address << dec << " store_merged: " << +queue->entry[index].store_merged;
            ////cout << " load_merged: " << +queue->entry[index].load_merged << endl; }); 

            handle_merged_translation(&queue->entry[index]);
        }
        else { 
            LQ.entry[lq_index].physical_address = (queue->entry[index].data_pa << LOG2_PAGE_SIZE) | (LQ.entry[lq_index].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            LQ.entry[lq_index].translated = COMPLETED;
            LQ.entry[lq_index].event_cycle = current_core_cycle[cpu];

            RTL1[RTL1_tail] = lq_index;
            RTL1_tail++;
            if (RTL1_tail == LQ_SIZE)
                RTL1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[RTL1] " << __func__ << " instr_id: " << LQ.entry[lq_index].instr_id << " rob_index: " << LQ.entry[lq_index].rob_index << " is added to RTL1";
            ////cout << " head: " << RTL1_head << " tail: " << RTL1_tail << endl; }); 

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[lq_index].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +LQ.entry[lq_index].translated << hex << " page: " << (LQ.entry[lq_index].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " store_merged: " << +queue->entry[index].store_merged;
            ////cout << " load_merged: " << +queue->entry[index].load_merged << endl; }); 

            handle_merged_translation(&queue->entry[index]);
        }

        ROB.entry[rob_index].event_cycle = queue->entry[index].event_cycle;
    }
    else { // L1D

        if (queue->entry[index].type == RFO)
            handle_merged_load(&queue->entry[index]);
        else { 
#ifdef SANITY_CHECK
            if (queue->entry[index].store_merged)
                assert(0);
#endif
            LQ.entry[lq_index].fetched = COMPLETED;
            LQ.entry[lq_index].event_cycle = current_core_cycle[cpu];
            ROB.entry[rob_index].num_mem_ops--;
            ROB.entry[rob_index].event_cycle = queue->entry[index].event_cycle;

#ifdef SANITY_CHECK
            if (ROB.entry[rob_index].num_mem_ops < 0) {
                cerr << "instr_id: " << ROB.entry[rob_index].instr_id << endl;
                assert(0);
            }
#endif
            if (ROB.entry[rob_index].num_mem_ops == 0)
                inflight_mem_executions++;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[lq_index].instr_id;
            ////cout << " L1D_FETCH_DONE fetched: " << +LQ.entry[lq_index].fetched << hex << " address: " << (LQ.entry[lq_index].physical_address>>LOG2_BLOCK_SIZE);
            ////cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " remain_mem_ops: " << ROB.entry[rob_index].num_mem_ops;
            ////cout << " load_merged: " << +queue->entry[index].load_merged << " inflight_mem: " << inflight_mem_executions << endl; }); 

            release_load_queue(lq_index);
            handle_merged_load(&queue->entry[index]);
        }
    }

    // remove this entry
    queue->remove_queue(&queue->entry[index]);
}

//@Vishal: This function is not used anywhere
/*
void O3_CPU::handle_o3_fetch(PACKET *current_packet, uint32_t cache_type)
{
    uint32_t rob_index = current_packet->rob_index,
             sq_index  = current_packet->sq_index,
             lq_index  = current_packet->lq_index;

    // update ROB entry
    if (cache_type == 0) { // DTLB

#ifdef SANITY_CHECK
        if (rob_index != check_rob(current_packet->instr_id))
            assert(0);
#endif
        if (current_packet->type == RFO) {
            SQ.entry[sq_index].physical_address = (current_packet->data_pa << LOG2_PAGE_SIZE) | (SQ.entry[sq_index].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            SQ.entry[sq_index].translated = COMPLETED;

            RTS1[RTS1_tail] = sq_index;
            RTS1_tail++;
            if (RTS1_tail == SQ_SIZE)
                RTS1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " RFO instr_id: " << SQ.entry[sq_index].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +SQ.entry[sq_index].translated << hex << " page: " << (SQ.entry[sq_index].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << SQ.entry[sq_index].physical_address << dec << " store_merged: " << +current_packet->store_merged;
            ////cout << " load_merged: " << +current_packet->load_merged << endl; }); 

            handle_merged_translation(current_packet);
        }
        else { 
            LQ.entry[lq_index].physical_address = (current_packet->data_pa << LOG2_PAGE_SIZE) | (LQ.entry[lq_index].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            LQ.entry[lq_index].translated = COMPLETED;

            RTL1[RTL1_tail] = lq_index;
            RTL1_tail++;
            if (RTL1_tail == LQ_SIZE)
                RTL1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[RTL1] " << __func__ << " instr_id: " << LQ.entry[lq_index].instr_id << " rob_index: " << LQ.entry[lq_index].rob_index << " is added to RTL1";
            ////cout << " head: " << RTL1_head << " tail: " << RTL1_tail << endl; }); 

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[lq_index].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +LQ.entry[lq_index].translated << hex << " page: " << (LQ.entry[lq_index].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " store_merged: " << +current_packet->store_merged;
            ////cout << " load_merged: " << +current_packet->load_merged << endl; }); 

            handle_merged_translation(current_packet);
        }

        ROB.entry[rob_index].event_cycle = current_packet->event_cycle;
    }
    else { // L1D

        if (current_packet->type == RFO)
            handle_merged_load(current_packet);
        else { // do traditional things
#ifdef SANITY_CHECK
            if (rob_index != check_rob(current_packet->instr_id))
                assert(0);

            if (current_packet->store_merged)
                assert(0);
#endif
            LQ.entry[lq_index].fetched = COMPLETED;
            ROB.entry[rob_index].num_mem_ops--;

#ifdef SANITY_CHECK
            if (ROB.entry[rob_index].num_mem_ops < 0) {
                cerr << "instr_id: " << ROB.entry[rob_index].instr_id << endl;
                assert(0);
            }
#endif
            if (ROB.entry[rob_index].num_mem_ops == 0)
                inflight_mem_executions++;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[lq_index].instr_id;
            ////cout << " L1D_FETCH_DONE fetched: " << +LQ.entry[lq_index].fetched << hex << " address: " << (LQ.entry[lq_index].physical_address>>LOG2_BLOCK_SIZE);
            ////cout << " full_addr: " << LQ.entry[lq_index].physical_address << dec << " remain_mem_ops: " << ROB.entry[rob_index].num_mem_ops;
            ////cout << " load_merged: " << +current_packet->load_merged << " inflight_mem: " << inflight_mem_executions << endl; }); 

            release_load_queue(lq_index);

            handle_merged_load(current_packet);

            ROB.entry[rob_index].event_cycle = current_packet->event_cycle;
        }
    }
}*/

void O3_CPU::handle_merged_translation(PACKET *provider)
{

	//@Vishal: VIPT, Translation are not sent from processor, so this code should not be executed.
	assert(0);


    if (provider->store_merged) {
	ITERATE_SET(merged, provider->sq_index_depend_on_me, SQ.SIZE) {
            if ((((1 << LOG2_THREADS) - 1) & provider->instr_id) != (((1 << LOG2_THREADS) - 1) & SQ.entry[merged].instr_id))
	    {
		    //cout << " ERROR: No communication os allowed among different threads\n";
		    assert(0);
	    }
	    SQ.entry[merged].translated = COMPLETED;
            SQ.entry[merged].physical_address = (provider->data_pa << LOG2_PAGE_SIZE) | (SQ.entry[merged].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            SQ.entry[merged].event_cycle = current_core_cycle[cpu];

            RTS1[RTS1_tail] = merged;
            RTS1_tail++;
            if (RTS1_tail == SQ_SIZE)
                RTS1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " store instr_id: " << SQ.entry[merged].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +SQ.entry[merged].translated << hex << " page: " << (SQ.entry[merged].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << SQ.entry[merged].physical_address << dec << " by instr_id: " << +provider->instr_id << endl; });
        }
    }
    if (provider->load_merged) {
	ITERATE_SET(merged, provider->lq_index_depend_on_me, LQ.SIZE) {
            if ((((1 < LOG2_THREADS) - 1) & provider->instr_id) != (((1 << LOG2_THREADS) - 1) & LQ.entry[merged].instr_id))
	    {
		    //cout << " ERROR: No communication is allowed among different threads\n";
		    assert(0);
	    }
	    LQ.entry[merged].translated = COMPLETED;
            LQ.entry[merged].physical_address = (provider->data_pa << LOG2_PAGE_SIZE) | (LQ.entry[merged].virtual_address & ((1 << LOG2_PAGE_SIZE) - 1)); // translated address
            LQ.entry[merged].event_cycle = current_core_cycle[cpu];

            RTL1[RTL1_tail] = merged;
            RTL1_tail++;
            if (RTL1_tail == LQ_SIZE)
                RTL1_tail = 0;

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[RTL1] " << __func__ << " instr_id: " << LQ.entry[merged].instr_id << " rob_index: " << LQ.entry[merged].rob_index << " is added to RTL1";
            ////cout << " head: " << RTL1_head << " tail: " << RTL1_tail << endl; }); 

            //DP (if (warmup_complete[cpu]) {
            ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[merged].instr_id;
            ////cout << " DTLB_FETCH_DONE translation: " << +LQ.entry[merged].translated << hex << " page: " << (LQ.entry[merged].physical_address>>LOG2_PAGE_SIZE);
            ////cout << " full_addr: " << LQ.entry[merged].physical_address << dec << " by instr_id: " << +provider->instr_id << endl; });
        }
    }
}

void O3_CPU::handle_merged_load(PACKET *provider)
{
    ITERATE_SET(merged, provider->lq_index_depend_on_me, LQ.SIZE) {
        uint32_t merged_rob_index = LQ.entry[merged].rob_index;
	if ((((1 << LOG2_THREADS) - 1) & LQ.entry[merged].instr_id) != (((1 << LOG2_THREADS) - 1) & ROB.entry[merged_rob_index].instr_id))
	{
		//cout << " ERROR: No communication is allowed among different threads\n";
		assert(0);
	}
        LQ.entry[merged].fetched = COMPLETED;
        LQ.entry[merged].event_cycle = current_core_cycle[cpu];
     
	ROB.entry[merged_rob_index].num_mem_ops--;
        ROB.entry[merged_rob_index].event_cycle = current_core_cycle[cpu];

#ifdef SANITY_CHECK
        if (ROB.entry[merged_rob_index].num_mem_ops < 0) {
            cerr << "instr_id: " << ROB.entry[merged_rob_index].instr_id << " rob_index: " << merged_rob_index << endl;
            assert(0);
        }
#endif
        if (ROB.entry[merged_rob_index].num_mem_ops == 0)
            inflight_mem_executions++;
	
               //DP (if (warmup_complete[cpu]) {
        ////cout << "[ROB] " << __func__ << " load instr_id: " << LQ.entry[merged].instr_id;
        ////cout << " L1D_FETCH_DONE translation: " << +LQ.entry[merged].translated << hex << " address: " << (LQ.entry[merged].physical_address>>LOG2_BLOCK_SIZE);
        ////cout << " full_addr: " << LQ.entry[merged].physical_address << dec << " by instr_id: " << +provider->instr_id;
        ////cout << " remain_mem_ops: " << ROB.entry[merged_rob_index].num_mem_ops << endl; });

        release_load_queue(merged);
    }
}

void O3_CPU::release_load_queue(uint32_t lq_index)
{
    // release LQ entries
    //DP ( if (warmup_complete[cpu]) {
    ////cout << "[LQ] " << __func__ << " instr_id: " << LQ.entry[lq_index].instr_id << " releases lq_index: " << lq_index;
    ////cout << hex << " full_addr: " << LQ.entry[lq_index].physical_address << dec << endl; });

    LSQ_ENTRY empty_entry;
    LQ.entry[lq_index] = empty_entry;
    LQ.occupancy--;
}

void O3_CPU::retire_rob()
{
	uint16_t thread_index = last_retired_thread + 1;
	if (thread_index == ooo_cpu[cpu].thread)
		thread_index = 0;
	uint32_t count = 0;
	uint32_t num_of_iterations = 0;
	/*#ifdef DETECT_CRITICAL_TRANSLATIONS
	if (current_core_cycle[cpu] % 1000000 == 0)
	{
		cout << "CRITICAL_TRANSLATION SIZE : " << critical_translation[cpu][0].size() << endl; 
		critical_translation[cpu][0].clear();
	}
	#endif*/

    	for (uint32_t n=0; n<RETIRE_WIDTH; n++) 
	{
		count++;
		//check for infinite loop
		if (count > RETIRE_WIDTH * thread)
			return;
		num_of_iterations = 0;
		while (ROB.entry[ROB.head[thread_index]].ip == 0)
		{
			thread_index++;
			if (thread_index == thread)
				thread_index = 0;
			if (num_of_iterations == thread)
				return;
			num_of_iterations++;
		}

        	// retire is in-order
        	if (ROB.entry[ROB.head[thread_index]].executed != COMPLETED) {

#ifdef ROB_STALL_STATS
		 if (all_warmup_complete > thread && simulation_complete[cpu][thread_index] == 0)
	         {

		        uint64_t curr_instr_id = ROB.entry[ROB.head[thread_index]].instr_id;

		        total_stall[thread_index].insert(curr_instr_id);

		        for (uint32_t i=0; i< STLB.MSHR_SIZE; i++){
				if (STLB.MSHR.entry[i].instr_id == curr_instr_id)
				{
					stlb_miss_stall[thread_index].insert(curr_instr_id);
			        } 
		        }

		        for (uint32_t i=0; i< L1D.MSHR_SIZE; i++)
			{
				if (L1D.MSHR.entry[i].instr_id == curr_instr_id)
				{
					if (L1D.MSHR.entry[i].type == LOAD_TRANSLATION)
					{
			   			l1d_tr_miss_stall[thread_index].insert(curr_instr_id);
			  		} 
					else if(L1D.MSHR.entry[i].type == LOAD) 
					{
						l1d_load_miss_stall[thread_index].insert(curr_instr_id);
			  		}	
				}		
		        }

		        for (uint32_t i=0; i< L2C.MSHR_SIZE; i++)
			{
				if (L2C.MSHR.entry[i].instr_id == curr_instr_id)
				{
			  		if (L2C.MSHR.entry[i].type == LOAD_TRANSLATION)
					{
						l2c_tr_miss_stall[thread_index].insert(curr_instr_id);
				  	}
					else if(L2C.MSHR.entry[i].type == LOAD) 
					{
			    			l2c_load_miss_stall[thread_index].insert(curr_instr_id);
			  		}
				}	
		        }


		      	for (uint32_t i=0; i< uncore.LLC.MSHR_SIZE; i++)
			{
				if (uncore.LLC.MSHR.entry[i].instr_id == curr_instr_id)
				{
			  		if (uncore.LLC.MSHR.entry[i].type == LOAD_TRANSLATION)
					{
			    			llc_tr_miss_stall[thread_index].insert(curr_instr_id);
			  		} 
					else if(uncore.LLC.MSHR.entry[i].type == LOAD) 
					{
			    			llc_load_miss_stall[thread_index].insert(curr_instr_id);
			  		}
				}
		      	}
		 }
		

#endif

		        	

//Neelu: Setting stall flag and stall begin cycle if loads are stuck at the head of ROB
#if defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
		//if(warmup_complete[cpu][thread_index])
		if (all_warmup_complete > thread)
		{
			assert(warmup_complete[cpu][thread_index]);
			//@Vasudha: Check whether DTLB miss got serviced to count stall till DTLB miss only
			if (ROB.entry[ROB.head[thread_index]].load_stall_flag == 1 && ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag == 0)
			{
				assert(way_stalled[cpu][thread_index] != UINT64_MAX);
				#ifdef CHECK_STLB
				int check_stlb = 0;
				for (int way_stlb=0; way_stlb < STLB_MSHR_SIZE; way_stlb++)
				{
					if (knob_cloudsuite && (ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].address) == (((ooo_cpu[cpu].L1D.RQ.entry[way_stalled[cpu][thread_index]].full_addr >> LOG2_PAGE_SIZE) << 9) | ooo_cpu[cpu].L1D.RQ.entry[way_stalled[cpu][thread_index]].asid[1]) && (ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].returned == COMPLETED))
					{
									check_stlb = 1;
					ROB.entry[ROB.head[thread_index]].dtlb_stall_end_cycle = current_core_cycle[cpu];
					ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag = 1;
					way_stalled[cpu][thread_index] = UINT64_MAX;
									break;
					}
					else
					if (ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].address == ((ooo_cpu[cpu].L1D.RQ.entry[way_stalled[cpu][thread_index]].full_addr >> LOG2_PAGE_SIZE) << 9) | ooo_cpu[cpu].L1D.RQ.entry[way_stalled[cpu][thread_index]].asid[1] && ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].returned == COMPLETED)
					{
									check_stlb = 1;
					ROB.entry[ROB.head[thread_index]].dtlb_stall_end_cycle = current_core_cycle[cpu];
					ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag = 1;
					way_stalled[cpu][thread_index] = UINT64_MAX;
									break;
					}
				}
				
				#else
				if (ooo_cpu[0].L1D.RQ.entry[way_stalled[cpu][thread_index]].translated == 2)
				{
					ROB.entry[ROB.head[thread_index]].dtlb_stall_end_cycle = current_core_cycle[cpu];
					ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag = 1;
					way_stalled[cpu][thread_index] = UINT64_MAX;
				}
				#endif
			}
			if(ROB.entry[ROB.head[thread_index]].stall_flag == 0)
			{
				ROB.entry[ROB.head[thread_index]].stall_flag = 1;
				ROB.entry[ROB.head[thread_index]].stall_begin_cycle = current_core_cycle[cpu];
				rob_stall_count[cpu]++;
				bool flag;
				for (uint32_t i=0; i<NUM_INSTR_SOURCES; i++) {
					if(ROB.entry[ROB.head[thread_index]].lq_index[i] != UINT32_MAX)
					{
						flag = false;
						//@Vasudha: check if ROB is stalled due to DTLB miss
                                for(uint32_t way = 0; way < ooo_cpu[0].L1D.NUM_WAY; way ++)
                                {
                                  	if(ooo_cpu[0].L1D.RQ.entry[way].full_virtual_address == LQ.entry[ROB.entry[ROB.head[thread_index]].lq_index[i]].virtual_address &&
												   	ooo_cpu[0].L1D.RQ.entry[way].cpu < NUM_CPUS)
                              	{
								if (knob_cloudsuite || (((1 << LOG2_THREADS) - 1) & ooo_cpu[0].L1D.RQ.entry[way].instr_id) == (((1 << LOG2_THREADS) - 1) & LQ.entry[ROB.entry[ROB.head[thread_index]].lq_index[i]].instr_id))					    {
                                   if(ooo_cpu[0].L1D.RQ.entry[way].translated == 1)        // && ooo_cpu[0].L1D.RQ.entry[way].cpu < NUM_CPUS )
                                   {
										#ifdef CHECK_STLB
										int check_stlb = 0;
										for (int way_stlb=0; way_stlb < STLB_RQ_SIZE; way_stlb++)
										{
												if (knob_cloudsuite && ooo_cpu[cpu].STLB.RQ.entry[way_stlb].address == ((ooo_cpu[cpu].L1D.RQ.entry[way].full_addr >> LOG2_PAGE_SIZE) << 9) | ooo_cpu[cpu].L1D.RQ.entry[way].asid[1])
												{
														check_stlb = 1;
														break;
												}
												else
												if (ooo_cpu[cpu].STLB.RQ.entry[way_stlb].address  == ooo_cpu[cpu].L1D.RQ.entry[way].full_addr >> LOG2_PAGE_SIZE && 
													 (((1 << LOG2_THREADS)-1) & ooo_cpu[cpu].STLB.RQ.entry[way_stlb].instr_id) == (((1 << LOG2_THREADS) - 1) & ooo_cpu[cpu].L1D.RQ.entry[way].instr_id))
												{
														check_stlb = 1;
														break;
												}	
										}
										if (check_stlb == 0)
										{
												for (int way_stlb=0; way_stlb < STLB_MSHR_SIZE; way_stlb++)
												{
												if (knob_cloudsuite && ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].address == ((ooo_cpu[cpu].L1D.RQ.entry[way].full_addr >> LOG2_PAGE_SIZE) << 9) | ooo_cpu[cpu].L1D.RQ.entry[way].asid[1])
												{
														check_stlb = 1;
														break;
												}
												else
												if (ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].address  == ooo_cpu[cpu].L1D.RQ.entry[way].full_addr >> LOG2_PAGE_SIZE &&
													 (((1 << LOG2_THREADS)-1) & ooo_cpu[cpu].STLB.MSHR.entry[way_stlb].instr_id) == (((1 << LOG2_THREADS) - 1) & ooo_cpu[cpu].L1D.RQ.entry[way].instr_id))
												{
														check_stlb = 1;
														break;
												}	
										}

										}
										if (check_stlb == 1)
										{
										#endif
                                        ROB.entry[ROB.head[thread_index]].load_stall_flag = 1;
                                        ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag = 0;
										load_rob_stall_count[cpu]++;
                                        total_rob_occupancy[cpu]+=ROB.occupancy[thread_index];
                                        flag = true;
										#if defined(DETECT_CRITICAL_TRANSLATIONS) || defined(DETECT_CRITICAL_IPS)
										translation_stalled[cpu][thread_index] = ooo_cpu[0].L1D.RQ.entry[way].full_addr >> LOG2_PAGE_SIZE;	
										way_stalled[cpu][thread_index] = way;
										//@Vasudha: EXCLUDE COLD MISSES from getting detected as critical translations
										auto it = ooo_cpu[cpu].PTW.page_table[knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
										if (it == ooo_cpu[cpu].PTW.page_table[knob_cloudsuite?0:thread_index].end())
										{
											ROB.entry[ROB.head[thread_index]].load_stall_flag = 0;
											translation_stalled[cpu][thread_index] = UINT64_MAX;
										}
										#endif
										break;
									
										#ifdef CHECK_STLB
										}
										#endif
									}
                                    }
							}
                                        	}
						#ifndef DETECT_CRITICAL_TRANSLATIONS
						if (flag)
                                        		break;
						#endif
					}	
				}
			}
		}
#endif
            	//DP ( if (all_warmup_complete > ooo_cpu[cpu].thread) {
            		//cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[ROB.head[thread_index]].instr_id << " head: " << ROB.head[thread_index] << " is not executed yet" << endl; });
            	//return;
		n--;
		thread_index++;
		if (thread_index == thread)
			thread_index = 0;
		continue;
        	}

        	// check store instruction
        	uint32_t num_store = 0;
        	for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        	    if (ROB.entry[ROB.head[thread_index]].destination_memory[i])
        	        num_store++;
        	}

        	if (num_store) {
        	    if ((L1D.WQ.occupancy + num_store) <= L1D.WQ.SIZE) {
        	        for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
        	            if (ROB.entry[ROB.head[thread_index]].destination_memory[i]) {

                	        PACKET data_packet;
                        	uint32_t sq_index = ROB.entry[ROB.head[thread_index]].sq_index[i];

                        	// sq_index and rob_index are no longer available after retirement
                        	// but we pass this information to avoid segmentation fault
                        	data_packet.fill_level = FILL_L1;
				data_packet.fill_l1d = 1;
                        	data_packet.cpu = cpu;
                        	data_packet.data_index = SQ.entry[sq_index].data_index;
                        	data_packet.sq_index = sq_index;

                        	//@Vishal: VIPT, send virtual address
                        	//data_packet.address = SQ.entry[sq_index].physical_address >> LOG2_BLOCK_SIZE;
                        	//data_packet.full_addr = SQ.entry[sq_index].physical_address;

                        	data_packet.address = SQ.entry[sq_index].virtual_address >> LOG2_BLOCK_SIZE;
                        	data_packet.full_addr = SQ.entry[sq_index].virtual_address;
				data_packet.full_virtual_address = SQ.entry[sq_index].virtual_address;

                        	data_packet.instr_id = SQ.entry[sq_index].instr_id;
                        	data_packet.rob_index = SQ.entry[sq_index].rob_index;
                        	data_packet.ip = SQ.entry[sq_index].ip;
                        	data_packet.type = RFO;
                        	data_packet.asid[0] = SQ.entry[sq_index].asid[0];
                        	data_packet.asid[1] = SQ.entry[sq_index].asid[1];
                        	data_packet.event_cycle = current_core_cycle[cpu];

                        	L1D.add_wq(&data_packet);
				sim_store_sent[((1 << LOG2_THREADS) - 1) & data_packet.instr_id]++;
                    	}
                	}
            }
            else {
                DP ( if (warmup_complete[cpu]) {
                	cout << "[ROB] " << __func__ << " instr_id: " << ROB.entry[ROB.head[thread_index]].instr_id << " L1D WQ is full" << endl; });

                L1D.WQ.FULL++;
                L1D.STALL[RFO]++;

                return;
            }
        }

        // release SQ entries
        for (uint32_t i=0; i<MAX_INSTR_DESTINATIONS; i++) {
            if (ROB.entry[ROB.head[thread_index]].sq_index[i] != UINT32_MAX) {
                uint32_t sq_index = ROB.entry[ROB.head[thread_index]].sq_index[i];

                //DP ( if (warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & ROB.entry[ROB.head[thread_index]].instr_id]) {
                //cout << "[SQ] " << __func__ << " instr_id: " << ROB.entry[ROB.head[thread_index]].instr_id << " releases sq_index: " << sq_index;
                //cout << hex << " address: " << (SQ.entry[sq_index].physical_address>>LOG2_BLOCK_SIZE);
                //cout << " full_addr: " << SQ.entry[sq_index].physical_address << dec << endl; });

                LSQ_ENTRY empty_entry;
                SQ.entry[sq_index] = empty_entry;
                
                SQ.occupancy--;
                SQ.head++;
                if (SQ.head == SQ.SIZE)
                    SQ.head = 0;
            }
        }

        // release ROB entry
        //DP ( if (warmup_complete[cpu][thread_index]) {
        //cout << "[ROB] " << __func__ << " instr_id: " << (ROB.entry[ROB.head[thread_index]].instr_id >> 3)  << " is retired" << endl; });
 	#if  defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
        if(ROB.entry[ROB.head[thread_index]].load_stall_flag && ROB.entry[ROB.head[thread_index]].dtlb_end_stall_flag) //this will be set only for loads which stalled at ROB's head and only after warmup_complete
        {
                assert(warmup_complete[cpu][thread_index]);
                int load_rob_stall_cycles = (ROB.entry[ROB.head[thread_index]].dtlb_stall_end_cycle - ROB.entry[ROB.head[thread_index]].stall_begin_cycle);
                total_load_rob_stall_cycles[cpu][thread_index] += load_rob_stall_cycles;

		#ifdef DETECT_CRITICAL_TRANSLATIONS
		auto it = per_crit_trans_stall[thread_index].find(translation_stalled[cpu][thread_index]);
		if (it == per_crit_trans_stall[thread_index].end())
			per_crit_trans_stall[thread_index].insert({translation_stalled[cpu][thread_index], load_rob_stall_cycles});
		else
			(it->second) = (it->second) + load_rob_stall_cycles;
		#endif

		#ifdef DETECT_CRITICAL_IPS
		auto it = per_crit_ip_stall[thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
		if (it == per_crit_ip_stall[thread_index].end())
			per_crit_ip_stall[thread_index].insert({ROB.entry[ROB.head[thread_index]].ip, load_rob_stall_cycles});
		else
			(it->second) = (it->second) + load_rob_stall_cycles;
		#endif

            if(load_rob_stall_cycles > 0 && load_rob_stall_cycles <=10)
			{
			load_rob_stall_prob_distri[cpu][thread_index][0]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
			//critical_trans_0_10[cpu][knob_cloudsuite?0:thread_index].insert(translation_stalled[cpu][thread_index]);
		}
		else if(load_rob_stall_cycles > 10 && load_rob_stall_cycles <=50)
		{
                        load_rob_stall_prob_distri[cpu][thread_index][1]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_10_50[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
			
			//critical_trans_10_40[cpu][knob_cloudsuite?0:thread_index].insert(translation_stalled[cpu][thread_index]);
		}
		else if(load_rob_stall_cycles > 50 && load_rob_stall_cycles <=100)
		{
			load_rob_stall_prob_distri[cpu][thread_index][2]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_50_100[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
			//critical_trans_40_100[cpu][knob_cloudsuite?0:thread_index].insert(translation_stalled[cpu][thread_index]);
		}
		else if(load_rob_stall_cycles > 100 && load_rob_stall_cycles <=200)
		{
                        load_rob_stall_prob_distri[cpu][thread_index][3]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
                	//critical_trans_100_200[cpu][knob_cloudsuite?0:thread_index].insert(translation_stalled[cpu][thread_index]);
		}
		else if(load_rob_stall_cycles > 200 && load_rob_stall_cycles <= 500)
		{
		       	load_rob_stall_prob_distri[cpu][thread_index][4]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_200_500[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
		}
		else if(load_rob_stall_cycles > 500)
		{
                        load_rob_stall_prob_distri[cpu][thread_index][5]++;
			#ifdef DETECT_CRITICAL_TRANSLATIONS
			auto it = critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]);
			if (it == critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (translation_stalled[cpu][thread_index], 1));
			}
			else
				(it->second)++;
			#endif
			#ifdef DETECT_CRITICAL_IPS
			auto it = critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].find(ROB.entry[ROB.head[thread_index]].ip);
			if (it == critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].end())
			{
				critical_trans_500_[cpu][knob_cloudsuite?0:thread_index].insert (pair <uint64_t, uint64_t> (ROB.entry[ROB.head[thread_index]].ip, 1));
			}
			else
				(it->second)++;
			#endif
			//critical_trans_200_[cpu][knob_cloudsuite?0:thread_index].insert(translation_stalled[cpu][thread_index]);
		}
		else if (load_rob_stall_cycles != 0)
		{

			cout << " LOAD_ROB_STALL_CYCLE: " << load_rob_stall_cycles << " STALL end cycle: " << ROB.entry[ROB.head[thread_index]].dtlb_stall_end_cycle ;
			cout << " STALL begin cycle: " << ROB.entry[ROB.head[thread_index]].stall_begin_cycle << endl;
                        assert(0);
		}
		
		#ifdef DETECT_CRITICAL_IPS
                if (knob_cloudsuite && load_rob_stall_cycles > MIN_CRIT_CYCLE)
		{
			#ifdef MAX_CRIT_CYCLE
			if (load_rob_stall_cycles <= MAX_CRIT_CYCLE)
			{
			#endif
			//auto it = critical_trans_0_10[cpu][0].find (ROB.entry[ROB.head[thread_index]].ip);
			//if (it != critical_trans_0_10[cpu][0].end() &&  (it->second) >= CRIT_RECUR)
			{
				critical_ips[cpu][0].insert(ROB.entry[ROB.head[thread_index]].ip & ((1 << PARTIAL_TAG) - 1));
				crit_ip_stlb_miss[cpu][0].insert({ROB.entry[ROB.head[thread_index]].ip, 0});
			}
			#ifdef MAX_CRIT_CYCLE
			}
			#endif
		}
		else if(load_rob_stall_cycles > MIN_CRIT_CYCLE)
		{
			#ifdef MAX_CRIT_CYCLE
			if (load_rob_stall_cycles <= MAX_CRIT_CYCLE)
			{
			#endif
			//auto it = critical_trans_0_10[cpu][thread_index].find (ROB.entry[ROB.head[thread_index]].ip);
			//if (it != critical_trans_0_10[cpu][thread_index].end() &&  (it->second) >= CRIT_RECUR)
			{
				critical_ips[cpu][thread_index].insert(ROB.entry[ROB.head[thread_index]].ip & ((1 << PARTIAL_TAG) - 1));
				crit_ip_stlb_miss[cpu][thread_index].insert({ROB.entry[ROB.head[thread_index]].ip, 0});
			}
			#ifdef MAX_CRIT_CYCLE
			}
			#endif
		}
		#endif
		#ifdef DETECT_CRITICAL_TRANSLATIONS
		if(translation_stalled[cpu][thread_index] == UINT64_MAX)
			assert(0);
		if (load_rob_stall_cycles >= 100 && critical_translation[cpu][knob_cloudsuite?0:thread_index].find(translation_stalled[cpu][thread_index]) != critical_translation[cpu][knob_cloudsuite?0:thread_index].end())
		{
			//cout << "REUSED -  " << translation_stalled[cpu][thread_index] << endl;
			++reused_translation;
		}
		else
		{
			struct miss_stats Miss_stats;
			Miss_stats.occurence = 1;
			Miss_stats.miss = 1;
			freq_critical[cpu][knob_cloudsuite?0:thread_index].insert(pair <uint64_t, struct miss_stats> (translation_stalled[cpu][thread_index], Miss_stats));
		}
		if (knob_cloudsuite && load_rob_stall_cycles > MIN_CRIT_CYCLE)	// && load_rob_stall_cycles <= MAX_CRIT_CYCLE)	
		{
			#ifdef MAX_CRIT_CYCLE
			if (load_rob_stall_cycles <= MAX_CRIT_CYCLE)
			{
			#endif
			//auto it = critical_trans_0_10[cpu][0].find (translation_stalled[cpu][thread_index]);
			//if (it != critical_trans_0_10[cpu][0].end() &&  (it->second) >= CRIT_RECUR)
				critical_translation[cpu][0].insert(translation_stalled[cpu][thread_index]);
			#ifdef MAX_CRIT_CYCLE
			}
			#endif
		}
		else 	if (load_rob_stall_cycles > MIN_CRIT_CYCLE)	// && load_rob_stall_cycles <= MAX_CRIT_CYCLE)
		{
			#ifdef MAX_CRIT_CYCLE
			if (load_rob_stall_cycles <= MAX_CRIT_CYCLE)
			{
			#endif
			//auto it = critical_trans_0_10[cpu][thread_index].find (translation_stalled[cpu][thread_index]);
			//if (it != critical_trans_0_10[cpu][thread_index].end() && (it->second) >= CRIT_RECUR)
				critical_translation[cpu][thread_index].insert(translation_stalled[cpu][thread_index]);
			#ifdef MAX_CRIT_CYCLE
			}
			#endif
		}
		translation_stalled[cpu][thread_index] = UINT64_MAX;
		#endif
		
		//cout << "CRITICAL IP DETECTED : " << ROB.entry[ROB.head[thread_index]].ip << endl;
		//criticality_detector_operate(ROB.entry[ROB.head].ip, load_rob_stall_cycles, cpu);
        }
        #endif

	////if (all_warmup_complete > thread)
	////cout << "Cycle-" << current_core_cycle[cpu] << " retire rob - thread: " << thread_index << " instr_id: " << ROB.entry[ROB.head[thread_index]].instr_id << " rob_index: " << ROB.head[thread_index] << endl; 
        ooo_model_instr empty_entry;
        ROB.entry[ROB.head[thread_index]] = empty_entry;

        ROB.head[thread_index]++;
        if (ROB.head[thread_index] == ROB.partition[thread_index] + 1)
	{
		if (thread_index == 0)
			ROB.head[thread_index] = 0;
		else
			ROB.head[thread_index] = ROB.partition[thread_index - 1] + 1;
	}
        ROB.occupancy[thread_index]--;
        completed_executions--;
        num_retired[thread_index]++;

	thread_index++;
	if (thread_index == thread)
		thread_index = 0;
    }
	
    last_retired_thread = thread_index;	//(thread_index == 0 ? ooo_cpu[cpu].thread - 1: thread_index--);
}
void O3_CPU::core_final_stats()
{

	//cout << "PSCL2 stats- SETS: " << PSCL2_SET << " WAYS: " << PSCL2_WAY << endl; 
        //cout << "all_pscl2_entry.size(): " << all_pscl2_entry[cpu][0].size() << " " << all_pscl2_entry[cpu][1].size() << endl;
	//double average_rob_stall_cycles = double(total_load_rob_stall_cycles[cpu])/double(load_rob_stall_count[cpu]);
        //double fraction_of_load_rob_stalls = double(load_rob_stall_count[cpu])*100/double(rob_stall_count[cpu]);
        //cout << endl << "CPU " << cpu << " Average Load ROB stall cycles: " << average_rob_stall_cycles << endl;
        //cout << "CPU " << cpu << " Fraction of times ROB stalled by Loads: " << fraction_of_load_rob_stalls << endl;
        //cout << "CPU " << cpu << " Percentage of Load stall cycles in Total stall cycles: " << double(double(total_load_rob_stall_cycles[cpu])*100/double(total_rob_stall_cycles[cpu])) << endl;
        cout << "Total ROB stall cycles: " << total_rob_stall_cycles[cpu] << endl;
        cout << "TOTAL ROB stalled times: " << rob_stall_count[cpu] << endl;
        cout << "TOTAL Load ROB stalled times: " << load_rob_stall_count[cpu] << endl;
        cout << "Total Load ROB stalled cycles T0: " << total_load_rob_stall_cycles[cpu][0] << endl;
	cout << "Total Load ROB stalled cycles T1: " << total_load_rob_stall_cycles[cpu][1] << endl;

	#ifdef ARKA_DP_PRED
        cout << "\nDEAD PAGE PREDICTOR ACCURACY: HPCA'21" << endl;
	cout << "STLB loads correct_prediction(used): " << corrpred_stlb_load << " misprediction(unused): " << mispred_stlb_load << endl;
	cout << "STLB bypass correct_prediction(unused): " << corrpred_stlb_bypass << " misprediction(used): " << mispred_stlb_bypass << endl;
	cout << "Bypasses in STLB: " << bypass_stlb << endl;	
	#endif	
	#ifdef DETECT_CRITICAL_IPS 
        cout << "CPU " << cpu << " NUM of IPS identified critical Thread 0: " << critical_ips[cpu][0].size() << " TOTAL IPS: " << total_ips[cpu][0].size() << endl;
        cout << "CPU " << cpu << " NUM of IPS identified critical Thread 1: " << critical_ips[cpu][1].size() << " TOTAL IPS: " << total_ips[cpu][1].size() << endl;
        //cout << "CPU " << cpu << " NUM of IPS identified critical Thread 2: " << critical_ips[cpu][2].size() << " TOTAL IPC: " << total_ips[cpu][2].size() << endl;
        //cout << "CPU " << cpu << " NUM of IPS identified critical Thread 3: " << critical_ips[cpu][3].size() << " TOTAL IPC: " << total_ips[cpu][3].size() << endl;
	/*cout << "List of all critical IPs - "  << endl;
	set<uint64_t>::iterator it = critical_ips[cpu][0].begin();
        cout << "Thread 0 :" << endl;
	while (it != critical_ips[cpu][0].end())
	{
		cout << hex << (*it) << endl;
		it++;
	}

	it = critical_ips[cpu][1].begin();
        cout << "Thread 1 :" << endl;
	while (it != critical_ips[cpu][1].end())
	{
		cout << hex << (*it) << endl;
		it++;
	}
	cout << dec<< endl;*/	
	#endif
	#ifdef DETECT_CRITICAL_TRANSLATIONS
        cout << "CPU " << cpu << " TRANSLATIONS identified critical Thread 0: " << critical_translation[cpu][0].size() << " TOTAL TRANSLATIONS: " << total_translation[cpu][0].size() << endl;
        cout << "CPU " << cpu << " TRANSLATIONS identified critical Thread 1: " << critical_translation[cpu][1].size() << " TOTAL TRANSLATIONS: " << total_translation[cpu][1].size() << endl;
        //cout << "CPU " << cpu << " TRANSLATIONS identified critical Thread 2: " << critical_translation[cpu][2].size() << " TOTAL TRANSLATIONS: " << total_translation[cpu][2].size() << endl;
        //cout << "CPU " << cpu << " TRANSLATIONS identified critical Thread 3: " << critical_translation[cpu][3].size() << " TOTAL TRSNALATIONS: " << total_translation[cpu][3].size() << endl;
	#endif


	cout << "Printing reuse distance of DOA and DEAD: " << endl;
	for (int i=0; i<2; i++)
	{
		if (i==0)
			cout << "DOA ";
		else
			cout << "Dead ";
		cout << "(1-4): " << dead_reuse_1_4[i] << " (5-12): " << dead_reuse_5_12[i] << " (13-50): " << dead_reuse_13_50[i] << " (51-100): " 
			<< dead_reuse_51_100[i] << " (101-500): " << dead_reuse_101_500[i] << " (501-1000): " << dead_reuse_501_1000[i] << " (>1000): " 
		        << dead_reuse_1001[i] << endl;	
	}
	
	#ifdef MIN_CRIT_CYCLE
	cout << "MIN_CRIT_CYCLE: " << MIN_CRIT_CYCLE << endl;
	#endif
	#ifdef MAX_CRIT_CYCLE
	cout << " MAX_CRIT_CYCLE: " << MAX_CRIT_CYCLE  <<  " CRIT_RECUR: " << CRIT_RECUR << endl;	
	#endif
	cout << "List all elements of critical translation:  " << endl;
	
	set<uint64_t>::iterator itr;
       	//for (itr = critical_translation[0][0].begin(); itr != critical_translation[0][0].end(); ++itr)
	//	cout << "CRITICAL: " << hex << *itr << endl;
	
	cout << "Reused translation : "<< reused_translation << endl;	
	double average_rob_occupancy = double(total_rob_occupancy[cpu])/double(load_rob_stall_count[cpu]);
        cout << "CPU " << cpu << " Average ROB occupancy - " << average_rob_occupancy << endl;
        
	//for (int thread_index = 0; thread_index < ooo_cpu[cpu].thread; thread_index++)
	//	for(int i = 0; i < 6; i++)
          //i     		cout << "Load ROB Stall Cycles Prob Distri["<<cpu<<"]["<<thread_index<<"]["<<i<<"]: " << load_rob_stall_prob_distri[cpu][thread_index][i] << endl;

	int total = 0;

	#ifdef BYPASS_TLB
	cout << "Number of bypasses in DTLb- " << bypass_count << endl;;
	cout << "DS BYPASS_STATS size: T0 " << bypass_stats[cpu][0].size() << " T1: " << bypass_stats[cpu][1].size() << endl;
	map<uint64_t, struct evicted_stats>::iterator iter;
	for (uint16_t thread=0; thread < ooo_cpu[cpu].thread; thread++)
	{
		for (iter=bypass_stats[cpu][thread].begin(); iter!=bypass_stats[cpu][thread].end(); iter++)
		{
			cout <<"T" << thread << " IP: " <<hex<< (iter->first) << " trans: " << (iter->second).translation << " count: " << dec << (iter->second).count << endl;
		}
	}
	#endif
#ifdef DETECT_CRITICAL_IPS	
 	/*cout << "Final criticality stats-------------------------------------------------  "  <<  endl;	
	total = 0;
	uint64_t total_1_4=0, total_5_12=0, total_13_100=0, total_101_500=0, total_501_1000, total_1001=0;
	for (uint16_t thread = 0; thread < ooo_cpu[cpu].thread; thread++)
	{
		total = 0;
	        total_1_4=0, total_5_12=0, total_13_100=0, total_101_500=0, total_501_1000=0, total_1001=0;
		map <uint64_t, struct find_reuse>::iterator iter = critical_reuse[cpu][thread].begin();
	        for (iter; iter != critical_reuse[cpu][thread].end(); iter++)
		{
			total++;
			total_1_4 += (iter->second).reuse_1_4;
			total_5_12 += (iter->second).reuse_5_12;
			total_13_100 += (iter->second).reuse_13_100;
			total_101_500 += (iter->second).reuse_101_500;
			total_501_1000 += (iter->second).reuse_501_1000;
			total_1001 += (iter->second).reuse_1001;
		}
		cout << "Thread" << thread << " critical_ip accessing " << total << " translations " << endl;	
		cout << "Thread" << thread << " critical_reuse: " << total_1_4 << " " << total_5_12 << " " << total_13_100 << " " << total_101_500 << " ";
		cout << total_501_1000 << " " << total_1001 << endl;
		
		total = 0;
	        //total_1_50=0, total_50_500=0, total_500_5000=0, total_5000_50000=0, total_50000=0;
		iter = critical_reuse[cpu][thread].begin();
		for (iter; iter != critical_reuse[cpu][thread].end(); iter++)
		{
			if ((iter->second).criticalOrNot == 0)
			{
				total++;
				nc_reuse_1_50 += (iter->second).reuse_1_50;
				nc_reuse_50_500 += (iter->second).reuse_50_500;
				nc_reuse_500_5000 += (iter->second).reuse_500_5000;
				nc_reuse_5000_50000 += (iter->second).reuse_5000_50000;
				nc_reuse_50000 += (iter->second).reuse_50000;
			}
		}

		cout << "Thread" << thread << " non-critical_ip accessing " << total << " translations " << endl;;
		cout << "Thread" << thread << " non-critical_reuse: " << nc_reuse_1_50 << " " << nc_reuse_50_500 << " " << nc_reuse_500_5000 << " ";
	        cout << nc_reuse_5000_50000 << " " << nc_reuse_50000 << endl;
	}*/
#endif
	cout << "Replacement stats" << endl;
	cout << "DTLB non-critical IP: unused: " << nc[0][0] << " used: " << nc[0][1] << endl;
	cout << "STLB non-critical IP: unused: " << nc[1][0] << " used: " << nc[1][1] << endl;
	cout << "DTLB critical IP: unused: " << c[0][0] << " used: " << c[0][1] << endl;
	cout << "STLB critical IP: unused: " << c[1][0] << " used: " << c[1][1] << endl;

	#ifdef DETECT_CRITICAL_TRANSLATIONS
	/*for (uint16_t thread_num=0; thread_num < ooo_cpu[cpu].thread; thread_num++)
	{
		map <uint64_t, uint64_t>::iterator it = per_crit_trans_stall[thread_num].begin(); 
		for ( ; it != per_crit_trans_stall[thread_num].end(); it++)
			cout << "T" << thread_num << " Trans: " << (it->first) << " ROB_stall: " << (it->second) << endl;
	}*/
	#endif
	#ifdef DETECT_CRITICAL_IPS
	/*for (uint16_t thread_num=0; thread_num < ooo_cpu[cpu].thread; thread_num++)
	{
		uint64_t sum_stall = 0;
		map <uint64_t, uint64_t>::iterator it = per_crit_ip_stall[thread_num].begin(); 
		for ( ; it != per_crit_ip_stall[thread_num].end(); it++)
			cout << "T" << thread_num << " IPs: " << (it->first) << " ROB_stall: " << (it->second) << endl;

	}*/
	#endif
}
