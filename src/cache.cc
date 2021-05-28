#include "cache.h"
#include "set.h"
#include "ooo_cpu.h"
#include "uncore.h"
//#include "dp_pred.cc"
#include<vector>
#include<unordered_set>
#include<iterator>

uint64_t l2pf_access = 0;
//#define DEBUG

//#define PUSH_PREFETCHES_FROM_L2_TO_L1	//Neelu: Adding to L1 PQ after filling prefetch in L2

//#define CHECK_DATA_HIT_ON_STLB_HIT	//Neelu: Adding to check where the corresponding data is present in case of an STLB hit

//#define STLB_HINT_TO_L2_PREF

//#define NOTIFY_L1D_OF_DTLB_EVICTION

#define PREF_CLASS_MASK 0xF00 //0x1E000	//Neelu: IPCP pref class
#define NUM_OF_STRIDE_BITS 8 //13	//Neelu: IPCP stride

//Neelu: For Ideal Spatial Prefetcher
#define IDEAL_SPATIAL_REGIONS 64
vector <uint64_t> regions_accessed, total_regions;                             //Neelu: regions accessed for ideal spatial prefetcher
#define LOG2_SPATIAL_REGION_SIZE 11                             //Neelu: For 2 KB region size
uint64_t stlb_evict_crit_trans = 0;

#ifdef ARKA_DP_PRED
uint64_t mispred_stlb_load=0, corrpred_stlb_load=0, mispred_stlb_bypass=0, corrpred_stlb_bypass=0;
#endif

uint64_t dead_reuse_1_4[2] = {0};
uint64_t dead_reuse_5_12[2] = {0};
uint64_t dead_reuse_13_50[2] = {0};
uint64_t dead_reuse_51_100[2] = {0};
uint64_t dead_reuse_101_500[2] = {0};
uint64_t dead_reuse_501_1000[2] = {0};
uint64_t dead_reuse_1001[2] = {0};

//unordered_map <uint64_t, set<uint64_t>> trans_per_cycle[DTLB_SET];
vector<uint64_t> trans_per_cycle[DTLB_SET];
uint64_t total_access_count[DTLB_SET] = {0};
unordered_map <uint64_t, set<uint64_t>>::iterator Iter;
map <uint64_t, uint64_t> evict_dead_trans[MAX_THREADS][2][DTLB_SET];	//Keeps a track of DOA[0] and dead[1] translations 

//set<uint64_t> uniqt;
set <uint64_t> temp_set;
set<uint64_t>::iterator Iterator;
map<uint64_t, struct find_reuse> critical_reuse[NUM_CPUS][MAX_THREADS];
map<uint64_t, struct find_reuse> non_critical_reuse[NUM_CPUS][MAX_THREADS];
map<uint64_t, struct find_reuse> both_reuse[NUM_CPUS][MAX_THREADS];
void increment_reuse(struct find_reuse &Find_reuse, uint64_t distance);
uint64_t bypass_stlb = 0;
#ifdef BYPASS_TLB
map<uint64_t, struct evicted_stats> bypass_stats[NUM_CPUS][MAX_THREADS];
uint64_t bypass_count=0;
#endif

uint64_t dead_pred_crit_dtlb[MAX_THREADS][3];
uint64_t dead_pred_crit_stlb[MAX_THREADS][3];

map <uint64_t, struct dead_stats> doa_dtlb[MAX_THREADS];
map <uint64_t, struct dead_stats> doa_stlb[MAX_THREADS];
map <uint64_t, struct dead_stats> doa_pscl2[MAX_THREADS];

// @Vasudha: for replacement stats
uint64_t nc[2][2]={0}, c[2][2]={0}; 

void CACHE::handle_fill()
{

    // handle fill
    uint32_t fill_cpu = (MSHR.next_fill_index == MSHR_SIZE) ? NUM_CPUS : MSHR.entry[MSHR.next_fill_index].cpu;
    if (fill_cpu == NUM_CPUS)
        return;
	
    if (MSHR.next_fill_cycle <= current_core_cycle[fill_cpu]) {
	
#ifdef SANITY_CHECK
        if (MSHR.next_fill_index >= MSHR.SIZE)
            assert(0);
#endif

        uint32_t mshr_index = MSHR.next_fill_index;
	
        // find victim
        uint32_t set = get_set(MSHR.entry[mshr_index].address), way;
            way = (this->*find_victim)(fill_cpu, MSHR.entry[mshr_index].instr_id, set, block[set], MSHR.entry[mshr_index].ip, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].type);

	#ifdef ARKA_DP_PRED
	if (cache_type == IS_STLB && all_warmup_complete > ooo_cpu[0].thread)
	{
		int bypass = 0;
		bypass = lookup_pHIST(&MSHR.entry[mshr_index]);
		if (bypass == 1)
		{
			way = NUM_WAY;
			if (all_warmup_complete > ooo_cpu[0].thread)
				++bypass_stlb;
		}
	}
        #endif	    
	#ifdef BYPASS_TLB
	if (cache_type == IS_DTLB)	// && all_warmup_complete > ooo_cpu[cpu].thread)
	{
		uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id;
		map<uint64_t, struct evicted_stats>::iterator iter;
		for (iter=bypass_stats[cpu][thread_index].begin(); iter!=bypass_stats[cpu][thread_index].end(); iter++)
		{
			if((iter->second).translation == MSHR.entry[mshr_index].full_addr >> LOG2_PAGE_SIZE && (iter->second).count > 100)
			{
				way = NUM_WAY;
				if (all_warmup_complete > ooo_cpu[cpu].thread)
					bypass_count += 1;
			}
		}
	}
	#endif
#ifdef LLC_BYPASS
        if ((cache_type == IS_LLC) && (way == LLC_WAY)) { // this is a bypass that does not fill the LLC

            // update replacement policy
            (this->*update_replacement_state)(fill_cpu, set, way, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].ip, 0, MSHR.entry[mshr_index].type, 0);

            // COLLECT STATS
            sim_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
            sim_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;

#ifdef PT_STATS
     	    if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && MSHR.entry[mshr_index].type == LOAD_TRANSLATION)
	    {
		    assert(MSHR.entry[mshr_index].translation_level > 0 && MSHR.entry[mshr_index].translation_level < 6);
	            sim_pt_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].translation_level-1]++;
         	    sim_pt_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].translation_level-1]++;
	    }
#endif

	    
            // check fill level
             if (MSHR.entry[mshr_index].fill_level < fill_level) {

		     if(fill_level == FILL_L2)
		     {
			     if(MSHR.entry[mshr_index].fill_l1i)
			     {
				     upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			     }
			     if(MSHR.entry[mshr_index].fill_l1d)
			     {
				     upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			     }
		     }
		     else
		     {
			     if (MSHR.entry[mshr_index].instruction)
				     upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			     if (MSHR.entry[mshr_index].is_data)
				     upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
		     }

            }
            

	    if(warmup_complete[fill_cpu])
	      {
		uint64_t current_miss_latency = (current_core_cycle[fill_cpu] - MSHR.entry[mshr_index].cycle_enqueued);	
		total_miss_latency += current_miss_latency;

	//	sim_miss_latency[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type] += current_miss_latency;
	      }

            MSHR.remove_queue(&MSHR.entry[mshr_index]);
            MSHR.num_returned--;

            update_fill_cycle();

            return; // return here, no need to process further in this function
        }
#endif

        uint8_t  do_fill = 1;

	//Prefetch translation requests should be dropped in case of page fault
	if(cache_type == IS_ITLB || cache_type == IS_DTLB || cache_type == IS_STLB)
	{
		if (cache_type == IS_DTLB && MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D)
			assert(0);
		if(MSHR.entry[mshr_index].data == (UINT64_MAX >> LOG2_PAGE_SIZE))
		{
			do_fill = 0;	//Drop the prefetch packet
			
			// COLLECT STATS
			//@Vasudha:TODO:insert your own stats
			if (MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D)
			{
				pf_dropped++;
				//@Vasudha: if merged with prefetch_request from upper level
				if(cache_type == IS_STLB && MSHR.entry[mshr_index].fill_level == 1)
				{
					if(MSHR.entry[mshr_index].send_both_tlb)
					{
						upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
						upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
					}
					else if (MSHR.entry[mshr_index].instruction)
					    upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
					else // data
					    upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);

				}

				//Add to procesed queue to notify L1D about dropped request due to page fault
				if(cache_type == IS_STLB && MSHR.entry[mshr_index].l1_pq_index != -1 && MSHR.entry[mshr_index].fill_l1d != -1) //@Vishal: Prefetech request from L1D prefetcher
				{
					assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
					PACKET temp = MSHR.entry[mshr_index];
					temp.data_pa = MSHR.entry[mshr_index].data;
					assert(temp.l1_rq_index == -1 && temp.l1_wq_index == -1);
					temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
					temp.write_translation_merged = 0;
					if (PROCESSED.occupancy < PROCESSED.SIZE)
					       PROCESSED.add_queue(&temp);
					else
						assert(0);			
				}
				else if(cache_type == IS_STLB && MSHR.entry[mshr_index].prefetch_translation_merged) //@Vishal: Prefetech request from L1D prefetcher
				{
				       assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
				       PACKET temp = MSHR.entry[mshr_index];
				       temp.data_pa = MSHR.entry[mshr_index].data;
				       temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
				       temp.write_translation_merged = 0;
				       if (PROCESSED.occupancy < PROCESSED.SIZE)
						PROCESSED.add_queue(&temp);
				       else
				      assert(0);
				}
				else if(cache_type == IS_ITLB && (MSHR.entry[mshr_index].l1_pq_index != -1 || MSHR.entry[mshr_index].prefetch_translation_merged) && MSHR.entry[mshr_index].fill_l1i != -1)
				{
					assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
					PACKET temp = MSHR.entry[mshr_index];
					temp.instruction_pa = MSHR.entry[mshr_index].data;
					assert(temp.l1_rq_index == -1 && temp.l1_wq_index == -1);
					temp.read_translation_merged = 0;
					temp.write_translation_merged = 0;
					if(ooo_cpu[0].ITLB.PROCESSED.occupancy < ooo_cpu[0].ITLB.PROCESSED.SIZE)
						ooo_cpu[0].ITLB.PROCESSED.add_queue(&temp);
					else
						assert(0);
				}
				MSHR.remove_queue(&MSHR.entry[mshr_index]);
				MSHR.num_returned--;
				update_fill_cycle();

			}
			else if(MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION)
			{
				pf_dropped++;
				//@Vasudha:if merged with TRANSLATION_FROM_L1D
				//Add to procesed queue to notify L1D about dropped request due to page fault
				if(cache_type == IS_STLB && MSHR.entry[mshr_index].l1_pq_index != -1) //@Vishal: Prefetech request from L1D prefetcher
				{
					assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
					PACKET temp = MSHR.entry[mshr_index];
					temp.data_pa = MSHR.entry[mshr_index].data;
					assert(temp.l1_rq_index == -1 && temp.l1_wq_index == -1);
					temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
					temp.write_translation_merged = 0;
					if (PROCESSED.occupancy < PROCESSED.SIZE)
					       PROCESSED.add_queue(&temp);
					else
						assert(0);			
				}
				else if(cache_type == IS_STLB && MSHR.entry[mshr_index].prefetch_translation_merged) //@Vishal: Prefetech request from L1D prefetcher
				{
					assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
				       PACKET temp = MSHR.entry[mshr_index];
				       temp.data_pa = MSHR.entry[mshr_index].data;
				       temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
				       temp.write_translation_merged = 0;
				       if (PROCESSED.occupancy < PROCESSED.SIZE)
						PROCESSED.add_queue(&temp);
				       else
				      assert(0);
				}
				else if(cache_type == IS_ITLB && (MSHR.entry[mshr_index].l1_pq_index != -1 || MSHR.entry[mshr_index].prefetch_translation_merged) && MSHR.entry[mshr_index].fill_l1i != -1)
				{
					assert(MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D);
					PACKET temp = MSHR.entry[mshr_index];
					temp.instruction_pa = MSHR.entry[mshr_index].data;
					assert(temp.l1_rq_index == -1 && temp.l1_wq_index == -1);
					temp.read_translation_merged = 0;
					temp.write_translation_merged = 0;
					if(ooo_cpu[0].ITLB.PROCESSED.occupancy < ooo_cpu[0].ITLB.PROCESSED.SIZE)
						ooo_cpu[0].ITLB.PROCESSED.add_queue(&temp);
					else
						assert(0);
				}
				// check fill level
				if (MSHR.entry[mshr_index].fill_level < fill_level) {

					if(cache_type == IS_STLB)
					{
						if(MSHR.entry[mshr_index].send_both_tlb)
						{
							upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
							upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
						}
						else if (MSHR.entry[mshr_index].instruction)
						    upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
						else // data
						    upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
					}
					else
                                        {
                                                if (MSHR.entry[mshr_index].instruction)
                                                    upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
                                                else // data
                                                    upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
                                        }
				}


				MSHR.remove_queue(&MSHR.entry[mshr_index]);
				MSHR.num_returned--;
				update_fill_cycle();

			}
			else
			{
				//When prefetch translation gets merged with demand request (load)
				//Case 1 : STLB prefetch request merged (from prefetch queue) - mark MSHR inflight and add to PTW.RQ
				//Case 2 : Translation of L1D prefetch request merged (from read queue) - mark MSHR inflight and add to DTLB RQ
				
				if(MSHR.entry[mshr_index].l1_pq_index != -1 || MSHR.entry[mshr_index].prefetch_translation_merged)	//TRANSALTION_FROM_L1D merged with LOAD/RFO
				{
					//MSHR.entry[mshr_index].returned = INFLIGHT;
					//ooo_cpu[fill_cpu].DTLB.add_rq(&MSHR.entry[mshr_index]);
					//MSHR.remove_queue(&MSHR.entry[mshr_index]);
					//MSHR.num_returned--;
					MSHR.entry[mshr_index].returned = INFLIGHT;
					if(lower_level)
						lower_level->add_rq(&MSHR.entry[mshr_index]);
					update_fill_cycle();
				}
				else	//PREFETCH merged with LOAD/RFO
				{
					MSHR.entry[mshr_index].returned = INFLIGHT;
					////cout << "PREFETCH_TRANSLATION merged with LOAD_TRANSLATION-instr_id: "<<MSHR.entry[mshr_index].instr_id<<" prefetch_id: "<<MSHR.entry[mshr_index].prefetch_id;
					////cout << " address: " << hex << MSHR.entry[mshr_index].address << " full_addr: "<< MSHR.entry[mshr_index].full_addr << " data: ";
					////cout << MSHR.entry[mshr_index].data << " type: " << dec << MSHR.entry[mshr_index].type << " event_cycle: " << MSHR.entry[mshr_index].event_cycle << endl; 
					if(lower_level)
						lower_level->add_rq(&MSHR.entry[mshr_index]);
					update_fill_cycle();
				}
			}


		
		}
	}	
        // is this dirty?
        if (block[set][way].dirty) {

            // check if the lower level WQ has enough room to keep this writeback request
            if (lower_level) {
                if (lower_level->get_occupancy(2, block[set][way].address) == lower_level->get_size(2, block[set][way].address)) {

                    // lower level WQ is full, cannot replace this victim
                    do_fill = 0;
                    lower_level->increment_WQ_FULL(block[set][way].address);
                    STALL[MSHR.entry[mshr_index].type]++;

                   //DP ( if (warmup_complete[fill_cpu] ) {
                    //cout << "[" << NAME << "] " << __func__ << "do_fill: " << +do_fill;
                    //cout << " lower level wq is full!" << " fill_addr: " << hex << MSHR.entry[mshr_index].address;
                    //cout << " victim_addr: " << block[set][way].tag << dec << endl; });
                }
                else {
                    PACKET writeback_packet;

                    writeback_packet.fill_level = fill_level << 1;
                    writeback_packet.cpu = fill_cpu;
                    writeback_packet.address = block[set][way].address;
                    writeback_packet.full_addr = block[set][way].full_addr;
                    writeback_packet.data = block[set][way].data;
                    writeback_packet.instr_id = MSHR.entry[mshr_index].instr_id;
                    writeback_packet.ip = 0; // writeback does not have ip
                    writeback_packet.type = WRITEBACK;
                    writeback_packet.event_cycle = current_core_cycle[fill_cpu];

                    lower_level->add_wq(&writeback_packet);
                }
            }
#ifdef SANITY_CHECK
            else {
                // sanity check
                if (cache_type != IS_STLB)
                    assert(0);
            }
#endif
        }

        if (do_fill){

		#ifdef ARKA_DP_PRED
		if (cache_type == IS_STLB && way != NUM_WAY && block[set][way].valid == 1 && all_warmup_complete > ooo_cpu[fill_cpu].thread)
		{
			modify_pHIST(&block[set][way]);
			if(all_warmup_complete > ooo_cpu[fill_cpu].thread)
			{
				if (block[set][way].used == 0)
					mispred_stlb_load++;
				else
					corrpred_stlb_load++;
			}
		}
		#endif

		if (cache_type == IS_PSCL2 && all_warmup_complete > ooo_cpu[fill_cpu].thread && block[set][way].valid == 1)
		{
			
			uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
		   	if (knob_cloudsuite)
				thread_index = 0;
			/*auto it = doa_pscl2[thread_index].find(block[set][way].address);
			if (it == doa_pscl2[thread_index].end() && block[set][way].used == 0)
			{
				struct dead_stats Dead_stats;
				Dead_stats.dead = 0;
				Dead_stats.used = 0;
				Dead_stats.doa_crit = 0;
				Dead_stats.doa_non_crit = 0;
				Dead_stats.doa= 1;
				if (block[set][way].critical)
					Dead_stats.doa_crit = 1;
				else
					Dead_stats.doa_non_crit = 1;
				doa_pscl2[thread_index].insert({block[set][way].address, Dead_stats});
			}
			else
			{
				if (block[set][way].used == 0)
				{
					(it->second).doa++;
					if (block[set][way].critical)
						(it->second).doa_crit++;
					else
						(it->second).doa_non_crit++;

				}
				else if (block[set][way].used == 1)
					(it->second).dead++;
				else if (block[set][way].used == 2)
					(it->second).used++;
			}*/
		}

		//@Vasudha: DOA entries
		if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[fill_cpu].thread && block[set][way].valid == 1)
		{
			
			uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
		   	if (knob_cloudsuite)
				thread_index = 0;

			if (block[set][way].critical)
				dead_pred_crit_dtlb[thread_index][block[set][way].used]++;
			
			/*{
			auto it = evict_dead_trans[thread_index][0][set].find(block[set][way].address);
			if (it == evict_dead_trans[thread_index][0][set].end())
			{
				if (block[set][way].used == 0)
					//evict_dead_trans[thread_index][0][set].insert({block[set][way].full_addr >> LOG2_PAGE_SIZE, current_core_cycle[cpu]});
					evict_dead_trans[thread_index][0][set].insert({block[set][way].address, total_access_count[set]});
			}
			else
			{
				cout << "Cycle: " << current_core_cycle[cpu] << " evicteD: " << block[set][way].address;
				cout << " Prev_cycle: " << (it->second) << " it->first: " << (it->first) << endl;
				assert(0);
			}
			}
			{
			auto it = evict_dead_trans[thread_index][1][set].find(block[set][way].address);
			if (it == evict_dead_trans[thread_index][1][set].end())
			{
				if (block[set][way].used == 1)
					//evict_dead_trans[thread_index][1][set].insert({block[set][way].full_addr >> LOG2_PAGE_SIZE, current_core_cycle[cpu]});
					evict_dead_trans[thread_index][1][set].insert({block[set][way].address, total_access_count[set]});

			}
			else
			{
				cout << "Cycle: " << current_core_cycle[cpu] << " evicteD: " << (block[set][way].address);
				cout << " Prev_cycle: " << (it->second) << " it->first: " << (it->first) << " address: " << block[set][way].address << endl;
				assert(0);
			}
			}*/
			
			auto it = doa_dtlb[thread_index].find(block[set][way].full_addr >> LOG2_PAGE_SIZE);
			if (it == doa_dtlb[thread_index].end())
			{
				struct dead_stats Dead_stats;
				Dead_stats.doa = 0;
				Dead_stats.dead = 0;
				Dead_stats.used = 0;
				if (block[set][way].used == 0)	
					Dead_stats.doa++;
				else if (block[set][way].used == 1)
					Dead_stats.dead++;
				else if (block[set][way].used == 2)
					Dead_stats.used++;
				if (block[set][way].critical)
					Dead_stats.critical = 1;
				else
					Dead_stats.critical = 0;
				doa_dtlb[thread_index].insert({block[set][way].full_addr >> LOG2_PAGE_SIZE, Dead_stats});
			}
			else
			{
				if (block[set][way].used == 0)
					(it->second).doa++;
				else if (block[set][way].used == 1)
					(it->second).dead++;
				else if (block[set][way].used == 2)
					(it->second).used++;
				if (block[set][way].critical == 1 && (it->second).critical == 0)
					(it->second).critical = 1;
				else if (block[set][way].critical == 0 && (it->second).critical == 1)
					(it->second).critical = 2;

				//(it->second)++;
			}
		}
		if (cache_type == IS_STLB && all_warmup_complete > ooo_cpu[fill_cpu].thread && block[set][way].valid == 1)
		{
			
				uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
				if (knob_cloudsuite)
					thread_index = 0;

				if (block[set][way].critical)
					dead_pred_crit_stlb[thread_index][block[set][way].used]++;

				auto it = doa_stlb[thread_index].find(block[set][way].full_addr >> LOG2_PAGE_SIZE);
				if (it == doa_stlb[thread_index].end())
				{
					struct dead_stats Dead_stats;
					Dead_stats.doa = 0;
					Dead_stats.dead = 0;
					Dead_stats.used = 0;
					if (block[set][way].used == 0)
						Dead_stats.doa++;
					else if (block[set][way].used == 1)
						Dead_stats.dead++;
					else if (block[set][way].used == 2)
						Dead_stats.used++;
					if (block[set][way].critical)
						Dead_stats.critical = 1;
					else
						Dead_stats.critical = 0;
					doa_stlb[thread_index].insert({block[set][way].full_addr >> LOG2_PAGE_SIZE, Dead_stats});
				}
				else
				{
					if (block[set][way].used == 0)
						(it->second).doa++;
					else if (block[set][way].used == 1)
						(it->second).dead++;
					else if (block[set][way].used == 2)
						(it->second).used++;
					if (block[set][way].critical == 1 && (it->second).critical == 0)
						(it->second).critical = 1;
					else if (block[set][way].critical == 0 && (it->second).critical == 1)
						(it->second).critical = 2;
				}
		}

		if (all_warmup_complete > ooo_cpu[fill_cpu].thread && (cache_type == IS_DTLB || cache_type == IS_STLB))
		{
			if (block[set][way].critical == 0)
			{
				if (block[set][way].used == 0)
					nc[cache_type-1][0]++;
				else
					nc[cache_type-1][1]++;
			}
			else 
			{
				if (block[set][way].used == 0)
					c[cache_type-1][0]++;
				else 
					c[cache_type-1][1]++;
			}
		}
	    #ifdef BYPASS_TLB
	    // If translation in DTLB doesn't get reused, then store that IP and translation
	    if (cache_type == IS_DTLB && block[set][way].used == 0 && way != NUM_WAY)	// && all_warmup_complete > ooo_cpu[cpu].thread)
	    {
		    uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
		    if(knob_cloudsuite)
			    thread_index = 0;
		    int found = 0;
		    if (bypass_stats[cpu][thread_index].find(block[set][way].ip) != bypass_stats[cpu][thread_index].end())
		    {
			    map<uint64_t, struct evicted_stats>::iterator itr = bypass_stats[cpu][thread_index].begin();
			    for (itr; itr != bypass_stats[cpu][thread_index].end(); itr++)
			    {
				    if ((itr->second).translation == block[set][way].full_addr >> LOG2_PAGE_SIZE)
				    {
					found = 1;
					(itr->second).count++;
					break;
				    }
			    }
			    if (found == 0)
			    {
				    struct evicted_stats Evicted_stats;
				    Evicted_stats.translation = block[set][way].full_addr >> LOG2_PAGE_SIZE;
				    Evicted_stats.count = 1;
				    bypass_stats[cpu][thread_index].insert (pair<uint64_t, struct evicted_stats> (block[set][way].ip, Evicted_stats)); 
			    }
		    } 
		    else
		    {
		    	struct evicted_stats Evicted_stats;
			Evicted_stats.translation = block[set][way].full_addr >> LOG2_PAGE_SIZE;
			Evicted_stats.count = 1;
			bypass_stats[cpu][thread_index].insert (pair<uint64_t, struct evicted_stats> (block[set][way].ip, Evicted_stats));
		    }
	    }
	    else if (cache_type == IS_DTLB && block[set][way].used == 1 && way != NUM_WAY)
	    {
		    uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
		    if(knob_cloudsuite)
			    thread_index = 0;
		    int found = 0;
		    if (bypass_stats[cpu][thread_index].find(block[set][way].ip) != bypass_stats[cpu][thread_index].end())
		    {
			    map<uint64_t, struct evicted_stats>::iterator itr = bypass_stats[cpu][thread_index].begin();
			    for (itr; itr != bypass_stats[cpu][thread_index].end(); itr++)
			    {
				    if ((itr->second).translation == block[set][way].full_addr >> LOG2_PAGE_SIZE)
				    {
					(itr->second).count--;
					break;
				    }
			    }
		    } 
	    }
	    #endif

	    #ifdef DEBUG_PREF
	    if (cache_type == IS_STLB && all_warmup_complete > ooo_cpu[fill_cpu].thread){
		    cout << "Cycle: " << dec << current_core_cycle[cpu] << " evicted address: " << hex << block[set][way].address << " loaded address: " ;
	    	    cout << MSHR.entry[mshr_index].address << dec << endl;
	    }
	    #endif

	    #ifdef CRITICAL_TLB
	    if (cache_type == IS_DTLB && way == NUM_WAY)
		    ;
	    else
	    #endif
	    
	    //@Vasudha: For PC-offset DTLB prefetcher, in case of eviction, transfer block from training table to trained table
	    if(cache_type == IS_DTLB && block[set][way].valid==1 && way != NUM_WAY)
	    {
	            #ifdef PUSH_DTLB_PB
		    if (MSHR.entry[mshr_index].type != PREFETCH_TRANSLATION)
		    {
		    #endif
		    #ifndef PUSH_DTLB_UB
			    uint64_t enter_VB;
			    enter_VB = dtlb_prefetcher_cache_fill(MSHR.entry[mshr_index].address, set, way, (MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION) ? 1 : 0, block[set][way].address, 
					    MSHR.entry[mshr_index].pf_metadata, ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id);
			    #ifdef PUSH_VICTIMS_DTLB_VB
			    if (enter_VB == 1)
			    {	    
				uint32_t victim_way;
				victim_way = (&ooo_cpu[fill_cpu].DTLB_VB->*find_victim)(cpu, 0, 0, ooo_cpu[fill_cpu].DTLB_VB.block[0], 0, block[set][way].full_addr, 0);
				#ifdef DEBUG
				//if (victim_way >= 0 && ooo_cpu[cpu].DTLB_VB.block[0][victim_way].used != 1)
				//	cout << "DTLB_VB block - NOT USED: " << hex << ooo_cpu[cpu].DTLB_VB.block[0][victim_way].address << " Cycle: " << dec << current_core_cycle[cpu] << endl;
				#endif
				if (victim_way >= 0)
					dtlb_victim_buffer_fill(block[set][way].address, 0, victim_way, block[set][way].prefetch, ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].address, (uint32_t)ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].used, ((1 << LOG2_THREADS) - 1) & ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].instr_id );
				#ifdef DEBUG
				if (victim_way >= 0)
					cout << "Cycle: " <<dec<<current_core_cycle[cpu] << " T" << (((1<<LOG2_THREADS)-1) & ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].instr_id) <<
						" evicted_addr:from VB " <<hex<< ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].address << " evicted_region: " <<
						(ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].address >> 13) << endl;
				#endif
				(&ooo_cpu[fill_cpu].DTLB_VB->*update_replacement_state)(cpu, 0, victim_way, block[set][way].full_addr, 0, ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].full_addr, 0, 0);
				PACKET evicted_block;
				evicted_block.instr_id = block[set][way].instr_id;
				evicted_block.address = block[set][way].address;
				evicted_block.full_addr = block[set][way].full_addr;
				evicted_block.data = block[set][way].data;
				evicted_block.ip = block[set][way].ip;
				evicted_block.cpu = block[set][way].cpu;
				ooo_cpu[fill_cpu].DTLB_VB.fill_cache(0, victim_way, &evicted_block);
			    }
			    #ifdef DETECT_CRITICAL_TRANSLATIONS
			    uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id;
			    if(knob_cloudsuite)
				    thread_index = 0;
			    if (critical_translation[cpu][thread_index].find(block[set][way].address) != critical_translation[cpu][thread_index].end())
			    {
				    auto it = freq_critical[cpu][thread_index].find(block[set][way].address);
				    if (it != freq_critical[cpu][thread_index].end())
				    {
					    if ((it->second).occurence > 75 && (it->second).miss >50)
					    {
						    uint32_t victim_way =  (&ooo_cpu[fill_cpu].DTLB_VB->*find_victim)(cpu, 0, 0, ooo_cpu[fill_cpu].DTLB_VB.block[0], 0, block[set][way].full_addr, 0);
						    (&ooo_cpu[fill_cpu].DTLB_VB->*update_replacement_state)(cpu, 0, victim_way, block[set][way].full_addr, 0, ooo_cpu[fill_cpu].DTLB_VB.block[0][victim_way].full_addr, 0, 0);
						     PACKET evicted_block;
						     evicted_block.instr_id = block[set][way].instr_id;
						     evicted_block.address = block[set][way].address;
							evicted_block.full_addr = block[set][way].full_addr;
							evicted_block.data = block[set][way].data;
						evicted_block.ip = block[set][way].ip;
						evicted_block.cpu = block[set][way].cpu;
						ooo_cpu[fill_cpu].DTLB_VB.fill_cache(0, victim_way, &evicted_block);

					    }
				    }
				    else
					    assert(0);
			    }
			    #endif
		            #endif
		    #endif
		    #ifdef PUSH_DTLB_PB
		    }
		    #endif 
	    }
            
	    // update prefetcher
	    if (cache_type == IS_L1I)
                l1i_prefetcher_cache_fill(fill_cpu, ((MSHR.entry[mshr_index].ip)>>LOG2_BLOCK_SIZE)<<LOG2_BLOCK_SIZE, set, way, (MSHR.entry[mshr_index].type == PREFETCH) ? 1 : 0, ((block[set][way].ip)>>LOG2_BLOCK_SIZE)<<LOG2_BLOCK_SIZE);
	    if (cache_type == IS_L1D)
	    {
		    uint64_t v_fill_addr, v_evicted_addr;
		    map <uint64_t, uint64_t>::iterator ppage_check = ooo_cpu[cpu].PTW.inverse_table.find(MSHR.entry[mshr_index].full_addr >> LOG2_PAGE_SIZE);
		    //assert(ppage_check != ooo_cpu[cpu].PTW.inverse_table.end());
		    v_fill_addr = (ppage_check->second) << LOG2_PAGE_SIZE;
		    v_fill_addr |= (MSHR.entry[mshr_index].full_addr & ((1 << LOG2_PAGE_SIZE)-1));

		    //Now getting virtual address for the evicted address
		    /*Neelu: Note that it is not always necessary that evicted address is a valid address and is present in the inverse table, hence (1) do not use the assert and (2) if it is not present, assign it to zero. */


		    ppage_check = ooo_cpu[cpu].PTW.inverse_table.find(block[set][way].address >> (LOG2_PAGE_SIZE - LOG2_BLOCK_SIZE));
		    if(ppage_check != ooo_cpu[cpu].PTW.inverse_table.end())
		    {
			    v_evicted_addr = (ppage_check->second) << LOG2_PAGE_SIZE;
			    v_evicted_addr |= ((block[set][way].address << LOG2_BLOCK_SIZE) & ((1 << LOG2_PAGE_SIZE)-1));
		    }
		    else
			    v_evicted_addr = 0;

		    l1d_prefetcher_cache_fill(v_fill_addr, MSHR.entry[mshr_index].full_addr, set, way, (MSHR.entry[mshr_index].type == PREFETCH) ? 1 : 0, v_evicted_addr, block[set][way].address<<LOG2_BLOCK_SIZE, MSHR.entry[mshr_index].pf_metadata);

	    }
	    if  (cache_type == IS_L2C)
	      MSHR.entry[mshr_index].pf_metadata = l2c_prefetcher_cache_fill(MSHR.entry[mshr_index].address<<LOG2_BLOCK_SIZE, set, way, (MSHR.entry[mshr_index].type == PREFETCH) ? 1 : 0,
									     block[set][way].address<<LOG2_BLOCK_SIZE, MSHR.entry[mshr_index].pf_metadata);
            if (cache_type == IS_LLC)
	      {
		cpu = fill_cpu;
		MSHR.entry[mshr_index].pf_metadata = llc_prefetcher_cache_fill(MSHR.entry[mshr_index].address<<LOG2_BLOCK_SIZE, set, way, (MSHR.entry[mshr_index].type == PREFETCH) ? 1 : 0,
									       block[set][way].address<<LOG2_BLOCK_SIZE, MSHR.entry[mshr_index].pf_metadata);
		cpu = 0;
	      }

#ifdef NOTIFY_L1D_OF_DTLB_EVICTION
              //Neelu: Sending DTLB eviction notice to L1D
              if(cache_type == IS_DTLB)
	      {
		      ooo_cpu[fill_cpu].L1D.l1d_prefetcher_notify_about_dtlb_eviction(MSHR.entry[mshr_index].address<<LOG2_PAGE_SIZE, set, way, 0, block[set][way].address<<LOG2_PAGE_SIZE, MSHR.entry[mshr_index].pf_metadata);
	      }
#endif
            
	    
	    if (way != NUM_WAY) 
            // update replacement policy
                (this->*update_replacement_state)(fill_cpu, set, way, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].ip, block[set][way].full_addr, MSHR.entry[mshr_index].type, 0);
#ifdef IDEAL_CACHE_FOR_TRANSLATION_ACCESS
#ifdef IDEAL_L1D
            if (cache_type == IS_L1D && 			    
 	        (MSHR.entry[mshr_index].type == LOAD_TRANSLATION || MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) != unique_translation_access.end() )
#endif

#ifdef IDEAL_L2
            if (cache_type == IS_L1D &&
                (MSHR.entry[mshr_index].type == LOAD_TRANSLATION || MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) != unique_translation_access.end() )
#endif

#ifdef IDEAL_L3
            if (cache_type == IS_LLC &&
                (MSHR.entry[mshr_index].type == LOAD_TRANSLATION || MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) != unique_translation_access.end() )
#endif	    
            {
            }
	    else
            {
                    // COLLECT STATS
                    sim_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
                    sim_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
            }
#else
            // COLLECT STATS
            sim_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
            sim_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
#endif

#ifdef PT_STATS
	    if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && MSHR.entry[mshr_index].type == LOAD_TRANSLATION)
	    {
		    assert(MSHR.entry[mshr_index].translation_level > 0 && MSHR.entry[mshr_index].translation_level < 6);
        	    sim_pt_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].translation_level-1]++;
                    sim_pt_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].translation_level-1]++;
	    }
#endif
	    


           /*
	    // COLLECT STATS
            sim_miss[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
            sim_access[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type]++;
	   */

	    //Neelu: IPCP stats collection
	    if(cache_type == IS_L1D)
	    {
		    if(MSHR.entry[mshr_index].late_pref == 1)
		    {
			    int temp_pf_class = (MSHR.entry[mshr_index].pf_metadata & PREF_CLASS_MASK) >> NUM_OF_STRIDE_BITS;
			    if(temp_pf_class < 5)
			    {
				    pref_late[cpu][((MSHR.entry[mshr_index].pf_metadata & PREF_CLASS_MASK) >> NUM_OF_STRIDE_BITS)]++;
			    }
		    }
	     }


		uint32_t victim_way_ub = -1;
	#ifdef PUSH_DTLB_PB
	    if ( (cache_type!=IS_DTLB) || (cache_type==IS_DTLB && MSHR.entry[mshr_index].type != PREFETCH_TRANSLATION) )
	    {
	#endif
	#ifndef PUSH_DTLB_UB	    
		uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id;
		if (knob_cloudsuite)
			thread_index = 0;	
		//if ((cache_type == IS_DTLB || (cache_type == IS_STLB && MSHR.entry[mshr_index].instruction == 0)) && critical_translation[cpu][thread_index].size() > 100)
		/*if (cache_type == IS_DTLB && critical_translation[cpu][thread_index].size() > 100)
		{
			#ifdef CRITICAL_TLB
	    		if (critical_translation[cpu][thread_index].find(MSHR.entry[mshr_index].full_virtual_address >> LOG2_PAGE_SIZE) != critical_translation[cpu][thread_index].end()) 
	    		{
				fill_cache(set, way, &MSHR.entry[mshr_index]);
	    		}
			#else
				fill_cache(set, way, &MSHR.entry[mshr_index]);
			#endif
		}
		    
		else*/
		
		//@Vasudha: To Bypass DTLB
		//if (!(cache_type == IS_DTLB && MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION))
		
		if (way != NUM_WAY)
		    fill_cache(set, way, &MSHR.entry[mshr_index]);
		#ifdef DEBUG_PREF
		else if (all_warmup_complete > 2)
			cout << "Cycle: " << dec << current_core_cycle[cpu] << " address_bypassed: " << hex << MSHR.entry[mshr_index].address << dec << endl;
		#endif
	#else
	    //fill in DTLB_UTILITY_BUFFER
	    if(cache_type == IS_DTLB)
	    {
		victim_way_ub = (&ooo_cpu[fill_cpu].DTLB_UB->*find_victim) (fill_cpu, MSHR.entry[mshr_index].instr_id, 0, ooo_cpu[fill_cpu].DTLB_UB.block[0], MSHR.entry[mshr_index].ip, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].type);
		(&ooo_cpu[fill_cpu].DTLB_UB->*update_replacement_state) (fill_cpu, 0, victim_way_ub, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].ip, ooo_cpu[fill_cpu].DTLB_UB.block[0][victim_way_ub].full_addr, MSHR.entry[mshr_index].type, 0);
		    ooo_cpu[fill_cpu].DTLB_UB.fill_cache(0, victim_way_ub, &MSHR.entry[mshr_index]);
	    }
	    else
	    	fill_cache(set, way, &MSHR.entry[mshr_index]);
	#endif
	#ifdef PUSH_DTLB_PB
	    }
	    else if (cache_type == IS_DTLB && MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION)
	    {
		    uint32_t victim_way;
		     victim_way = (&ooo_cpu[fill_cpu].DTLB_PB->*find_victim)( fill_cpu, MSHR.entry[mshr_index].instr_id, 0, ooo_cpu[fill_cpu].DTLB_PB.block[0] , MSHR.entry[mshr_index].ip, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].type);
                        (&ooo_cpu[fill_cpu].DTLB_PB->*update_replacement_state)(fill_cpu, 0, victim_way, MSHR.entry[mshr_index].full_addr, MSHR.entry[mshr_index].ip, ooo_cpu[fill_cpu].DTLB_PB.block[0][victim_way].full_addr, MSHR.entry[mshr_index].type, 0);
                        ooo_cpu[fill_cpu].DTLB_PB.fill_cache( 0, victim_way, &MSHR.entry[mshr_index] );
	    }
	#endif
            // RFO marks cache line dirty
            if (cache_type == IS_L1D) {
                if (MSHR.entry[mshr_index].type == RFO)
                    block[set][way].dirty = 1;
            }

	    	//Neelu: Adding condition to ensure that STLB does not insert instruction translations to Processed queue.
		if(cache_type == IS_STLB && MSHR.entry[mshr_index].l1_pq_index != -1 && (MSHR.entry[mshr_index].send_both_tlb or !MSHR.entry[mshr_index].instruction)) //@Vishal: Prefetech request from L1D prefetcher
		{

			PACKET temp = MSHR.entry[mshr_index];
			temp.data_pa = block[set][way].data;
			assert(temp.l1_rq_index == -1 && temp.l1_wq_index == -1);
			temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
			temp.write_translation_merged = 0;
                if (PROCESSED.occupancy < PROCESSED.SIZE)
                    PROCESSED.add_queue(&temp);
				else
					assert(0);			
		}
		else if(cache_type == IS_STLB && MSHR.entry[mshr_index].prefetch_translation_merged) //@Vishal: Prefetech request from L1D prefetcher
        {
            PACKET temp = MSHR.entry[mshr_index];
            temp.data_pa = block[set][way].data;
            temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
            temp.write_translation_merged = 0;
                if (PROCESSED.occupancy < PROCESSED.SIZE)
                    PROCESSED.add_queue(&temp);
                else
                    assert(0);
        }

//Neelu: Invoking the L2C prefetcher on STLB fills



            // check fill level
            if (MSHR.entry[mshr_index].fill_level < fill_level) {

		if(cache_type == IS_STLB)
		{
			MSHR.entry[mshr_index].prefetch_translation_merged = 0;

			if(MSHR.entry[mshr_index].send_both_tlb)
			{
				upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
				upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			}
			else if (MSHR.entry[mshr_index].instruction)
			    upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			else // data
			#if defined(PERFECT_STLB_EXCLUDING_COLD_MISSES) || defined(PERFECT_STLB)
			#if defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
			if (MSHR.entry[mshr_index].sent == 0)
			#endif	
			#endif
			    upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
		}

		//@Rahul: PTW
#ifdef PTW_L1D
		else if(cache_type == IS_L1D &&
			(MSHR.entry[mshr_index].type == LOAD_TRANSLATION ||
			 MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION ||
			 MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D))
		{


#ifdef IDEAL_L1D
       		        if (unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) == unique_translation_access.end())
			{
              			PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
             			unique_translation_access.insert(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE);
                        }
#else
            		PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
#endif
		}
#endif


#ifdef PTW_L2C
		else if(cache_type == IS_L2C &&
			(MSHR.entry[mshr_index].type == LOAD_TRANSLATION ||
			 MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION ||
			 MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D))
		{
#ifdef IDEAL_L2
                        if (unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) == unique_translation_access.end())
                        {
                                PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
                                unique_translation_access.insert(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE);
                        }
#else
                        PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
#endif
		}
#endif

#ifdef PTW_LLC
		else if(cache_type == IS_LLC &&
			(MSHR.entry[mshr_index].type == LOAD_TRANSLATION ||
			 MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION ||
			 MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D))
		{
#ifdef IDEAL_L3
                        if (unique_translation_access.find(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE) == unique_translation_access.end())
                        {
                                PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
                                unique_translation_access.insert(MSHR.entry[mshr_index].full_addr >> LOG2_BLOCK_SIZE);
                        }
#else
                        PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
#endif
                }
#endif

#ifdef PTW_L1D_L2C
               else if((MSHR.entry[mshr_index].type == LOAD_TRANSLATION || MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) &&
                       ((cache_type == IS_L1D && MSHR.entry[mshr_index].translation_level > 1) || (cache_type == IS_L2C && MSHR.entry[mshr_index].translation_level == 1))) 
	       {
          		assert(MSHR.entry[mshr_index].translation_level > 0 && MSHR.entry[mshr_index].translation_level < 6);
          		PTW_interface[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
               }
#endif



		/*else if(cache_type == IS_L2C && (MSHR.entry[mshr_index].type == LOAD_TRANSLATION || MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D))
                {
                        extra_interface->return_data(&MSHR.entry[mshr_index]);
		}*/

		else if(cache_type == IS_L2C)
		{
			if(MSHR.entry[mshr_index].send_both_cache)
			{
		                upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
		                upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
		        }
			else if(MSHR.entry[mshr_index].fill_l1i || MSHR.entry[mshr_index].fill_l1d)
			{
				if(MSHR.entry[mshr_index].fill_l1i)
					upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
				if(MSHR.entry[mshr_index].fill_l1d)
					upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			}
			else if (MSHR.entry[mshr_index].instruction)
				upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			else if (MSHR.entry[mshr_index].is_data)
				upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);				           
		}
		else
		{
			if (MSHR.entry[mshr_index].instruction) 
			    upper_level_icache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
			else // data
			    upper_level_dcache[fill_cpu]->return_data(&MSHR.entry[mshr_index]);
		}
            }
            //@v if send_both_tlb == 1 in STLB, response should return to both ITLB and DTLB
            

            // update processed packets
            if ((cache_type == IS_ITLB) && (MSHR.entry[mshr_index].type != PREFETCH_TRANSLATION)) { //@v Responses to prefetch requests should not go to PROCESSED queue 
                MSHR.entry[mshr_index].instruction_pa = block[set][way].data;
                if (PROCESSED.occupancy < PROCESSED.SIZE)
                    PROCESSED.add_queue(&MSHR.entry[mshr_index]);
            }
            else if ((cache_type == IS_DTLB) && (MSHR.entry[mshr_index].type != PREFETCH_TRANSLATION)) {
             //@Vasudha: Perfect DTLB Prefetcher: commenting as in case of read miss, translation is already sent to PROCESSED QUEUE
		////uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id;
		////if (knob_cloudsuite)
		////	thread_index = 0;
	    	
		#if defined(PERFECT_DTLB)  || defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES)
		#if defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
		//if (critical_ips[cpu][thread_index].find(MSHR.entry[mshr_index].ip) != critical_ips[cpu][thread_index].end())	;
		if (MSHR.entry[mshr_index].sent == 0)
		{
		#endif
		#endif
		    #ifdef PUSH_DTLB_UB
		    	assert(victim_way_ub >= 0);
		        MSHR.entry[mshr_index].data_pa = ooo_cpu[fill_cpu].DTLB_UB.block[0][victim_way_ub].data;
		    #else 
		    	MSHR.entry[mshr_index].data_pa = MSHR.entry[mshr_index].data;	//block[set][way].data;
		    #endif
		    if (PROCESSED.occupancy < PROCESSED.SIZE)
		    {
		   	PROCESSED.add_queue(&MSHR.entry[mshr_index]);
			DP (if (all_warmup_complete > ooo_cpu[cpu].thread)
			{
				cout << "Translation sent from handle fill instr_id "<<dec<<(MSHR.entry[mshr_index].instr_id >>LOG2_THREADS) <<" address:"<<hex;
				cout << MSHR.entry[mshr_index].address << " Full_addr: " << MSHR.entry[mshr_index].full_addr << " pa-" ;
				cout << MSHR.entry[mshr_index].data_pa <<dec<< endl;
			});
		    
		    }
		
	    	#if defined(PERFECT_DTLB) || defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES)
		#if defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
		}
		#endif
		#endif
	     }
	    //@Vasudha: To Bypass DTLB
	    /*else if (cache_type == IS_DTLB && MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION)
	    {
		MSHR.entry[mshr_index].data_pa = MSHR.entry[mshr_index].data;
		if (PROCESSED.occupancy < PROCESSED.SIZE)
			PROCESSED.add_queue(&MSHR.entry[mshr_index]);
	    }*/
            else if (cache_type == IS_L1I && (MSHR.entry[mshr_index].type != PREFETCH)) {
		    if (PROCESSED.occupancy < PROCESSED.SIZE)
                    PROCESSED.add_queue(&MSHR.entry[mshr_index]);
            }
	    //@Rahul: PTW
      #if defined(PTW_L1D)||defined(PTW_L1D_L2C)      
      	    else if ((cache_type == IS_L1D) && ((MSHR.entry[mshr_index].type != PREFETCH) && (MSHR.entry[mshr_index].type != LOAD_TRANSLATION)))
      #else
            else if ((cache_type == IS_L1D) && (MSHR.entry[mshr_index].type != PREFETCH))
      #endif
            {
                if (PROCESSED.occupancy < PROCESSED.SIZE)	//Neelu: Commenting for ideal L1 prefetcher i.e. not sending to processed queue
                   PROCESSED.add_queue(&MSHR.entry[mshr_index]);
            }

	    if(warmup_complete[fill_cpu])
	      {
		uint64_t current_miss_latency = (current_core_cycle[fill_cpu] - MSHR.entry[mshr_index].cycle_enqueued);
		total_miss_latency += current_miss_latency;

	//	sim_miss_latency[fill_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id][MSHR.entry[mshr_index].type] += current_miss_latency;
	      }
	  
            MSHR.remove_queue(&MSHR.entry[mshr_index]);
            MSHR.num_returned--;

            update_fill_cycle();
        }
    }
}

ostream& operator<<(ostream& os, const PACKET &packet)
{
return os << " cpu: " << packet.cpu << " instr_id: " << packet.instr_id << " Translated: " << +packet.translated << " address: " << hex << packet.address << " full_addr: " << packet.full_addr << dec << " event_cycle: " << packet.event_cycle <<  " current_core_cycle: " <<  current_core_cycle[packet.cpu] << endl;
};



void CACHE::handle_writeback()
{
	
     if(WQ.occupancy == 0)
           return;

    multimap<uint64_t, uint32_t> writes_ready; //{cycle_time,index}

    assert(cache_type != IS_L1I || cache_type != IS_STLB); //@Vishal: TLB should not generate write packets

    if(cache_type == IS_L1D) //Get completed index in WQ, as it is non-fifo
    {
        for (uint32_t wq_index=0; wq_index < WQ.SIZE; wq_index++)
            if(WQ.entry[wq_index].translated == COMPLETED && (WQ.entry[wq_index].event_cycle <= current_core_cycle[cpu])) 
             writes_ready.insert({WQ.entry[wq_index].event_cycle, wq_index});
    }
    auto writes_ready_it = writes_ready.begin();

    if(cache_type == IS_L1D && writes_ready.size() == 0)
        return;
    
    if(cache_type == IS_L1D)
        WQ.head = writes_ready_it->second; //@Vishal: L1 WQ is non fifo, so head variable is set to next index to be read	

    // handle write
    uint32_t writeback_cpu = WQ.entry[WQ.head].cpu;
    if (writeback_cpu == NUM_CPUS)
        return;


    // handle the oldest entry
    if ((WQ.entry[WQ.head].event_cycle <= current_core_cycle[writeback_cpu]) && (WQ.occupancy > 0)) {
        int index = WQ.head;

        // access cache
        uint32_t set = get_set(WQ.entry[index].address);
        int way = check_hit(&WQ.entry[index], set);


        if (way >= 0) { // writeback hit (or RFO hit for L1D)

                (this->*update_replacement_state)(writeback_cpu, set, way, block[set][way].full_addr, WQ.entry[index].ip, 0, WQ.entry[index].type, 1);

            // COLLECT STATS
            sim_hit[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].type]++;
            sim_access[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].type]++;

#ifdef PT_STATS
	    if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && WQ.entry[index].type == LOAD_TRANSLATION)
	    {
		    assert(WQ.entry[index].translation_level > 0 && WQ.entry[index].translation_level < 6);
		    sim_pt_hit[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].translation_level-1]++;
		    sim_pt_access[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].translation_level-1]++;
            }
#endif
	    

            // mark dirty
            block[set][way].dirty = 1;

            if (cache_type == IS_ITLB)
                WQ.entry[index].instruction_pa = block[set][way].data;
            else if (cache_type == IS_DTLB)
                WQ.entry[index].data_pa = block[set][way].data;
            
                WQ.entry[index].data = block[set][way].data;

            // check fill level
            if (WQ.entry[index].fill_level < fill_level) {
		if(fill_level == FILL_L2)
		{
			if(WQ.entry[index].fill_l1i)
			{
				upper_level_icache[writeback_cpu]->return_data(&WQ.entry[index]);
			}
			if(WQ.entry[index].fill_l1d)
			{
				upper_level_dcache[writeback_cpu]->return_data(&WQ.entry[index]);
			}
		}
		else
		{
			if (WQ.entry[index].instruction)
				upper_level_icache[writeback_cpu]->return_data(&WQ.entry[index]);
			if (WQ.entry[index].is_data)
				upper_level_dcache[writeback_cpu]->return_data(&WQ.entry[index]);
		}

            }

            HIT[WQ.entry[index].type]++;
            ACCESS[WQ.entry[index].type]++;

            // remove this entry from WQ
            WQ.remove_queue(&WQ.entry[index]);
        }
        else { // writeback miss (or RFO miss for L1D)
            
            //DP ( if (warmup_complete[writeback_cpu]) {
            //cout << "[" << NAME << "] " << __func__ << " type: " << +WQ.entry[index].type << " miss";
            //cout << " instr_id: " << WQ.entry[index].instr_id << " address: " << hex << WQ.entry[index].address;
            //cout << " full_addr: " << WQ.entry[index].full_addr << dec;
            //cout << " cycle: " << WQ.entry[index].event_cycle << endl; });

            if (cache_type == IS_L1D) { // RFO miss

                // check mshr
                uint8_t miss_handled = 1;
                int mshr_index = check_nonfifo_queue(&MSHR, &WQ.entry[index],false); //@Vishal: Updated from check_mshr

		if(mshr_index == -2)
		{
			// this is a data/instruction collision in the MSHR, so we have to wait before we can allocate this miss
			miss_handled = 0;
		}

                if ((mshr_index == -1) && (MSHR.occupancy < MSHR_SIZE)) { // this is a new miss
        		      
                      assert(WQ.entry[index].full_physical_address != 0);
                     PACKET new_packet = WQ.entry[index];
                     //@Vishal: Send physical address to lower level and track physical address in MSHR  
                     new_packet.address = WQ.entry[index].full_physical_address >> LOG2_BLOCK_SIZE;
                     new_packet.full_addr = WQ.entry[index].full_physical_address; 


                      // add it to mshr (RFO miss)
        		      add_nonfifo_queue(&MSHR, &new_packet); //@Vishal: Updated from add_mshr
        		      
        		      // add it to the next level's read queue
        		      //if (lower_level) // L1D always has a lower level cache
        		      lower_level->add_rq(&new_packet);
                }
                else {
                    if ((mshr_index == -1) && (MSHR.occupancy == MSHR_SIZE)) { // not enough MSHR resource
                        
                        // cannot handle miss request until one of MSHRs is available
                        miss_handled = 0;
                        STALL[WQ.entry[index].type]++;
                    }
                    else if (mshr_index != -1) { // already in-flight miss

                        // update fill_level
                        if (WQ.entry[index].fill_level < MSHR.entry[mshr_index].fill_level)
                            MSHR.entry[mshr_index].fill_level = WQ.entry[index].fill_level;

			if((WQ.entry[index].fill_l1i) && (MSHR.entry[mshr_index].fill_l1i != 1))
			{
				MSHR.entry[mshr_index].fill_l1i = 1;
			}
			if((WQ.entry[index].fill_l1d) && (MSHR.entry[mshr_index].fill_l1d != 1))
			{
				MSHR.entry[mshr_index].fill_l1d = 1;
			}

                        // update request
                        if (MSHR.entry[mshr_index].type == PREFETCH) {
                            uint8_t  prior_returned = MSHR.entry[mshr_index].returned;
                            uint64_t prior_event_cycle = MSHR.entry[mshr_index].event_cycle;

			    uint64_t prior_address = MSHR.entry[mshr_index].address;
                            uint64_t prior_full_addr = MSHR.entry[mshr_index].full_addr;
                            uint64_t prior_full_physical_address = MSHR.entry[mshr_index].full_physical_address;


                            MSHR.entry[mshr_index] = WQ.entry[index];


                            //@Vishal: L1 RQ has virtual address, but miss needs to track physical address, so prior addresses are kept
                            if(cache_type == IS_L1D)
                            {
                                MSHR.entry[mshr_index].address = prior_address;
                                MSHR.entry[mshr_index].full_addr = prior_full_addr;
                                MSHR.entry[mshr_index].full_physical_address = prior_full_physical_address;
                            }

                            // in case request is already returned, we should keep event_cycle and retunred variables
                            MSHR.entry[mshr_index].returned = prior_returned;
                            MSHR.entry[mshr_index].event_cycle = prior_event_cycle;
                        }

                        MSHR_MERGED[WQ.entry[index].type]++;

                        //DP ( if (warmup_complete[writeback_cpu]) {
                        //cout << "[" << NAME << "] " << __func__ << " mshr merged";
                        //cout << " instr_id: " << WQ.entry[index].instr_id << " prior_id: " << MSHR.entry[mshr_index].instr_id; 
                        //cout << " address: " << hex << WQ.entry[index].address;
                        //cout << " full_addr: " << WQ.entry[index].full_addr << dec;
                        //cout << " cycle: " << WQ.entry[index].event_cycle << endl; });
                    }
                    else { // WE SHOULD NOT REACH HERE
                        cerr << "[" << NAME << "] MSHR errors" << endl;
                        assert(0);
                    }
                }

                if (miss_handled) {

                    MISS[WQ.entry[index].type]++;
                    ACCESS[WQ.entry[index].type]++;

                    // remove this entry from WQ
                    WQ.remove_queue(&WQ.entry[index]);
                }

            }
            else {
                // find victim
                uint32_t set = get_set(WQ.entry[index].address), way;
                    way = (this->*find_victim)(writeback_cpu, WQ.entry[index].instr_id, set, block[set], WQ.entry[index].ip, WQ.entry[index].full_addr, WQ.entry[index].type);

		 
#ifdef LLC_BYPASS
                if ((cache_type == IS_LLC) && (way == LLC_WAY)) {
                    cerr << "LLC bypassing for writebacks is not allowed!" << endl;
                    assert(0);
                }
#endif

                uint8_t  do_fill = 1;

                // is this dirty?
                if (block[set][way].dirty) {

                    // check if the lower level WQ has enough room to keep this writeback request
                    if (lower_level) { 
                        if (lower_level->get_occupancy(2, block[set][way].address) == lower_level->get_size(2, block[set][way].address)) {

                            // lower level WQ is full, cannot replace this victim
                            do_fill = 0;
                            lower_level->increment_WQ_FULL(block[set][way].address);
                            STALL[WQ.entry[index].type]++;

                            //DP ( if (warmup_complete[writeback_cpu] ) {
                            //cout << "[" << NAME << "] " << __func__ << "do_fill: " << +do_fill;
                            //cout << " lower level wq is full!" << " fill_addr: " << hex << WQ.entry[index].address;
                            //cout << " victim_addr: " << block[set][way].tag << dec << endl; });
                        }
                        else { 
                            PACKET writeback_packet;

                            writeback_packet.fill_level = fill_level << 1;
                            writeback_packet.cpu = writeback_cpu;
                            writeback_packet.address = block[set][way].address;
                            writeback_packet.full_addr = block[set][way].full_addr;
                            writeback_packet.data = block[set][way].data;
                            writeback_packet.instr_id = WQ.entry[index].instr_id;
                            writeback_packet.ip = 0;
                            writeback_packet.type = WRITEBACK;
                            writeback_packet.event_cycle = current_core_cycle[writeback_cpu];

                            lower_level->add_wq(&writeback_packet);
                        }
                    }
#ifdef SANITY_CHECK
                    else {
                        // sanity check
                        if (cache_type != IS_STLB)
                            assert(0);
                    }
#endif
                }

                if (do_fill) {
                    // update prefetcher
		    if (cache_type == IS_L1I)
			    l1i_prefetcher_cache_fill(writeback_cpu, ((WQ.entry[index].ip)>>LOG2_BLOCK_SIZE)<<LOG2_BLOCK_SIZE, set, way, 0, ((block[set][way].ip)>>LOG2_BLOCK_SIZE)<<LOG2_BLOCK_SIZE);
                    else if (cache_type == IS_L1D)
		    {
			    //Neelu: Sending virtual fill and evicted address to L1D prefetcher.
		      //l1d_prefetcher_cache_fill(WQ.entry[index].full_addr, set, way, 0, block[set][way].address<<LOG2_BLOCK_SIZE, WQ.entry[index].pf_metadata);
		    
			   uint64_t v_fill_addr, v_evicted_addr;
			   //First, getting virtual address for the fill address 
			   auto ppage_check = ooo_cpu[cpu].PTW.inverse_table.find(WQ.entry[index].full_addr >> LOG2_PAGE_SIZE);
			   //assert(ppage_check != ooo_cpu[cpu].PTW.inverse_table.end());
			   v_fill_addr = (ppage_check->second) << LOG2_PAGE_SIZE;
			   v_fill_addr |= (WQ.entry[index].full_addr & ((1 << LOG2_PAGE_SIZE)-1));

			   //Now getting virtual address for the evicted address
			   /*Neelu: Note that it is not always necessary that evicted address is a valid address and is present in the inverse table, hence (1) do not use the assert and (2) if it is not present, assign it to zero. */
			
			    ppage_check = ooo_cpu[cpu].PTW.inverse_table.find(block[set][way].address >> (LOG2_PAGE_SIZE - LOG2_BLOCK_SIZE));
			    if(ppage_check != ooo_cpu[cpu].PTW.inverse_table.end())
			    {
    				    v_evicted_addr = (ppage_check->second) << LOG2_PAGE_SIZE;
      				    v_evicted_addr |= ((block[set][way].address << LOG2_BLOCK_SIZE) & ((1 << LOG2_PAGE_SIZE)-1));
      			    }
      			    else
				    v_evicted_addr = 0;

      			    l1d_prefetcher_cache_fill(v_fill_addr, WQ.entry[index].full_addr, set, way, 0, v_evicted_addr, block[set][way].address<<LOG2_BLOCK_SIZE, WQ.entry[index].pf_metadata);		      
			   
		    }
                    else if (cache_type == IS_L2C)
		      WQ.entry[index].pf_metadata = l2c_prefetcher_cache_fill(WQ.entry[index].address<<LOG2_BLOCK_SIZE, set, way, 0,
									      block[set][way].address<<LOG2_BLOCK_SIZE, WQ.entry[index].pf_metadata);
                    if (cache_type == IS_LLC)
		      {
			cpu = writeback_cpu;
			WQ.entry[index].pf_metadata =llc_prefetcher_cache_fill(WQ.entry[index].address<<LOG2_BLOCK_SIZE, set, way, 0,
									       block[set][way].address<<LOG2_BLOCK_SIZE, WQ.entry[index].pf_metadata);
			cpu = 0;
		      }

#ifdef NOTIFY_L1D_OF_DTLB_EVICTION
		    //Neelu: Sending DTLB eviction notice to L1D
                        if(cache_type == IS_DTLB)
			{
				ooo_cpu[writeback_cpu].L1D.l1d_prefetcher_notify_about_dtlb_eviction(WQ.entry[index].address<<LOG2_PAGE_SIZE, set, way, 0, block[set][way].address<<LOG2_PAGE_SIZE, WQ.entry[index].pf_metadata);
			}
#endif

                    // update replacement policy
                        (this->*update_replacement_state)(writeback_cpu, set, way, WQ.entry[index].full_addr, WQ.entry[index].ip, block[set][way].full_addr, WQ.entry[index].type, 0);

                    // COLLECT STATS
                    sim_miss[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].type]++;
                    sim_access[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].type]++;
		    
#ifdef PT_STATS
		    if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && WQ.entry[index].type == LOAD_TRANSLATION)
		    {
			    assert(WQ.entry[index].translation_level > 0 && WQ.entry[index].translation_level < 6);
		            sim_pt_miss[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].translation_level-1]++;
		            sim_pt_access[writeback_cpu][((1 << LOG2_THREADS) - 1) & WQ.entry[index].instr_id][WQ.entry[index].translation_level-1]++;
		    }
#endif
		    

                    fill_cache(set, way, &WQ.entry[index]);

                    // mark dirty
                    block[set][way].dirty = 1; 

                    // check fill level
                    if (WQ.entry[index].fill_level < fill_level) {
			    if(fill_level == FILL_L2)
			    {
				    if(WQ.entry[index].fill_l1i)
				    {
					    upper_level_icache[writeback_cpu]->return_data(&WQ.entry[index]);
				    }
				    if(WQ.entry[index].fill_l1d)
				    {
					    upper_level_dcache[writeback_cpu]->return_data(&WQ.entry[index]);
				    }
			    }
			    else
			    {
				    if (WQ.entry[index].instruction)
					    upper_level_icache[writeback_cpu]->return_data(&WQ.entry[index]);
				    if (WQ.entry[index].is_data)
					    upper_level_dcache[writeback_cpu]->return_data(&WQ.entry[index]);
			    }

                    }

                    MISS[WQ.entry[index].type]++;
                    ACCESS[WQ.entry[index].type]++;

                    // remove this entry from WQ
                    WQ.remove_queue(&WQ.entry[index]);
                }
            }
        }
	}
}

//@Vishal: Translation coming from TLB to L1 cache
void CACHE::handle_processed()
{
	assert(cache_type == IS_L1I || cache_type == IS_L1D);

	CACHE &tlb = cache_type == IS_L1I ? ooo_cpu[cpu].ITLB : ooo_cpu[cpu].DTLB;

	//@Vishal: one translation is processed per cycle
	if(tlb.PROCESSED.occupancy != 0)
	{
		if((tlb.PROCESSED.entry[tlb.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
		{
			int index = tlb.PROCESSED.head;

			if(tlb.PROCESSED.entry[index].l1_rq_index != -1)
			{
				assert(tlb.PROCESSED.entry[index].l1_wq_index == -1 && tlb.PROCESSED.entry[index].l1_pq_index == -1); //Entry can't have write and prefetch index

				int rq_index = tlb.PROCESSED.entry[index].l1_rq_index;
	
				//DP ( if (warmup_complete[cpu] ) {	
				//cout << "["<<NAME << "_handle_processed] packet: " << RQ.entry[rq_index]; });

				RQ.entry[rq_index].translated = COMPLETED;

                		if(tlb.cache_type == IS_ITLB)
				    RQ.entry[rq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (RQ.entry[rq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
				else
                    			RQ.entry[rq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (RQ.entry[rq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                
				// ADD LATENCY
					if (RQ.entry[rq_index].event_cycle < current_core_cycle[cpu])
						RQ.entry[rq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
					else
						RQ.entry[rq_index].event_cycle += LATENCY;
			}
			else if(tlb.PROCESSED.entry[index].l1_wq_index != -1)
			{
				assert(tlb.PROCESSED.entry[index].l1_rq_index == -1 && tlb.PROCESSED.entry[index].l1_pq_index == -1); //Entry can't have read and prefetch index

				int wq_index = tlb.PROCESSED.entry[index].l1_wq_index;

				WQ.entry[wq_index].translated = COMPLETED;

                if(tlb.cache_type == IS_ITLB)
				    WQ.entry[wq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (WQ.entry[wq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
				else
                    WQ.entry[wq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (WQ.entry[wq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                

				// ADD LATENCY
					if (WQ.entry[wq_index].event_cycle < current_core_cycle[cpu])
						WQ.entry[wq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
					else
						WQ.entry[wq_index].event_cycle += LATENCY;

				//DP ( if (warmup_complete[cpu] ) {
				//cout << "["<<NAME << "_handle_processed] packet: " << WQ.entry[wq_index];});

			}
			//Neelu: Checking for l1_pq_index as well as L1I prefetching is turned on and the corresponding translation requests are sent to ITLB.
			else if(tlb.PROCESSED.entry[index].l1_pq_index != -1)
			{
				//Neelu: This should occur only for L1I, because L1D prefetch requests get translations from STLB.
				assert(cache_type == IS_L1I);

				assert(tlb.PROCESSED.entry[index].l1_wq_index == -1 && tlb.PROCESSED.entry[index].l1_rq_index == -1); //Entry can't have write and read index

                                int pq_index = tlb.PROCESSED.entry[index].l1_pq_index;

                                PQ.entry[pq_index].translated = COMPLETED;

                if(tlb.cache_type == IS_ITLB)
                                    PQ.entry[pq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (PQ.entry[pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                                else
				{
					//Neelu: L1D Prefetch translation requests don't go to DTLB.
					assert(0);
                    PQ.entry[pq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (PQ.entry[pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
				}
			
				//DP ( if (warmup_complete[cpu] ) {
				//cout << "["<<NAME << "_handle_processed] packet: " << PQ.entry[pq_index];});

                                // ADD LATENCY
                                        if (PQ.entry[pq_index].event_cycle < current_core_cycle[cpu])
                                                PQ.entry[pq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
                                        else
                                                PQ.entry[pq_index].event_cycle += LATENCY;
			}
			else
            {
		    assert(0); //Either read queue, prefetch queue or write queue index should be present
            }


				//Neelu: Commenting this assert as ITLB can have translation requests from L1I prefetches. 
				//assert(tlb.PROCESSED.entry[index].prefetch_translation_merged == false);

				if(tlb.PROCESSED.entry[index].read_translation_merged)
                {
                    ITERATE_SET(other_rq_index,tlb.PROCESSED.entry[index].l1_rq_index_depend_on_me, RQ.SIZE)
                    {
                        if(RQ.entry[other_rq_index].translated == INFLIGHT)
                        {
                            RQ.entry[other_rq_index].translated = COMPLETED;

                            if(tlb.cache_type == IS_ITLB)
                                RQ.entry[other_rq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (RQ.entry[other_rq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                            else
                                RQ.entry[other_rq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (RQ.entry[other_rq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                            
                            RQ.entry[other_rq_index].event_cycle = current_core_cycle[cpu];
							// ADD LATENCY
							if (RQ.entry[other_rq_index].event_cycle < current_core_cycle[cpu])
								RQ.entry[other_rq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
							else
								RQ.entry[other_rq_index].event_cycle += LATENCY;

								//DP ( if (warmup_complete[cpu] ) {
								//cout << "["<<NAME << "_handle_processed] packet: " << RQ.entry[other_rq_index];});
                        }
                    }
                }


				if(tlb.PROCESSED.entry[index].write_translation_merged)
				{
					ITERATE_SET(other_wq_index,tlb.PROCESSED.entry[index].l1_wq_index_depend_on_me, WQ.SIZE) 
					{
						WQ.entry[other_wq_index].translated = COMPLETED;

                        if(tlb.cache_type == IS_ITLB)
						  WQ.entry[other_wq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (WQ.entry[other_wq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
						else
                          WQ.entry[other_wq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (WQ.entry[other_wq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                          
                        WQ.entry[other_wq_index].event_cycle = current_core_cycle[cpu];
						// ADD LATENCY
							if (WQ.entry[other_wq_index].event_cycle < current_core_cycle[cpu])
								WQ.entry[other_wq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
							else
								WQ.entry[other_wq_index].event_cycle += LATENCY;

						//DP ( if (warmup_complete[cpu] ) {
						//cout << "["<<NAME << "_handle_processed] packet: " << WQ.entry[other_wq_index];});
					}
				}


				//Neelu: Checking for prefetch_translation_merges too.
				if(tlb.PROCESSED.entry[index].prefetch_translation_merged)
				{
					ITERATE_SET(other_pq_index,tlb.PROCESSED.entry[index].l1_pq_index_depend_on_me, PQ.SIZE)
                                        {
						//Neelu: Do we need to check whether translation is inflight or not? 
                                                PQ.entry[other_pq_index].translated = COMPLETED;

                        if(tlb.cache_type == IS_ITLB)
                                                  PQ.entry[other_pq_index].full_physical_address = (tlb.PROCESSED.entry[index].instruction_pa << LOG2_PAGE_SIZE) | (PQ.entry[other_pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
                                                else
			{
				assert(0); // Translation cannot come from DTLB
                          PQ.entry[other_pq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (PQ.entry[other_pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
			}

                        PQ.entry[other_pq_index].event_cycle = current_core_cycle[cpu];

				//DP ( if (warmup_complete[cpu] ) {
				//cout << "["<<NAME << "_handle_processed] packet: " << PQ.entry[other_pq_index];});

                                                // ADD LATENCY
                                                        if (PQ.entry[other_pq_index].event_cycle < current_core_cycle[cpu])
                                                                PQ.entry[other_pq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
                                                        else
                                                                PQ.entry[other_pq_index].event_cycle += LATENCY;

                                        }

				}


			tlb.PROCESSED.remove_queue(&tlb.PROCESSED.entry[index]);
		}
	}

	//@Vishal: Check for Prefetch translations from STLB processed queue
	if(cache_type == IS_L1D)
	{
		CACHE &tlb = ooo_cpu[cpu].STLB;

		//@Vishal: one translation is processed per cycle
		if(tlb.PROCESSED.occupancy != 0)
		{
			if((tlb.PROCESSED.entry[tlb.PROCESSED.head].event_cycle <= current_core_cycle[cpu]))
			{
				int index = tlb.PROCESSED.head;

				if(tlb.PROCESSED.entry[index].l1_pq_index != -1)
				{
					int pq_index = tlb.PROCESSED.entry[index].l1_pq_index;

					PQ.entry[pq_index].translated = COMPLETED;

					//@Vishal: L1D prefetch is sending request, so translation should be in data_pa.
					assert(tlb.PROCESSED.entry[index].data_pa != 0 && tlb.PROCESSED.entry[index].instruction_pa == 0);

	                PQ.entry[pq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (PQ.entry[pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
	               
					//DP ( if (warmup_complete[cpu] ) {
				//cout << "["<<NAME << "_handle_processed] packet: " << PQ.entry[pq_index];});

 
					// ADD LATENCY
					if (PQ.entry[pq_index].event_cycle < current_core_cycle[cpu])
						PQ.entry[pq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
					else
						PQ.entry[pq_index].event_cycle += LATENCY;

					assert((tlb.PROCESSED.entry[index].read_translation_merged == false) && (tlb.PROCESSED.entry[index].write_translation_merged == false));
				}

					if(tlb.PROCESSED.entry[index].prefetch_translation_merged)
					{
						ITERATE_SET(other_pq_index, tlb.PROCESSED.entry[index].l1_pq_index_depend_on_me, PQ.SIZE) 
						{
							if(PQ.entry[other_pq_index].translated == INFLIGHT)
							{
								PQ.entry[other_pq_index].translated = COMPLETED;
								PQ.entry[other_pq_index].full_physical_address = (tlb.PROCESSED.entry[index].data_pa << LOG2_PAGE_SIZE) | (PQ.entry[other_pq_index].full_addr & ((1 << LOG2_PAGE_SIZE) - 1));
			
				//DP ( if (warmup_complete[cpu] ) {
				//cout << "["<<NAME << "_handle_processed] packet: " << PQ.entry[other_pq_index];});


				// ADD LATENCY
							if (PQ.entry[other_pq_index].event_cycle < current_core_cycle[cpu])
								PQ.entry[other_pq_index].event_cycle = current_core_cycle[cpu] + LATENCY;
							else
								PQ.entry[other_pq_index].event_cycle += LATENCY;
							}
						}
					}
		
				tlb.PROCESSED.remove_queue(&tlb.PROCESSED.entry[index]);
			}
			else
				return;
		}
	}
}

//@Vasudha: To find reuse distance
/*void increment_reuse(struct find_reuse &Find_reuse, uint64_t distance)
{
	if (distance >= 0 && distance <= 50)
		Find_reuse.reuse_1_50 =+ 1;
	else if (distance > 50 && distance <= 500)
		Find_reuse.reuse_50_500 =+ 1;
	else if (distance > 500 && distance <= 5000)
		Find_reuse.reuse_500_5000 =+ 1;
	else if (distance > 5000 && distance <= 50000)
		Find_reuse.reuse_5000_50000 =+ 1;
	else if (distance > 50000)
		Find_reuse.reuse_50000 =+ 1;	


}*/

void CACHE::handle_read()
{
    if(cache_type == IS_L1D) {
	//	//cout << "Handle read cycle: " << current_core_cycle[cpu] << "PQ Occupancy: " << PQ.occupancy << endl;
	sum_pq_occupancy += PQ.occupancy;
    }

    if(RQ.occupancy == 0)
	   return;


    multimap<uint64_t, uint32_t> reads_ready; //{cycle_time,index}

    if(cache_type == IS_L1I || cache_type == IS_L1D) //Get completed index in RQ, as it is non-fifo
    {
    	for (uint32_t rq_index=0; rq_index < RQ.SIZE; rq_index++)
		if(RQ.entry[rq_index].cpu < NUM_CPUS && RQ.entry[rq_index].translated == COMPLETED && (RQ.entry[rq_index].event_cycle <= current_core_cycle[cpu])) 
			reads_ready.insert({RQ.entry[rq_index].event_cycle, rq_index});
    }
    auto reads_ready_it = reads_ready.begin();

    if((cache_type == IS_L1I || cache_type == IS_L1D) && reads_ready.size() == 0)
    	return;

    //cout << "handle_read called with reads available = " << reads_available_this_cycle << endl;

    // handle read
    static int counter;
    for (uint32_t i=0; i<MAX_READ; i++) {


      if(cache_type == IS_L1I || cache_type == IS_L1D)
      {
      		if(reads_ready_it == reads_ready.end())
      			return;
      		RQ.head = reads_ready_it->second; //@Vishal: L1 RQ is non fifo, so head variable is set to next index to be read
      		reads_ready_it++;
      }

      uint32_t read_cpu = RQ.entry[RQ.head].cpu;
      if (read_cpu == NUM_CPUS)
        return;

        // handle the oldest entry
        if ((RQ.entry[RQ.head].event_cycle <= current_core_cycle[read_cpu]) && (RQ.occupancy > 0)) {
            int index = RQ.head;

            // access cache
            uint32_t set = get_set(RQ.entry[index].address);
            int way = check_hit(&RQ.entry[index], set);


	    #ifdef ARKA_DP_PRED
	    if (cache_type == IS_STLB && way < 0 && all_warmup_complete > ooo_cpu[read_cpu].thread)
	    {
		    int found;
		    // if STLB miss, search in Shadow table
		    found = check_shadow_table(&RQ.entry[index]);
		    if (found == 1)
		    {
            		way = (this->*find_victim)(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
                	(this->*update_replacement_state)(read_cpu, set, way, RQ.entry[index].full_addr, RQ.entry[index].ip, block[set][way].full_addr,RQ.entry[index].type,0);
		    	fill_cache(set, way, &RQ.entry[index]);
		    }
	    }
	    if (cache_type == IS_STLB)
	    {
		int way = ooo_cpu[0].VICTIM_ST.check_hit(&RQ.entry[index], set);
		if (way >= 0)
			 ooo_cpu[0].VICTIM_ST.block[set][way].used = 1;
	    }
	    #endif

	    uint16_t thread_index = ((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id;
            if (knob_cloudsuite)
		    thread_index = 0;
	    
	    #ifdef PERFECT_DTLB_EXCLUDING_COLD_MISSES
	    #ifdef DETECT_CRITICAL_TRANSLATIONS
	    if (cache_type == IS_DTLB && critical_translation[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != critical_translation[cpu][thread_index].end() && way < 0) 
	    {
	    	    assert( warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id] );
		    if (ooo_cpu[0].PTW.page_table[thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != ooo_cpu[0].PTW.page_table[thread_index].end())
			    way = NUM_WAY;
		    else
			    assert(0);
	    }
	    #endif
	    #ifdef DETECT_CRITICAL_IPS
	    if(cache_type == IS_DTLB && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end() && way<0)
            {
		    int found = 0;
                    map <uint64_t, uint64_t>::iterator pr = ooo_cpu[0].PTW.page_table[thread_index].begin();
                    if (ooo_cpu[0].PTW.page_table[thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != ooo_cpu[0].PTW.page_table[thread_index].end()) 
		    {
			      for (pr; pr != ooo_cpu[0].PTW.page_table[thread_index].end(); pr++)
			      {
				      if (pr->first == RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
				      {
					// if hit, no change in way
					// if miss, 
					// 1. find victim way in DTLB
					// 2. fill_cache
					// way = NUM_WAY
					////way = (this->*find_victim)(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
					RQ.entry[index].data = pr->second;
	        			RQ.entry[index].data_pa = pr->second;
					if (PROCESSED.occupancy < PROCESSED.SIZE)
					{
						RQ.entry[index].sent = 1;
						PROCESSED.add_queue(&RQ.entry[index]);
					}
					//fill_cache(set, way, &RQ.entry[index]);
					found = 1;
					break;
				      }
			    }
			      if (found == 0)
				      assert(0);
		    }
		    else
			    RQ.entry[index].data_pa = UINT64_MAX;
            }   
	    #endif
	    #if !defined(DETECT_CRITICAL_TRANSLATIONS) && !defined(DETECT_CRITICAL_IPS)
	    if(cache_type == IS_DTLB)// && way < 0)
	    {
		    auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find( RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE );
	    	    if (it != ooo_cpu[cpu].PTW.page_table[thread_index].end())
			way = NUM_WAY;
	    }
	    #endif
	    #endif
	    
	    #ifdef PERFECT_STLB_EXCLUDING_COLD_MISSES
	    #ifdef DETECT_CRITICAL_IPS
	    if(cache_type == IS_STLB && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end() && way<0 && RQ.entry[index].instruction != 1 && RQ.entry[index].sent == 0)
            {
		    int found = 0;
                    map <uint64_t, uint64_t>::iterator pr = ooo_cpu[cpu].PTW.page_table[thread_index].begin();
                    if (ooo_cpu[cpu].PTW.page_table[thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != ooo_cpu[cpu].PTW.page_table[thread_index].end()) 
		    {
			      for (pr; pr != ooo_cpu[cpu].PTW.page_table[thread_index].end(); pr++)
			      {
				      if (pr->first == RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
				      {
					RQ.entry[index].data = pr->second;
	        			RQ.entry[index].data_pa = pr->second;
					RQ.entry[index].sent = 1;
					int way_dtlb;
					upper_level_dcache[cpu]->return_data(&RQ.entry[index]);

					found = 1;
					break;
				      }
			    }
			      if (found == 0)
				      assert(0);
		    }
		    else
			    RQ.entry[index].data_pa = UINT64_MAX;
            }   
	    #endif
	    #if !defined(DETECT_CRITICAL_TRANSLATIONS) && !defined(DETECT_CRITICAL_IPS)
	    if (cache_type == IS_STLB)
	    {
		    auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find( RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE );
		    if (it != ooo_cpu[cpu].PTW.page_table[thread_index].end())
			    way = 1;
	    }
	    #endif
	    #endif

     	    //@Vasudha: Perfect DTLB Critical Prefetcher
	    #ifdef PERFECT_DTLB
	    #ifdef DETECT_CRITICAL_IPS
	    if(cache_type == IS_DTLB && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end() && way<0)
            {
		    int found = 0;
                    map <uint64_t, uint64_t>::iterator pr = ooo_cpu[0].PTW.page_table[thread_index].begin();
                    //if (ooo_cpu[0].PTW.page_table[thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != ooo_cpu[0].PTW.page_table[thread_index].end()) {
                      for (pr; pr != ooo_cpu[0].PTW.page_table[thread_index].end(); pr++)
		      {
			      if (pr->first == RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
			      {
		        	// if hit, no change in way
				// if miss, 
				// 1. find victim way in DTLB
				// 2. fill_cache
				// way = NUM_WAY
				//way = (this->*find_victim)(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
				RQ.entry[index].data = pr->second;
				RQ.entry[index].data_pa = pr->second;
				//fill_cache(set, way, &RQ.entry[index]);
                		//int new_miss = check_nonfifo_queue(&MSHR, &RQ.entry[index],false); //@Vishal: Updated from check_mshr
				//if (new_miss == -1)
				//{
				if (PROCESSED.occupancy < PROCESSED.SIZE)
                        	{
					PROCESSED.add_queue(&RQ.entry[index]);
					RQ.entry[index].sent = 1;
					DP (if (all_warmup_complete > ooo_cpu[cpu].thread)
					{
						cout << "Translation sent instr_id "<<dec<<(RQ.entry[index].instr_id >>3) <<" address:"<<hex<< RQ.entry[index].address;
						cout << " Full_addr: " << RQ.entry[index].full_addr << " pa-" << RQ.entry[index].data_pa <<dec<< endl;
					});
				}	
				//}
				found = 1;
				break;
			      }
		    }
		      if (found == 0)
			      assert(0);
            }   
	    #endif
	    #ifdef DETECT_CRITICAL_TRANSLATIONS
	    /*if (cache_type == IS_DTLB && critical_translation[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != critical_translation[cpu][thread_index].end() && way < 0) 
	    {
	    	    assert( warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id] );
		    if (ooo_cpu[0].PTW.page_table[thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE) != ooo_cpu[0].PTW.page_table[thread_index].end())
			    way = NUM_WAY;
		    else
			    assert(0);
	    }*/
	    #endif
	    #endif
	    
	    
		    int way_vb = -1;
	    #ifdef PUSH_VICTIMS_DTLB_VB
	    if (cache_type == IS_DTLB && way < 0)
	    {
		    way_vb = ooo_cpu[read_cpu].DTLB_VB.check_hit( &RQ.entry[index], 0);
		    if (way_vb >= 0)
		    {
			    #ifdef DEBUG
			    cout << dec << "Cycle: "<<current_core_cycle[cpu] << " T" << (((1 << LOG2_THREADS)-1) & RQ.entry[index].instr_id) <<" DTLB_VB_HIT, address: " <<hex<<RQ.entry[index].address << 
				    " region: " << (RQ.entry[index].address >> 13) << endl;
			    #endif
			    ooo_cpu[read_cpu].DTLB_VB.block[0][way_vb].used = 1;
			    RQ.entry[index].data_pa = ooo_cpu[read_cpu].DTLB_VB.block[0][way_vb].data;
			    ooo_cpu[read_cpu].DTLB_VB.sim_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                	    ooo_cpu[read_cpu].DTLB_VB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
			    ooo_cpu[read_cpu].DTLB_VB.HIT[RQ.entry[index].type]++;
                	    ooo_cpu[read_cpu].DTLB_VB.ACCESS[RQ.entry[index].type]++;
	   	    	    (&ooo_cpu[read_cpu].DTLB_VB->*update_replacement_state)(read_cpu, 0, way_vb, ooo_cpu[read_cpu].DTLB_VB.block[0][way_vb].full_addr, RQ.entry[index].ip, 0, RQ.entry[index].type, 1);
		    	    RQ.entry[index].data = ooo_cpu[read_cpu].DTLB_VB.block[0][way_vb].data;
	        	    if (PROCESSED.occupancy < PROCESSED.SIZE)
                           	 PROCESSED.add_queue(&RQ.entry[index]);
			    //if (RQ.entry[index].type == LOAD_TRANSLATION) 
				// dtlb_prefetcher_operate(RQ.entry[index].address, RQ.entry[index].ip, 1, RQ.entry[index].type, RQ.entry[index].instr_id, RQ.entry[index].instruction, RQ.entry[index].critical_ip_flag);
                	    RQ.remove_queue(&RQ.entry[index]);
			    //ooo_cpu[read_cpu].DTLB_VB.pf_useful++;
		    	    // If DTLB victim buffer gets hit, fill DTLB and then proceed
	    	    	    //way = find_victim(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
		      	    //RQ.entry[index].type = PREFETCH_TRANSLATION;
			    //fill_cache(set, way, &RQ.entry[index]);
		    }
		    else
		    {
			    //DTLB_VB MISS
			    ooo_cpu[read_cpu].DTLB_VB.MISS[RQ.entry[index].type]++;
                    	    ooo_cpu[read_cpu].DTLB_VB.ACCESS[RQ.entry[index].type]++;
			    ooo_cpu[read_cpu].DTLB_VB.sim_miss[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_VB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
		    }

	    }
	    #endif

	    #ifdef PERFECT_STLB
	    #if !defined(DETECT_CRITICAL_IPS) && !defined(DETECT_CRITICAL_TRANSLATIONS)
	    	if (cache_type == 2)
			way = 1;
	    #endif
	    #endif

	    #ifdef PERFECT_DTLB
	    #if !defined(DETECT_CRITICAL_IPS) && !defined(DETECT_CRITICAL_TRANSLATIONS)
	    	if(cache_type == 1) //Perfect DTLB and Baseline ITLB
	    		    way = 1;
	    #endif
	    #endif

	    #ifdef DEBUG_PREF
	    if(cache_type == IS_STLB && all_warmup_complete > ooo_cpu[cpu].thread)
	    {
		   if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end()) 
	    	    cout << "IS-CRITICAL " ;
	    	   else 
		    cout << "NON-CRITICAL ";
	    
		    cout <<  "Cycle: "<<dec <<current_core_cycle[0] << hex <<" DEMAND PAGE REQ. " <<  (RQ.entry[index].address) <<" IP: ";
		    cout << RQ.entry[index].ip << "  HIT: " << (way<0?0:1) << " THREAD: "<<  (((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id) << dec << endl ;
	    }
	    #endif



	    //@Vasudha - Dumping translation entries of DTLB with cycle number and hit/miss
	    /*if (warmup_complete[read_cpu] && cache_type==1 ) {
		cout <<  "no" << counter << ",T" << (((1<<LOG2_THREADS)-1) & RQ.entry[index].instr_id) << ","  << hex << (RQ.entry[index].address>>9) << "," << dec << RQ.entry[index].event_cycle << "," ;
		if(way >=0) cout << "1" << endl;
		else cout << "0" << endl;
		counter++;
            }*/

	    //@Vasudha: SET WISE REUSE DISTANCE FOR DOA and dead entries
	    if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[cpu].thread)
	    {
		    {
		    /*auto it = trans_per_cycle[set].find(current_core_cycle[cpu]);
		    if (it == trans_per_cycle[set].end())
		    {
			    uniqt.clear();
			    uniqt.insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
			    trans_per_cycle[set].insert({current_core_cycle[cpu], uniqt});
		    }
		    else
			    (it->second).insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);*/

		    total_access_count[set]++;
		    trans_per_cycle[set].push_back(((RQ.entry[index].address) << 3) | thread_index);		    
	            }
		    //@Vasudha: TODO
		    //Find reuse distance of DOA and dead entries
		    //After 1M cycles delete entries from unordered map
		    
		    //check if the DTLB demand request got evicted previously as dead/DOA
		    int found = 0;
		    {auto it = evict_dead_trans[thread_index][0][set].find(RQ.entry[index].address);
		    if (it != evict_dead_trans[thread_index][0][set].end())
		    {
			    found = 1;
			    /*uniqt.clear();
			    for (uint64_t last_access = (it->second); last_access < current_core_cycle[cpu]; last_access++)
			    {
				auto search_cycle = trans_per_cycle[set].find(last_access);
				if (search_cycle != trans_per_cycle[set].end())
				{
				    Iterator = (search_cycle->second).begin();
				    for (; Iterator != (search_cycle->second).end(); Iterator++)
				    	uniqt.insert(*Iterator);
				}
			    }*/
			    uint64_t last_access = (it->second);
			    if (trans_per_cycle[set].size() == 0)
				    cout << "EMPTY trans_per_cycle" << endl;
			    
			    	std::unordered_set<uint64_t> uniqt(trans_per_cycle[set].begin()+last_access, trans_per_cycle[set].end());

			    evict_dead_trans[thread_index][0][set].erase(RQ.entry[index].address);
			    if (uniqt.size() >= 1 && uniqt.size() <= 4)
					dead_reuse_1_4[0]++;
			    else if (uniqt.size() >= 5 && uniqt.size() <= 12)
					dead_reuse_5_12[0]++;
			    else if (uniqt.size() >= 13 && uniqt.size() <= 50)
					dead_reuse_13_50[0]++;
			    else if (uniqt.size() >= 51 && uniqt.size() <= 100)
				    dead_reuse_51_100[0]++;
		            else if (uniqt.size() >= 101 && uniqt.size() <= 500)
					dead_reuse_101_500[0]++;
			    else if (uniqt.size() >= 501 && uniqt.size() <= 1000)
					dead_reuse_501_1000[0]++;
			    else if (uniqt.size() >= 1001)
					dead_reuse_1001[0]++;
		    }}

		    auto it = evict_dead_trans[thread_index][1][set].find(RQ.entry[index].address);
		    if (it != evict_dead_trans[thread_index][1][set].end())
		    {
			    if (found == 1)
				    assert(0);
			    /*uniqt.clear();
			    for (uint64_t last_access = (it->second); last_access < current_core_cycle[cpu]; last_access++)
			    {
				auto search_cycle = trans_per_cycle[set].find(last_access);
				if (search_cycle != trans_per_cycle[set].end())
				{
				    Iterator = (search_cycle->second).begin();
				    for (; Iterator != (search_cycle->second).end(); Iterator++)
				    	uniqt.insert(*Iterator);
				}
			    }*/

			    uint64_t last_access = (it->second);
			    std::unordered_set<uint64_t> uniqt(trans_per_cycle[set].begin()+last_access, trans_per_cycle[set].end());

			    evict_dead_trans[thread_index][1][set].erase(RQ.entry[index].address);
			    if (uniqt.size() >= 1 && uniqt.size() <= 4)
					dead_reuse_1_4[1]++;
			    else if (uniqt.size() >= 5 && uniqt.size() <= 12)
					dead_reuse_5_12[1]++;
			    else if (uniqt.size() >= 13 && uniqt.size() <= 50)
					dead_reuse_13_50[1]++;
			    else if (uniqt.size() >= 51 && uniqt.size() <= 100)
				    dead_reuse_51_100[1]++;
		            else if (uniqt.size() >= 101 && uniqt.size() <= 500)
					dead_reuse_101_500[1]++;
			    else if (uniqt.size() >= 501 && uniqt.size() <= 1000)
					dead_reuse_501_1000[1]++;
			    else if (uniqt.size() >= 1001)
					dead_reuse_1001[1]++;

		    }

		    /*if (current_core_cycle[cpu] % 1000000 == 0)
		    {
			    //remove old stats from trans_per_cycle
			    for (uint32_t set = 0; set < DTLB_SET; set++)
			    {
				    uint32_t min_cycle = current_core_cycle[cpu];

				    map <uint64_t, uint64_t>::iterator iter;
				    for (uint16_t thread_num=0; thread_num < ooo_cpu[cpu].thread; thread_num)
				    {
				    	iter = evict_dead_trans[thread_num][0][set].begin();
				        for (iter; iter != evict_dead_trans[thread_num][0][set].end(); iter++)
						if ((iter->second) < min_cycle)	
							min_cycle = (iter->second);

					iter = evict_dead_trans[thread_num][1][set].begin();
					for (iter; iter != evict_dead_trans[thread_num][1][set].end(); iter++)
						if ((iter->second) < min_cycle)
							min_cycle = (iter->second);
				    }

				    if (min_cycle != current_core_cycle[cpu])
				    {
				    	Iter = trans_per_cycle[set].begin();
				    	for (Iter; Iter != trans_per_cycle[set].end(); Iter++)
				    	{
						if ((Iter->first) < min_cycle)
						trans_per_cycle[set].erase((Iter->first));
				    	}
				    }
			    }
		    }*/
	    }


	#ifdef DETECT_CRITICAL_IPS
	//@Vasudha: SET_WISE reuse distance
	/*if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[cpu].thread)
	{
		auto it = trans_per_cycle[set].find(current_core_cycle[cpu]);
		if (it == trans_per_cycle[set].end())
		{
			uniqt.clear();
			uniqt.insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
			trans_per_cycle[set].insert({current_core_cycle[cpu], uniqt});
		}
		else
		{
			(it->second).insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		}
		
		auto search = critical_ips[cpu][thread_index].find(RQ.entry[index].ip);
		if (search != critical_ips[cpu][thread_index].end())
		{
			auto search_trans = critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
			if (search_trans != critical_reuse[cpu][thread_index].end())
			{
				uniqt.clear();
				//find reuse distance
				uint64_t last_access = (search_trans->second).last_cycle_accessed;
				for ( ; last_access <= current_core_cycle[cpu]; last_access++)
				{
					auto search_cycle = trans_per_cycle[set].find(last_access);
					if (search_cycle != trans_per_cycle[set].end())
					{
						Iterator = (search_cycle->second).begin();
						for (; Iterator != (search_cycle->second).end(); Iterator++)
							uniqt.insert(*Iterator);
					}	
				}
				if (uniqt.size() >= 1 && uniqt.size() <= 4)
					(search_trans->second).reuse_1_4++;
				else if (uniqt.size() >= 5 && uniqt.size() <= 12)
					(search_trans->second).reuse_5_12++;
				else if (uniqt.size() >= 13 && uniqt.size() <=100)
					(search_trans->second).reuse_13_100++;
				else if (uniqt.size() >= 101 && uniqt.size() <= 500)
					(search_trans->second).reuse_101_500++;
				else if (uniqt.size() >= 501 && uniqt.size() <= 1000)
					(search_trans->second).reuse_501_1000++;
				else if (uniqt.size() >= 1001)
					(search_trans->second).reuse_1001++;
				(search_trans->second).last_cycle_accessed = current_core_cycle[cpu];
			}
			else
			{
				//insert trans and initialise other paramters
				struct find_reuse Find_reuse;
				Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
				Find_reuse.reuse_1_4 = 0;
				Find_reuse.reuse_5_12 = 0;
				Find_reuse.reuse_13_100 = 0;
				Find_reuse.reuse_101_500 = 0;
				Find_reuse.reuse_501_1000 = 0;
				Find_reuse.reuse_1001 = 0;
				critical_reuse[cpu][thread_index].insert({RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse});
			}
		}
	}*/
	#endif


	//@Vasudha:(Single map implementation)To find reuse distance(in terms of accesses) of translations accessed by only critical IPs, only non-critical IPs and both
	/*#ifdef DETECT_CRITICAL_IPS
 	if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[cpu].thread)
	{
		auto it = trans_per_cycle[cpu][thread_index].find(current_core_cycle[cpu]);
		if (it == trans_per_cycle[cpu][thread_index].end())
		{
			//set<uint64_t> uniqt;
			uniqt.clear();
			uniqt.insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
			//trans_per_cycle[cpu][thread_index].insert(pair <uint64_t, set<uint64_t> (RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, uniqt));
			trans_per_cycle[cpu][thread_index].insert({RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, uniqt});
		}
		else
		{
			(it->second).insert(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		}

		auto search = critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != critical_reuse[cpu][thread_index].end())
		{
			if ((search->second).criticalOrNot == 0 || (search->second).criticalOrNot == 1)
			{	
				// find no. of unique translations between previous and current accesses
				uint64_t prev_cycle = (search->second).last_cycle_accessed;
				temp_set.clear();
				for (prev_cycle; prev_cycle < current_core_cycle[cpu]; prev_cycle++)
				{
					auto uiter = trans_per_cycle[cpu][thread_index].find(prev_cycle);
					if (uiter != trans_per_cycle[cpu][thread_index].end())
					{
						//set<uint64_t>::iterator Iterator;
						for (Iterator = (uiter->second).begin(); Iterator != (uiter->second).end(); Iterator++)
						{
							temp_set.insert(*Iterator);
						}
					}
				}
				increment_reuse((search->second), temp_set.size());
				temp_set.clear();
			}
			(search->second).last_cycle_accessed = current_core_cycle[cpu];

			if ((search->second).criticalOrNot == 0 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
			{
				// store all non-critical stats and restart calculating reuse distance for critical IPs
				nc_reuse_1_50 += (search->second).reuse_1_50;
				nc_reuse_50_500 += (search->second).reuse_50_500;
				nc_reuse_500_5000 += (search->second).reuse_500_5000;
				nc_reuse_5000_50000 += (search->second).reuse_5000_50000;
				nc_reuse_50000 += (search->second).reuse_50000;
				(search->second).reuse_1_50 = 0;
				(search->second).reuse_50_500 = 0;
				(search->second).reuse_500_5000 = 0;
				(search->second).reuse_5000_50000 = 0;
				(search->second).reuse_50000 = 0;
				(search->second).criticalOrNot = 1;
			}
			if ((search->second).criticalOrNot == 1 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
				(search->second).criticalOrNot = 2;
		}
		else
		{
			struct find_reuse Find_reuse;
			Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
			Find_reuse.criticalOrNot = 0;
			Find_reuse.reuse_1_50 = 0;
			Find_reuse.reuse_50_500 = 0;
			Find_reuse.reuse_500_5000 = 0;
			Find_reuse.reuse_5000_50000 = 0;
			Find_reuse.reuse_50000 = 0;
			
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				Find_reuse.criticalOrNot = 1;	
			critical_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
					(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
		}
	}	
	#endif*/

	//@Vasudha: To find the reuse distance(in terms of accesses) of translations accessed by only critical IPs, only non-critical IPs and both
/*	#ifdef DETECT_CRITICAL_IPS
	uint64_t check_cycle;
	if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[cpu].thread)
	{
		// 1. Find reuse dist. of translations used by only critical IPs. 
		// If criticalOrNot = 1, then find reuse distance, else ignore.
		auto search = critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != critical_reuse[cpu][thread_index].end())	
		{
			check_cycle = (search->second).last_cycle_accessed;
			// update number of unique translations accesses
			map <uint64_t, struct find_reuse>::iterator iter = critical_reuse[cpu][thread_index].begin();
			for (iter; iter != critical_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
				{
					if ((iter->second).last_cycle_accessed > check_cycle)
						(iter->second).unique_accesses += 1;
				}
			}
			(search->second).last_cycle_accessed = current_core_cycle[cpu];

			//If in past, translation is already accessed by any non-critical IP, then ignore
			if ((search->second).criticalOrNot == 1)
			{
				
				auto it = critical_ips[cpu][thread_index].find(RQ.entry[index].ip);
				// check if IP is critical or not
				if (it != critical_ips[cpu][thread_index].end())
				{
					increment_reuse(search->second, (search->second).unique_accesses);
					(search->second).unique_accesses = 0;
				}
				else
					// mark that the translation has been accessed by a non-critical IP
					(search->second).criticalOrNot = 2;
			}
			else if ((search->second).criticalOrNot == 0)
			{
				if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				{
					(search->second).criticalOrNot = 1;
					(search->second).unique_accesses = 0;
					(search->second).reuse_1_50 = 0;
					(search->second).reuse_50_500 = 0;
					(search->second).reuse_500_5000 = 0;
					(search->second).reuse_5000_50000 = 0;
					(search->second).reuse_50000 = 0;

				}	
				
			}
		}
		else
		{
			map <uint64_t, struct find_reuse>::iterator iter = critical_reuse[cpu][thread_index].begin();
			for (iter; iter != critical_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
					(iter->second).unique_accesses += 1;
				else
					assert(0);
			}
			// insert this new translation if IP is critical
			struct find_reuse Find_reuse;
			Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
			Find_reuse.criticalOrNot = 0;
			Find_reuse.unique_accesses = 0;
			Find_reuse.reuse_1_50 = 0;
			Find_reuse.reuse_50_500 = 0;
			Find_reuse.reuse_500_5000 = 0;
			Find_reuse.reuse_5000_50000 = 0;
			Find_reuse.reuse_50000 = 0;
			
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				Find_reuse.criticalOrNot = 1;	
			critical_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
					(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
		}
		
		
		// 2. Find reuse dist. of translations used by only non-critical IPs. 
		// If criticalOrNot = 0, then find reuse distance, else ignore.
		search = non_critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != non_critical_reuse[cpu][thread_index].end())	
		{
			check_cycle = (search->second).last_cycle_accessed;
			// update number of unique translations accesses
			map <uint64_t, struct find_reuse>::iterator iter = non_critical_reuse[cpu][thread_index].begin();
			for (iter; iter != non_critical_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
				{
					if ((iter->second).last_cycle_accessed > check_cycle)
						(iter->second).unique_accesses += 1;
				}
			}
			(search->second).last_cycle_accessed = current_core_cycle[cpu];

			//If in past, translation is already accessed by any critical IP, then ignore
			if ((search->second).criticalOrNot == 0)
			{
				// check if IP is non-critical or not
				if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
				{
					increment_reuse(search->second, (search->second).unique_accesses);
					(search->second).unique_accesses = 0;
				}
				else
				{
					// mark that the translation has been accessed by a critical IP
					(search->second).criticalOrNot = 1;
				}
			}
		}
		else
		{
			map <uint64_t, struct find_reuse>::iterator iter = non_critical_reuse[cpu][thread_index].begin();
			for (iter; iter != non_critical_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
					(iter->second).unique_accesses += 1;
				else
					assert(0);
			}
			// insert this new translation if IP is critical
			struct find_reuse Find_reuse;
			Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
			Find_reuse.criticalOrNot = 0;
			Find_reuse.unique_accesses = 0;
			Find_reuse.reuse_1_50 = 0;
			Find_reuse.reuse_50_500 = 0;
			Find_reuse.reuse_500_5000 = 0;
			Find_reuse.reuse_5000_50000 = 0;
			Find_reuse.reuse_50000 = 0;
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				Find_reuse.criticalOrNot = 1;
			non_critical_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> (RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
		}

		// 3. Find reuse dist. of translations used by both critical and non-critical IPs. 
		// Find reuse distance in both the cases.
		search = both_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != both_reuse[cpu][thread_index].end())	
		{
			check_cycle = (search->second).last_cycle_accessed;
			// update number of unique translations accesses
			map <uint64_t, struct find_reuse>::iterator iter = both_reuse[cpu][thread_index].begin();
			for (iter; iter != both_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
				{
					if ((iter->second).last_cycle_accessed > check_cycle)
						(iter->second).unique_accesses += 1;
				}
			}
			(search->second).last_cycle_accessed = current_core_cycle[cpu];
			increment_reuse(search->second, (search->second).unique_accesses);
			(search->second).unique_accesses = 0;

			if ((search->second).criticalOrNot == 0 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				(search->second).criticalOrNot = 1;
			if ((search->second).criticalOrNot == 1 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
				(search->second).criticalOrNot = 2;
		}
		else
		{
			map <uint64_t, struct find_reuse>::iterator iter = both_reuse[cpu][thread_index].begin();
			for (iter; iter != both_reuse[cpu][thread_index].end(); iter++)
			{
				if ((iter->first) != RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE)
					(iter->second).unique_accesses += 1;
				else
					assert(0);
			}
			// insert this new translation 
			struct find_reuse Find_reuse;
			Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
			Find_reuse.criticalOrNot = 0;
			Find_reuse.unique_accesses = 0;
			Find_reuse.reuse_1_50 = 0;
			Find_reuse.reuse_50_500 = 0;
			Find_reuse.reuse_500_5000 = 0;
			Find_reuse.reuse_5000_50000 = 0;
			Find_reuse.reuse_50000 = 0;
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				Find_reuse.criticalOrNot = 1;
			both_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
					(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
		}
	}
	#endif*/
	//@Vasudha: To find the reuse distance(in terms of cycle) of translations accessed by only critical IPs, only non-critical IPs and both
	/*#ifdef DETECT_CRITICAL_IPS
	uint64_t distance;
	if (cache_type == IS_DTLB && all_warmup_complete > ooo_cpu[cpu].thread)
	{
		// 1. Find reuse dist. of translations used by only critical IPs. 
		// If criticalOrNot = 1, then find reuse distance, else ignore.
		auto search = critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != critical_reuse[cpu][thread_index].end())	
		{
			//If in past, translation is already accessed by any non-critical IP, then ignore
			if ((search->second).criticalOrNot == 1)
			{
				// check if IP is critical or not
				if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				{
					// find reuse distance and update last cycle accessed
					distance = current_core_cycle[cpu] - (search->second).last_cycle_accessed;
					(search->second).last_cycle_accessed = current_core_cycle[cpu];
					increment_reuse(search->second, distance);
				}
				else
				{
					// mark that the translation has been accessed by a non-critical IP
					(search->second).criticalOrNot = 2;
				}
			}
		}
		else
		{
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
			{
				// insert this new translation if IP is critical
				struct find_reuse Find_reuse;
				Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
				Find_reuse.criticalOrNot = 1;
				Find_reuse.reuse_1_50 = 0;
				Find_reuse.reuse_50_500 = 0;
				Find_reuse.reuse_500_5000 = 0;
				Find_reuse.reuse_5000_50000 = 0;
				Find_reuse.reuse_50000 = 0;
				critical_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
						(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
			}
		}
		
		
		// 2. Find reuse dist. of translations used by only non-critical IPs. 
		// If criticalOrNot = 0, then find reuse distance, else ignore.
		search = non_critical_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != non_critical_reuse[cpu][thread_index].end())	
		{
			//If in past, translation is already accessed by any critical IP, then ignore
			if ((search->second).criticalOrNot == 0)
			{
				// check if IP is non-critical or not
				if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
				{
					// find reuse distance and update last cycle accessed
					distance = current_core_cycle[cpu] - (search->second).last_cycle_accessed;
					(search->second).last_cycle_accessed = current_core_cycle[cpu];
					increment_reuse(search->second, distance);
				}
				else
				{
					// mark that the translation has been accessed by a critical IP
					(search->second).criticalOrNot = 1;
				}
			}
		}
		else
		{
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
			{
				// insert this new translation if IP is critical
				struct find_reuse Find_reuse;
				Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
				Find_reuse.criticalOrNot = 0;
				Find_reuse.reuse_1_50 = 0;
				Find_reuse.reuse_50_500 = 0;
				Find_reuse.reuse_500_5000 = 0;
				Find_reuse.reuse_5000_50000 = 0;
				Find_reuse.reuse_50000 = 0;
				non_critical_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
						(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
			}
		}

		// 3. Find reuse dist. of translations used by both critical and non-critical IPs. 
		// Find reuse distance in both the cases.
		search = both_reuse[cpu][thread_index].find(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE);
		if (search != both_reuse[cpu][thread_index].end())	
		{
			// find reuse distance and update last cycle accessed
			distance = current_core_cycle[cpu] - (search->second).last_cycle_accessed;
			(search->second).last_cycle_accessed = current_core_cycle[cpu];
			increment_reuse(search->second, distance);
			if ((search->second).criticalOrNot == 0 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				(search->second).criticalOrNot = 1;
			if ((search->second).criticalOrNot == 1 && critical_ips[cpu][thread_index].find(RQ.entry[index].ip) == critical_ips[cpu][thread_index].end())
				(search->second).criticalOrNot = 2;
		}
		else
		{
			// insert this new translation 
			struct find_reuse Find_reuse;
			Find_reuse.last_cycle_accessed = current_core_cycle[cpu];
			Find_reuse.criticalOrNot = 0;
			Find_reuse.reuse_1_50 = 0;
			Find_reuse.reuse_50_500 = 0;
			Find_reuse.reuse_500_5000 = 0;
			Find_reuse.reuse_5000_50000 = 0;
			Find_reuse.reuse_50000 = 0;
			if (critical_ips[cpu][thread_index].find(RQ.entry[index].ip) != critical_ips[cpu][thread_index].end())
				Find_reuse.criticalOrNot = 1;
			both_reuse[cpu][thread_index].insert( pair<uint64_t, struct find_reuse> 
					(RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE, Find_reuse));
		}
	}
	#endif*/

	    if (cache_type == IS_DTLB && critical_translation[cpu][thread_index].find(RQ.entry[index].address) != critical_translation[cpu][thread_index].end())
	    {
	            auto it = freq_critical[cpu][thread_index].find(RQ.entry[index].address);
		    if (it != freq_critical[cpu][thread_index].end())
		    {
			    (it->second).occurence++;
			    (it->second).miss++;
		    }
		    else
			    assert(0);
	    }

            int  way_pb = -1;
	    #ifdef PUSH_DTLB_PB
            //If DTLB misses, check DTLB Prefetch Buffer
            if(cache_type == IS_DTLB && way < 0 && way_vb < 0)
            {
                    way_pb = ooo_cpu[read_cpu].DTLB_PB.check_hit( &RQ.entry[index], 0);
                    if(way_pb >= 0)
                    {
                            ooo_cpu[read_cpu].DTLB_PB.block[0][way_pb].used = 1;
                            RQ.entry[index].data_pa = ooo_cpu[read_cpu].DTLB_PB.block[0][way_pb].data;
                            ooo_cpu[read_cpu].DTLB_PB.sim_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.HIT[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.ACCESS[RQ.entry[index].type]++;
                            (&ooo_cpu[read_cpu].DTLB_PB->*update_replacement_state)(read_cpu, 0, way_pb, ooo_cpu[read_cpu].DTLB_PB.block[0][way_pb].full_addr, RQ.entry[index].ip, 0, RQ.entry[index].type, 1);
                            RQ.entry[index].data = ooo_cpu[read_cpu].DTLB_PB.block[0][way_pb].data;
                            ooo_cpu[read_cpu].DTLB_PB.pf_useful++;
                            
			    // If DTLB prefetch buffer gets hit, fill DTLB and then proceed
                            way = (this->*find_victim)(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
                            ////cout << "DTLB_PB hit "<< RQ.entry[index].instr_id << " " << ooo_cpu[read_cpu].DTLB_PB.pf_useful << endl;
			    
			    //@Vasudha: Check if the evicted translation from DTLB should enter DTLB_VB
			    if (block[set][way].valid==1)
			    {
				    uint64_t enter_VB;
				    enter_VB = dtlb_prefetcher_cache_fill(RQ.entry[index].address, set, way, 1, block[set][way].address, RQ.entry[index].pf_metadata, ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id);
				    #ifdef PUSH_VICTIMS_DTLB_VB
				    if (enter_VB == 1)
				    {	    
					uint32_t victim_way;
					victim_way = (&ooo_cpu[read_cpu].DTLB_VB->*find_victim)(cpu, 0, 0, ooo_cpu[read_cpu].DTLB_VB.block[0], 0, block[set][way].full_addr, 0);
					#ifdef DEBUG
					//if (victim_way >= 0 && ooo_cpu[cpu].DTLB_VB.block[0][victim_way].used != 1)
					//	cout << "DTLB_VB block - NOT USED: " << hex << ooo_cpu[cpu].DTLB_VB.block[0][victim_way].address << " Cycle: " << dec << current_core_cycle[cpu] << endl;
					#endif
					if (victim_way >= 0)
						dtlb_victim_buffer_fill(block[set][way].address, 0, victim_way, block[set][way].prefetch, ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address, (uint32_t)ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].used, ((1 << LOG2_THREADS) - 1) & ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].instr_id );
					#ifdef DEBUG
					if (victim_way >= 0)
						cout << "Cycle: " <<dec<<current_core_cycle[cpu] << " T" << (((1<<LOG2_THREADS)-1) & ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].instr_id) <<
							" evicted_addr:from VB " <<hex<< ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address << " evicted_region: " <<
							(ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address >> 13) << endl;
					#endif
					(&ooo_cpu[read_cpu].DTLB_VB->*update_replacement_state)(cpu, 0, victim_way, block[set][way].full_addr, 0, ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].full_addr, 0, 0);
					PACKET evicted_block;
					evicted_block.instr_id = block[set][way].instr_id;
					evicted_block.address = block[set][way].address;
					evicted_block.full_addr = block[set][way].full_addr;
					evicted_block.data = block[set][way].data;
					evicted_block.ip = block[set][way].ip;
					evicted_block.cpu = block[set][way].cpu;
					ooo_cpu[read_cpu].DTLB_VB.fill_cache(0, victim_way, &evicted_block);
				    }
				    #endif
			    }
                            RQ.entry[index].type = PREFETCH_TRANSLATION;
			    fill_cache(set, way, &RQ.entry[index]);
			    ooo_cpu[read_cpu].DTLB_PB.block[0][way_pb].valid = 0;
                    		
		    }
                    else
                    {
                            //DTLB_PB MISS
                              ooo_cpu[read_cpu].DTLB_PB.MISS[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.ACCESS[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.sim_miss[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_PB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                    }
            }
	    #endif
	
            int  way_ub = -1;
	    #ifdef PUSH_DTLB_UB
            //If DTLB VB and PB misses, check DTLB Utility Buffer
            if(cache_type == IS_DTLB && way < 0 && way_vb < 0 && way_pb < 0)
            {
                    way_ub = ooo_cpu[read_cpu].DTLB_UB.check_hit( &RQ.entry[index], 0);
                    if(way_ub >= 0)
                    {
                            ooo_cpu[read_cpu].DTLB_UB.block[0][way_ub].used = 1;
                            RQ.entry[index].data_pa = ooo_cpu[read_cpu].DTLB_UB.block[0][way_ub].data;
                            ooo_cpu[read_cpu].DTLB_UB.sim_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.HIT[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.ACCESS[RQ.entry[index].type]++;
                            (&ooo_cpu[read_cpu].DTLB_UB->*update_replacement_state)(read_cpu, 0, way_ub, ooo_cpu[read_cpu].DTLB_UB.block[0][way_ub].full_addr, RQ.entry[index].ip, 0, RQ.entry[index].type, 1);
                            RQ.entry[index].data = ooo_cpu[read_cpu].DTLB_UB.block[0][way_ub].data;
                            ooo_cpu[read_cpu].DTLB_UB.pf_useful++;
                            
			    // If DTLB prefetch buffer gets hit, fill DTLB and then proceed
                            way = (this->*find_victim)(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
                            ////cout << "DTLB_PB hit "<< RQ.entry[index].instr_id << " " << ooo_cpu[read_cpu].DTLB_PB.pf_useful << endl;
			    
			    //@Vasudha: Check if the evicted translation from DTLB should enter DTLB_VB
			    if (block[set][way].valid==1)
			    {
				    uint64_t enter_VB;
				    enter_VB = dtlb_prefetcher_cache_fill(RQ.entry[index].address, set, way, 1, block[set][way].address, RQ.entry[index].pf_metadata, ((1 << LOG2_THREADS) - 1) & block[set][way].instr_id);
				    #ifdef PUSH_VICTIMS_DTLB_VB
				    if (enter_VB == 1)
				    {	    
					uint32_t victim_way;
					victim_way = (&ooo_cpu[read_cpu].DTLB_VB->*find_victim)(cpu, 0, 0, ooo_cpu[read_cpu].DTLB_VB.block[0], 0, block[set][way].full_addr, 0);
					#ifdef DEBUG
					//if (victim_way >= 0 && ooo_cpu[cpu].DTLB_VB.block[0][victim_way].used != 1)
					//	cout << "DTLB_VB block - NOT USED: " << hex << ooo_cpu[cpu].DTLB_VB.block[0][victim_way].address << " Cycle: " << dec << current_core_cycle[cpu] << endl;
					#endif
					if (victim_way >= 0)
						dtlb_victim_buffer_fill(block[set][way].address, 0, victim_way, block[set][way].prefetch, ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address, (uint32_t)ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].used, ((1 << LOG2_THREADS) - 1) & ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].instr_id );
					#ifdef DEBUG
					if (victim_way >= 0)
						cout << "Cycle: " <<dec<<current_core_cycle[cpu] << " T" << (((1<<LOG2_THREADS)-1) & ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].instr_id) <<
							" evicted_addr:from VB " <<hex<< ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address << " evicted_region: " <<
							(ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].address >> 13) << endl;
					#endif
					(&ooo_cpu[read_cpu].DTLB_VB->*update_replacement_state)(cpu, 0, victim_way, block[set][way].full_addr, 0, ooo_cpu[read_cpu].DTLB_VB.block[0][victim_way].full_addr, 0, 0);
					PACKET evicted_block;
					evicted_block.instr_id = block[set][way].instr_id;
					evicted_block.address = block[set][way].address;
					evicted_block.full_addr = block[set][way].full_addr;
					evicted_block.data = block[set][way].data;
					evicted_block.ip = block[set][way].ip;
					evicted_block.cpu = block[set][way].cpu;
					ooo_cpu[read_cpu].DTLB_VB.fill_cache(0, victim_way, &evicted_block);
				    }
				    #endif
			    }
                            RQ.entry[index].type = LOAD_TRANSLATION;
			    fill_cache(set, way, &RQ.entry[index]);
			    ooo_cpu[read_cpu].DTLB_UB.block[0][way_ub].valid = 0;
                    		
		    }
                    else
                    {
                            //DTLB_UB MISS
                             ooo_cpu[read_cpu].DTLB_UB.MISS[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.ACCESS[RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.sim_miss[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                            ooo_cpu[read_cpu].DTLB_UB.sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                    }
            }
	    #endif


	    if (way >= 0) { // read hit

				
                if (cache_type == IS_ITLB) {
			
                    //RQ.entry[index].instruction_pa = (va_to_pa(read_cpu, RQ.entry[index].instr_id, RQ.entry[index].full_addr, RQ.entry[index].address))>>LOG2_PAGE_SIZE; //block[set][way].data;
		    RQ.entry[index].instruction_pa = block[set][way].data;
		    //RQ.entry[index].event_cycle = current_core_cycle[read_cpu];
                    if (PROCESSED.occupancy < PROCESSED.SIZE)
                        PROCESSED.add_queue(&RQ.entry[index]);
                }
                else if (cache_type == IS_DTLB) {
                    //RQ.entry[index].data_pa = (va_to_pa(read_cpu, RQ.entry[index].instr_id, RQ.entry[index].full_addr, RQ.entry[index].address))>>LOG2_PAGE_SIZE;  //block[set][way].data;
		    //if (warmup_complete[cpu][thread_index])
			//cout << "Cycle: " << current_core_cycle[0] << " ADDRESS: " << RQ.entry[index].address << " way= " << way << endl;
		   
		    if (way != NUM_WAY)
			RQ.entry[index].data_pa = block[set][way].data;
		   #if defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES) && !defined(DETECT_CRITICAL_TRANSLATIONS) && !defined(DETECT_CRITICAL_IPS)
		    else
		    {
			    auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find( RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE );
			    if (it == ooo_cpu[cpu].PTW.page_table[thread_index].end())
				    assert(0);
			    RQ.entry[index].data_pa = it->second;
		    }
		   #endif

		   #if defined(PERFECT_DTLB) || defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES)  
		   #if    defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
		    else
		    {
			    assert(warmup_complete[cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id]);
			    auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find( RQ.entry[index].full_virtual_address >> LOG2_PAGE_SIZE );
			    if (it == ooo_cpu[cpu].PTW.page_table[thread_index].end())
				    assert(0);
			    RQ.entry[index].data_pa = it->second;
		    }
		    #endif
		    #endif
		    //@Vasudha: Perfect DTLB Prefetcher
		    #ifdef PERFECT_DTLB
		    #if    !defined(DETECT_CRITICAL_IPS) && !defined(DETECT_CRITICAL_TRANSLATIONS)
                    auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find(RQ.entry[index].full_addr >> LOG2_PAGE_SIZE);
		    if(it == ooo_cpu[cpu].PTW.page_table[thread_index].end())
			    assert(0);
		    RQ.entry[index].data_pa = it->second;
		    #endif
		    #endif			
	        	if (PROCESSED.occupancy < PROCESSED.SIZE)
                        PROCESSED.add_queue(&RQ.entry[index]);
			
                }
                else if (cache_type == IS_STLB) 
                {

		 	#if defined(PERFECT_STLB) || defined(PERFECT_STLB_EXCLUDING_COLD_MISSES)
			#if !defined(DETECT_CRITICAL_IPS) && !defined(DETECT_CRITICAL_TRANSLATIONS) 
				thread_index = ((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id;
				if (knob_cloudsuite)
					thread_index = 0;
				auto it = ooo_cpu[cpu].PTW.page_table[thread_index].find(RQ.entry[index].full_addr >> LOG2_PAGE_SIZE);

				if (it == ooo_cpu[cpu].PTW.page_table[thread_index].end())
					assert(0);
				block[set][way].data = it->second;
			#endif
			#endif
			
			PACKET temp = RQ.entry[index];
			
                    if (temp.l1_pq_index != -1) //@Vishal: Check if the current request is sent from L1D prefetcher //TODO: Add condition to not send instruction translation request
                    {
                    	assert(RQ.entry[index].l1_rq_index == -1 && RQ.entry[index].l1_wq_index == -1);//@Vishal: these should not be set
        				        		
						temp.data_pa = block[set][way].data;
						temp.read_translation_merged = false;
						temp.write_translation_merged = false;
                    	if (PROCESSED.occupancy < PROCESSED.SIZE)
                        	PROCESSED.add_queue(&temp);
                        else
                        	assert(0);
                    }
			if(RQ.entry[index].prefetch_translation_merged) //@Vishal: Prefetech request from L1D prefetcher
			{
				PACKET temp = RQ.entry[index];
				temp.data_pa = block[set][way].data;
				temp.read_translation_merged = 0; //@Vishal: Remove this before adding to PQ
				temp.write_translation_merged = 0;
					if (PROCESSED.occupancy < PROCESSED.SIZE)
						PROCESSED.add_queue(&temp);
					else
						assert(0);
			}

                }
                else if (cache_type == IS_L1I) {
		   if (PROCESSED.occupancy < PROCESSED.SIZE)
                        PROCESSED.add_queue(&RQ.entry[index]);
                }

		//@Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)		
               else if ((cache_type == IS_L1D) &&
                        (RQ.entry[index].type != PREFETCH) &&
                        (RQ.entry[index].type != LOAD_TRANSLATION) &&
                        (RQ.entry[index].type != PREFETCH_TRANSLATION) &&
                        (RQ.entry[index].type != TRANSLATION_FROM_L1D))
#else
               else if ((cache_type == IS_L1D) &&
                        (RQ.entry[index].type != PREFETCH))
#endif
               {
                   if (PROCESSED.occupancy < PROCESSED.SIZE)
                        PROCESSED.add_queue(&RQ.entry[index]);
               }
               /* else if ((cache_type == IS_L1D) && (RQ.entry[index].type != PREFETCH)) {
                    if (PROCESSED.occupancy < PROCESSED.SIZE)
                        PROCESSED.add_queue(&RQ.entry[index]);
                }*/


				
                // update prefetcher on load instruction
		if (RQ.entry[index].type == LOAD) {
			assert(cache_type != IS_ITLB || cache_type != IS_DTLB || cache_type != IS_STLB);
		    if(cache_type == IS_L1I)
 			    l1i_prefetcher_cache_operate(read_cpu, RQ.entry[index].ip, 1, block[set][way].prefetch);
                    if (cache_type == IS_L1D) 
		      l1d_prefetcher_operate(RQ.entry[index].full_addr, RQ.entry[index].ip, 1, RQ.entry[index].type, RQ.entry[index].critical_ip_flag);	// RQ.entry[index].instr_id);
                    else if ((cache_type == IS_L2C) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].instruction == 0) && (RQ.entry[index].type != LOAD_TRANSLATION) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].type == TRANSLATION_FROM_L1D))	//Neelu: for dense region, only invoking on loads, check other l2c_pref_operate as well. 
		      l2c_prefetcher_operate(block[set][way].address<<LOG2_BLOCK_SIZE, RQ.entry[index].ip, 1, RQ.entry[index].type, 0);	// RQ.entry[index].instr_id);
		    else if (cache_type == IS_LLC)
		      {
			cpu = read_cpu;
			llc_prefetcher_operate(block[set][way].address<<LOG2_BLOCK_SIZE, RQ.entry[index].ip, 1, RQ.entry[index].type, 0);
			cpu = 0;
		      }
		}
		else if(RQ.entry[index].type == LOAD_TRANSLATION) {
		      assert(cache_type != IS_L1I || cache_type != IS_L1D || cache_type != IS_L2C || cache_type != IS_LLC);
		      if (cache_type == IS_ITLB)
		      {
		      	itlb_prefetcher_operate(RQ.entry[index].address<<LOG2_PAGE_SIZE, RQ.entry[index].ip, 1, RQ.entry[index].type, RQ.entry[index].instr_id, RQ.entry[index].instruction);
		      	
		      }
		      else if (cache_type == IS_DTLB)
		      {
		      #ifdef SANITY_CHECK
    			if(RQ.entry[index].instruction)
    			{
    				//cout << "DTLB prefetch packet should not prefetch address translation of instruction"<< endl;
    				assert(0);
    			}
			#endif
		          	
		      	dtlb_prefetcher_operate(RQ.entry[index].address, RQ.entry[index].ip, 1, RQ.entry[index].type, RQ.entry[index].instr_id, RQ.entry[index].instruction, RQ.entry[index].critical_ip_flag);
		      	
		      	
		      }
		      else if (cache_type == IS_STLB)
		      {
			      int temp_type = LOAD_TRANSLATION;
			      if(RQ.entry[index].prefetch_translation_merged == true || RQ.entry[index].l1_pq_index != -1)
				      temp_type = TRANSLATION_FROM_L1D;
		      	stlb_prefetcher_operate(RQ.entry[index].address, RQ.entry[index].ip, 1, temp_type, RQ.entry[index].instr_id, RQ.entry[index].instruction);
		      	
		      }
                }

		#if defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES) || defined(PERFECT_DTLB)
		if ((cache_type == IS_DTLB && way != NUM_WAY) || cache_type != IS_DTLB)
		#endif

		//if (!(cache_type == IS_STLB && block[set][way].prefetch))
                // update replacement policy
                    (this->*update_replacement_state)(read_cpu, set, way, block[set][way].full_addr, RQ.entry[index].ip, 0, RQ.entry[index].type, 1);

                // COLLECT STATS
                sim_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;

#ifdef PT_STATS
      		if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && RQ.entry[index].type == LOAD_TRANSLATION)
		{
			assert(RQ.entry[index].translation_level > 0 && RQ.entry[index].translation_level < 6);
		        sim_pt_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].translation_level-1]++;
		        sim_pt_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].translation_level-1]++;
      		}
#endif



                // check fill level
                // data should be updated (for TLBs) in case of hit
                if (RQ.entry[index].fill_level < fill_level) {

                    
			if(cache_type == IS_STLB)
			{
				RQ.entry[index].prefetch_translation_merged = false;
				
				//@Vasudha: To Bypass DTLB
				//if (block[set][way].prefetch)
				//	RQ.entry[index].type = PREFETCH_TRANSLATION;
				if(RQ.entry[index].send_both_tlb)
				{
					RQ.entry[index].data = block[set][way].data;
					upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
					upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);
				}
				else if (RQ.entry[index].instruction)
				{
					RQ.entry[index].data = block[set][way].data;
					upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
				}
				else // data
				{	
					RQ.entry[index].data = block[set][way].data;
					upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);
				}
				#ifdef SANITY_CHECK
                                if(RQ.entry[index].data == 0)
                                         assert(0);
                                 #endif
			}

			//@Rahul: PTW
#ifdef PTW_L1D
			else if(cache_type == IS_L1D && (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D))
		       	{
				PTW_interface[read_cpu]->return_data(&RQ.entry[index]);
			}
#endif

#ifdef PTW_L2C
                        else if(cache_type == IS_L2C && (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D)) 
                        {
                                PTW_interface[read_cpu]->return_data(&RQ.entry[index]);
                        }
#endif  
#ifdef PTW_LLC
                        else if(cache_type == IS_LLC && (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D)) 
                        {
                                PTW_interface[read_cpu]->return_data(&RQ.entry[index]);
                        }
#endif  			
#ifdef PTW_L1D_L2C
                       else if((RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D) &&
                               ((cache_type == IS_L1D && RQ.entry[index].translation_level > 1) || (cache_type == IS_L2C && RQ.entry[index].translation_level == 1)))
		       {
                               assert(RQ.entry[index].translation_level > 0 && RQ.entry[index].translation_level < 6);
                               PTW_interface[read_cpu]->return_data(&RQ.entry[index]);
                       }
#endif
			/*
			else if(cache_type == IS_L2C && (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D))
                    	{
                        	extra_interface->return_data(&RQ.entry[index]);
			}*/
			 else if(cache_type == IS_L2C)
			 {
				 if(RQ.entry[index].send_both_cache)
				 {
					 upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
					 upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);
				 }
				 else if(RQ.entry[index].fill_l1i || RQ.entry[index].fill_l1d)
				 {
					if(RQ.entry[index].fill_l1i)
					 	upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
					if(RQ.entry[index].fill_l1d)
						upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);	
				 }
				 else if (RQ.entry[index].instruction)
					 upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
				 else if (RQ.entry[index].is_data)
					 upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);
			 }
			else	
			{
			    if (RQ.entry[index].instruction) 
			    {
				RQ.entry[index].data = block[set][way].data;
				upper_level_icache[read_cpu]->return_data(&RQ.entry[index]);
			    }
			    else // data
			    {
				RQ.entry[index].data = block[set][way].data;
				upper_level_dcache[read_cpu]->return_data(&RQ.entry[index]);
			    }
			    #ifdef SANITY_CHECK
			    if(cache_type == IS_ITLB || cache_type == IS_DTLB)
			    	if(RQ.entry[index].data == 0)
					assert(0);
			    #endif
			}
                }

                // update prefetch stats and reset prefetch bit
                if (block[set][way].prefetch) {
                    pf_useful++;
		    #ifdef DEBUG_PREF
		    if( cache_type == IS_STLB)
			   cout << hex<<"USEFUL PREFETCH: T" <<(((1<<LOG2_THREADS)-1) & RQ.entry[index].instr_id)<< " addr: " << RQ.entry[index].address  << endl; 
		    #endif
		    block[set][way].prefetch = 0;

		    //Neelu: IPCP prefetch stats
		    if(block[set][way].pref_class < 5)
			    pref_useful[cpu][block[set][way].pref_class]++;

                }
		if ((cache_type == IS_DTLB || cache_type == IS_STLB) && block[set][way].used == 2);
		else if ((cache_type == IS_DTLB || cache_type == IS_STLB) && block[set][way].used == 1)
			block[set][way].used = 2;
		else
               		block[set][way].used = 1;

		//if ((cache_type == IS_DTLB || cache_type == IS_STLB) && block[set][way].critical && RQ.entry[index].critical_ip_flag != 1 && block[set][way].used==2)
		//	block[set][way].critical = 0;

                HIT[RQ.entry[index].type]++;
                ACCESS[RQ.entry[index].type]++;
                
                // remove this entry from RQ
                RQ.remove_queue(&RQ.entry[index]);
		reads_available_this_cycle--;
            }
            else 
	    #ifdef PUSH_VICTIMS_DTLB_VB
	    if (cache_type == IS_DTLB && way_vb >= 0)
	    {
    		reads_available_this_cycle--; 
	    }
	    else
	    #endif
	    { // read miss

	    if (cache_type == IS_STLB)
	    {
		    auto it = crit_ip_stlb_miss[read_cpu][thread_index].find(RQ.entry[index].ip);
		    if (it != crit_ip_stlb_miss[read_cpu][thread_index].end() && (it->second) < 1000)
		    {
			    (it->second)++;
		    }
	    }
#ifdef IDEAL_CACHE_FOR_TRANSLATION_ACCESS
#ifdef IDEAL_L1D
            if (cache_type == IS_L1D &&
                (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(RQ.entry[index].full_physical_address >> LOG2_BLOCK_SIZE) != unique_translation_access.end() &&
                stall_rq_index.find(index) == stall_rq_index.end())
#endif

#ifdef IDEAL_L2
            if (cache_type == IS_L2C && 
	        (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(RQ.entry[index].address) != unique_translation_access.end() &&
                stall_rq_index.find(index) == stall_rq_index.end() )
#endif

#ifdef IDEAL_L3
            if (cache_type == IS_LLC && 
                (RQ.entry[index].type == PREFETCH_TRANSLATION || RQ.entry[index].type == LOAD_TRANSLATION || RQ.entry[index].type == TRANSLATION_FROM_L1D) &&
                unique_translation_access.find(RQ.entry[index].address) != unique_translation_access.end() &&
                stall_rq_index.find(index) == stall_rq_index.end() )
#endif	    
            {
         	 	PTW_interface[read_cpu]->return_data(&RQ.entry[index]);

	                // COLLECT STATS
           		//change_miss_to_hit++;
			sim_hit[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
                        sim_access[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id][RQ.entry[index].type]++;
          		HIT[RQ.entry[index].type]++;
          		ACCESS[RQ.entry[index].type]++;
            }
#endif

		    
               /*DP ( if (warmup_complete[read_cpu][((1 << LOG2_THREADS) - 1) & RQ.entry[index].instr_id] ) {
		cout << "[" << NAME << "] " << __func__ << " read miss";
                cout << " instr_id: " << (RQ.entry[index].instr_id >> LOG2_THREADS) << " address: " << hex << RQ.entry[index].address;
                cout << " full_addr: " << RQ.entry[index].full_addr << dec;
                cout << " cycle: " << RQ.entry[index].event_cycle << " data: " << hex<< RQ.entry[index].data << " data_pa: " << RQ.entry[index].data_pa <<dec<<endl; });
		*/
		    //if (warmup_complete[cpu][thread_index] && cache_type == IS_DTLB)
		//	cout << "Cycle: " << current_core_cycle[0] << " DTLB MISS ADDRESS: " << RQ.entry[index].address << " way= " << way << endl;


                // check mshr
                uint8_t miss_handled = 1;
                int mshr_index = check_nonfifo_queue(&MSHR, &RQ.entry[index],false); //@Vishal: Updated from check_mshr

		if(mshr_index == -2)
		{
			// this is a data/instruction collision in the MSHR, so we have to wait before we can allocate this miss
			miss_handled = 0;
		}

                if ((mshr_index == -1) && (MSHR.occupancy < MSHR_SIZE)) { // this is a new miss

		  if(cache_type == IS_STLB && MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) 
			  pf_lower_level++;

		  if(cache_type == IS_LLC)
		    {
		      // check to make sure the DRAM RQ has room for this LLC read miss
		      if (lower_level->get_occupancy(1, RQ.entry[index].address) == lower_level->get_size(1, RQ.entry[index].address))
			{
			  miss_handled = 0;
			}
		      else
			{

				  add_nonfifo_queue(&MSHR, &RQ.entry[index]); //@Vishal: Updated from add_mshr
				  if(lower_level)
				    {
				      lower_level->add_rq(&RQ.entry[index]);
				    }
			}
		    }
		  else
		    {

		      if(cache_type == IS_L1I || cache_type == IS_L1D) //@Vishal: VIPT
			  {
			  	 assert(RQ.entry[index].full_physical_address != 0);
				 //Neelu: Added to processed queue even on miss for ideal L1 prefetcher, comment if not needed. 
				/* if ((cache_type == IS_L1D) && (RQ.entry[index].type != PREFETCH)) {
					 if (PROCESSED.occupancy < PROCESSED.SIZE)
						 PROCESSED.add_queue(&RQ.entry[index]);
   	                         }*/
			  	 PACKET new_packet = RQ.entry[index];
			  	 //@Vishal: Send physical address to lower level and track physical address in MSHR  
			  	 new_packet.address = RQ.entry[index].full_physical_address >> LOG2_BLOCK_SIZE;
			  	 new_packet.full_addr = RQ.entry[index].full_physical_address; 

			  	 add_nonfifo_queue(&MSHR, &new_packet); //@Vishal: Updated from add_mshr
				 lower_level->add_rq(&new_packet);
			  }
			  else
			  {

				  //@Vasudha: Perfect DTLB Prefetcher: add to processed queue even on miss
                                if (cache_type == IS_DTLB && RQ.entry[index].type != PREFETCH)
                                {
                                        //RQ.entry[index].data_pa = (va_to_pa(read_cpu, RQ.entry[index].instr_id, RQ.entry[index].full_addr, RQ.entry[index].address));
                                        /*auto it = temp_page_table.find(RQ.entry[index].full_addr >> LOG2_PAGE_SIZE);
					assert(it != temp_page_table.end());
					RQ.entry[index].data_pa = it->second;
					if (PROCESSED.occupancy < PROCESSED.SIZE)
                                                PROCESSED.add_queue(&RQ.entry[index]);*/
                                }

		      // add it to mshr (read miss)
		      add_nonfifo_queue(&MSHR, &RQ.entry[index]); //@Vishal: Updated from add_mshr
		      
		      // add it to the next level's read queue
		      if (lower_level)
                        lower_level->add_rq(&RQ.entry[index]);
		      else { // this is the last level
#ifdef INS_PAGE_TABLE_WALKER
			      assert(0);
#else
	              if (cache_type == IS_STLB) {
				  // TODO: need to differentiate page table walk and actual swap
				  
				  // emulate page table walk
				  uint64_t pa = va_to_pa(read_cpu, RQ.entry[index].instr_id, RQ.entry[index].full_addr, RQ.entry[index].address);
				  RQ.entry[index].data = pa >> LOG2_PAGE_SIZE; 
				  RQ.entry[index].event_cycle = current_core_cycle[read_cpu];

				  if (RQ.entry[index].l1_pq_index != -1) //@Vishal: Check if the current request is sent from L1D prefetcher
	                {
	                	assert(RQ.entry[index].l1_rq_index == -1 && RQ.entry[index].l1_wq_index == -1);//@Vishal: these should not be set

	                	RQ.entry[index].data_pa = pa >> LOG2_PAGE_SIZE;
	                	if (PROCESSED.occupancy < PROCESSED.SIZE)
	                    	PROCESSED.add_queue(&RQ.entry[index]);
	                    else
	                    	assert(0);
	                }

				  return_data(&RQ.entry[index]);
	                        }
#endif
			      }
		  		}
		    }
                }
                else {
                    if ((mshr_index == -1) && (MSHR.occupancy == MSHR_SIZE)) { // not enough MSHR resource
                        
                        // cannot handle miss request until one of MSHRs is available
                        miss_handled = 0;
                        STALL[RQ.entry[index].type]++;

#ifdef IDEAL_CACHE_FOR_TRANSLATION_ACCESS
 		        stall_rq_index.insert(index);
#endif

                    }
                    else if (mshr_index != -1) { // already in-flight miss
			//@Vasudha:Perfect DTLB Prefetcher: add to processed queue even on miss
                                /*if (cache_type == IS_DTLB && RQ.entry[index].type != PREFETCH)
                                {
                                        //RQ.entry[index].data_pa = (va_to_pa(read_cpu, RQ.entry[index].instr_id, RQ.entry[index].full_addr, RQ.entry[index].address)); 
                                        auto it = temp_page_table.find(RQ.entry[index].full_addr >> LOG2_PAGE_SIZE);
					assert(it != temp_page_table.end());
					RQ.entry[index].data_pa = it->second;
                                        if (PROCESSED.occupancy < PROCESSED.SIZE)
                                                PROCESSED.add_queue(&RQ.entry[index]);
                                }*/	

                        // mark merged consumer
                        if (RQ.entry[index].type == RFO) {

                            if (RQ.entry[index].tlb_access) {
                                uint32_t sq_index = RQ.entry[index].sq_index;
                                MSHR.entry[mshr_index].store_merged = 1;
                                MSHR.entry[mshr_index].sq_index_depend_on_me.insert (sq_index);
				MSHR.entry[mshr_index].sq_index_depend_on_me.join (RQ.entry[index].sq_index_depend_on_me, SQ_SIZE);
                            }

                            if (RQ.entry[index].load_merged) {
                                //uint32_t lq_index = RQ.entry[index].lq_index; 
                                MSHR.entry[mshr_index].load_merged = 1;
                                //MSHR.entry[mshr_index].lq_index_depend_on_me[lq_index] = 1;
				MSHR.entry[mshr_index].lq_index_depend_on_me.join (RQ.entry[index].lq_index_depend_on_me, LQ_SIZE);
                            }
							
                        }
                        else {
                            if (RQ.entry[index].instruction) {
                                uint32_t rob_index = RQ.entry[index].rob_index;
                              DP (if (warmup_complete[MSHR.entry[mshr_index].cpu][thread_index] ) {
                                if(cache_type==IS_ITLB || cache_type==IS_DTLB || cache_type==IS_STLB)
                                cout << "read request merged with MSHR entry -"<< MSHR.entry[mshr_index].type << endl; });
                                MSHR.entry[mshr_index].instr_merged = 1;
                                MSHR.entry[mshr_index].rob_index_depend_on_me.insert (rob_index);

                              //DP (if (warmup_complete[MSHR.entry[mshr_index].cpu] ) {
                                //cout << "[INSTR_MERGED] " << __func__ << " cpu: " << MSHR.entry[mshr_index].cpu << " instr_id: " << MSHR.entry[mshr_index].instr_id;
                                //cout << " merged rob_index: " << rob_index << " instr_id: " << RQ.entry[index].instr_id << endl; });

                                if (RQ.entry[index].instr_merged) {
				    MSHR.entry[mshr_index].rob_index_depend_on_me.join (RQ.entry[index].rob_index_depend_on_me, ROB_SIZE);
                                  //DP (if (warmup_complete[MSHR.entry[mshr_index].cpu] ) {
                                    //cout << "[INSTR_MERGED] " << __func__ << " cpu: " << MSHR.entry[mshr_index].cpu << " instr_id: " << MSHR.entry[mshr_index].instr_id;
                                    //cout << " merged rob_index: " << i << " instr_id: N/A" << endl; });
                                }
                            }
                            else 
                            {
  			        //Neelu: Added to processed queue even on miss for ideal L1 prefetcher, comment if not needed. 
				/*if ((cache_type == IS_L1D) && (RQ.entry[index].type != PREFETCH)) {
                    			if (PROCESSED.occupancy < PROCESSED.SIZE)
		                        	PROCESSED.add_queue(&RQ.entry[index]);
	                	} */

			        uint32_t lq_index = RQ.entry[index].lq_index;
                                MSHR.entry[mshr_index].load_merged = 1;
                                MSHR.entry[mshr_index].lq_index_depend_on_me.insert (lq_index);

                              //DP (if (warmup_complete[read_cpu] ) {
                                //cout << "[DATA_MERGED] " << __func__ << " cpu: " << read_cpu << " instr_id: " << RQ.entry[index].instr_id;
                                //cout << " merged rob_index: " << RQ.entry[index].rob_index << " instr_id: " << RQ.entry[index].instr_id << " lq_index: " << RQ.entry[index].lq_index << endl; });
				MSHR.entry[mshr_index].lq_index_depend_on_me.join (RQ.entry[index].lq_index_depend_on_me, LQ_SIZE);
                                if (RQ.entry[index].store_merged) {
                                    MSHR.entry[mshr_index].store_merged = 1;
				    MSHR.entry[mshr_index].sq_index_depend_on_me.join (RQ.entry[index].sq_index_depend_on_me, SQ_SIZE);
                                }
                            }
                        }

			//@Vasudha:request coming from both DTLB and ITLB should be returned to both
			if(cache_type == IS_STLB)
			{
				if(RQ.entry[index].fill_level == 1 && MSHR.entry[mshr_index].fill_level == 1)
					if(RQ.entry[index].instruction != MSHR.entry[mshr_index].instruction)
					{
						RQ.entry[index].send_both_tlb = 1;	//RQ will be overwritten to MSHR
						MSHR.entry[mshr_index].send_both_tlb = 1;	//If RQ in not overwritten to MSHR
					}
			}
                        // update fill_level
                        if (RQ.entry[index].fill_level < MSHR.entry[mshr_index].fill_level)
			{
			    	MSHR.entry[mshr_index].fill_level = RQ.entry[index].fill_level;
				MSHR.entry[mshr_index].instruction = RQ.entry[index].instruction;
			}
				

			if((RQ.entry[index].fill_l1i) && (MSHR.entry[mshr_index].fill_l1i != 1))
			{
				MSHR.entry[mshr_index].fill_l1i = 1;
			}
			if((RQ.entry[index].fill_l1d) && (MSHR.entry[mshr_index].fill_l1d != 1))
			{
				MSHR.entry[mshr_index].fill_l1d = 1;
			}

			bool merging_already_done = false;


			/*if(cache_type == IS_STLB)
			 {
				 // Fill level of incoming request and prefetch packet should be same else STLB prefetch request(with instruction=1) might get          merged with DTLB/ITLB, making send_both_tlb=1 due to a msimatch in instruction variable. If this happens, data will be returned to           both ITLB and DTLB, incurring MSHR miss
				 if(MSHR.entry[mshr_index].fill_level == 1 && RQ.entry[index].fill_level == 1)
				 {
					 if((MSHR.entry[mshr_index].instruction != RQ.entry[index].instruction) && MSHR.entry[mshr_index].send_both_tlb == 0)
					 {
						 MSHR.entry[mshr_index].send_both_tlb = 1;
					 }
					 if((MSHR.entry[mshr_index].instruction != RQ.entry[index].instruction) && MSHR.entry[mshr_index].type == PREFETCH)
					 {
						 RQ.entry[index].send_both_tlb = 1;
					 }
				 }
			 }*/


                        // update request
                        if ((MSHR.entry[mshr_index].type == PREFETCH && RQ.entry[index].type != PREFETCH) ||(MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION && RQ.entry[index].type != PREFETCH_TRANSLATION) || (MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D && RQ.entry[index].type != TRANSLATION_FROM_L1D )) {

			     merging_already_done = true;
                             uint8_t  prior_returned = MSHR.entry[mshr_index].returned;
                             uint64_t prior_event_cycle = MSHR.entry[mshr_index].event_cycle;
                             uint64_t prior_data;
                           
			     uint64_t prior_address = MSHR.entry[mshr_index].address;
                             uint64_t prior_full_addr = MSHR.entry[mshr_index].full_addr;
                             uint64_t prior_full_physical_address = MSHR.entry[mshr_index].full_physical_address;
			     uint8_t prior_fill_l1i = MSHR.entry[mshr_index].fill_l1i;
			     uint8_t prior_fill_l1d = MSHR.entry[mshr_index].fill_l1d;
			 
                            if(cache_type==IS_ITLB || cache_type==IS_DTLB || cache_type==IS_STLB)
                            {
                            	/* data(translation) should be copied in case of TLB if MSHR entry is completed and is not filled in cache yet */
                            	if(MSHR.entry[mshr_index].returned == COMPLETED)
                            	{
                            		prior_data = MSHR.entry[mshr_index].data;
                            	}
							
//@Vishal: Copy previous data from MSHR


			    if(MSHR.entry[mshr_index].read_translation_merged)
		    	    {
                                RQ.entry[index].read_translation_merged = 1;
                                RQ.entry[index].l1_rq_index_depend_on_me.join(MSHR.entry[mshr_index].l1_rq_index_depend_on_me, RQ_SIZE);
                            }

                            if(MSHR.entry[mshr_index].write_translation_merged)
                            {
                                RQ.entry[index].write_translation_merged = 1;
                                RQ.entry[index].l1_wq_index_depend_on_me.join(MSHR.entry[mshr_index].l1_wq_index_depend_on_me, WQ_SIZE);
                            }

                            if(MSHR.entry[mshr_index].prefetch_translation_merged)
                            {
                                RQ.entry[index].prefetch_translation_merged = 1;
                                RQ.entry[index].l1_pq_index_depend_on_me.join(MSHR.entry[mshr_index].l1_pq_index_depend_on_me, PQ_SIZE);
                            }

                            if(MSHR.entry[mshr_index].l1_rq_index != -1)
                            {
                                assert((MSHR.entry[mshr_index].l1_wq_index == -1) && (MSHR.entry[mshr_index].l1_pq_index == -1));
                                RQ.entry[index].read_translation_merged = 1;
                                RQ.entry[index].l1_rq_index_depend_on_me.insert(MSHR.entry[mshr_index].l1_rq_index);
                            }

                            if(MSHR.entry[mshr_index].l1_wq_index != -1)
                            {
                                assert((MSHR.entry[mshr_index].l1_rq_index == -1) && (MSHR.entry[mshr_index].l1_pq_index == -1));
                                RQ.entry[index].write_translation_merged = 1;
                                RQ.entry[index].l1_wq_index_depend_on_me.insert(MSHR.entry[mshr_index].l1_wq_index);
                            }

                            if(MSHR.entry[mshr_index].l1_pq_index != -1)
                            {
                                assert((MSHR.entry[mshr_index].l1_wq_index == -1) && (MSHR.entry[mshr_index].l1_rq_index == -1));
                                RQ.entry[index].prefetch_translation_merged = 1;
                                RQ.entry[index].l1_pq_index_depend_on_me.insert(MSHR.entry[mshr_index].l1_pq_index);

			
				DP ( if (warmup_complete[read_cpu][((1<<LOG2_THREADS)-1) & RQ.entry[index].instr_id]){ 
                        	cout << "[" << NAME << "] " << __func__ << " this should be printed.  instr_id: " << (RQ.entry[index].instr_id >> LOG2_THREADS)<< " prior_id: ";
			        cout << (MSHR.entry[mshr_index].instr_id >> LOG2_THREADS) << " address: " << hex << RQ.entry[index].address << " full_addr: " << RQ.entry[index].full_addr << dec;
                        	if(RQ.entry[index].read_translation_merged)
                               		cout << " read_translation_merged ";
                            	if(RQ.entry[index].write_translation_merged)
                                	cout << " write_translation_merged ";
                            	if(RQ.entry[index].prefetch_translation_merged)
                            		cout << " prefetch_translation_merged ";

                        	cout << " cycle: " << RQ.entry[index].event_cycle << endl; });
                            }
                            
			    }
                            
			    if(RQ.entry[index].fill_level > MSHR.entry[mshr_index].fill_level)
				    RQ.entry[index].fill_level = MSHR.entry[mshr_index].fill_level;


			    if(cache_type == IS_ITLB || cache_type == IS_DTLB || cache_type == IS_STLB)
			    {
				    if ((MSHR.entry[mshr_index].type == PREFETCH_TRANSLATION || MSHR.entry[mshr_index].type == TRANSLATION_FROM_L1D) && RQ.entry[index].type == LOAD_TRANSLATION)
				    {
					    #ifdef DEBUG
					    cout << "LATE PREFETCH: T"<< (((1<<LOG2_THREADS)-1)&MSHR.entry[mshr_index].instr_id) ;
					    cout <<hex << " ADDR: "<< MSHR.entry[mshr_index].address << " REGION: " << (MSHR.entry[mshr_index].address >> 13) ;
					    cout << " initiated by: "  << MSHR.entry[mshr_index].pf_metadata << endl;
					    #endif
					    ++pf_late;
				    }
			    }
			    else
			    	++pf_late;//@v Late prefetch-> on-demand requests hit in MSHR
			    
			    MSHR.entry[mshr_index] = RQ.entry[index];

				if(prior_fill_l1i && MSHR.entry[mshr_index].fill_l1i == 0)
					MSHR.entry[mshr_index].fill_l1i = 1;		
				if(prior_fill_l1d && MSHR.entry[mshr_index].fill_l1d == 0)
					MSHR.entry[mshr_index].fill_l1d = 1;
	
				DP ( if (warmup_complete[read_cpu]){ 
                        cout << "[" << NAME << "] " << __func__ << " this should be printed";
                        cout << " instr_id: " << (RQ.entry[index].instr_id >> LOG2_THREADS) << " prior_id: " << MSHR.entry[mshr_index].instr_id;
                        cout << " address: " << hex << RQ.entry[index].address;
                        cout << " full_addr: " << RQ.entry[index].full_addr << dec;
                        if(MSHR.entry[mshr_index].read_translation_merged)
                               cout << " read_translation_merged ";
                           if(MSHR.entry[mshr_index].write_translation_merged)
                                cout << " write_translation_merged ";
                           if(MSHR.entry[mshr_index].prefetch_translation_merged)
                                cout << " prefetch_translation_merged ";

                        cout << " cycle: " << RQ.entry[index].event_cycle << endl; });



				//Neelu: Commenting this as L1I prefetching is now enabled.
			    //assert(cache_type != IS_L1I);//@Vishal: L1I cache does not generate prefetch packet.
			    

			    //@Vishal: L1 RQ has virtual address, but miss needs to track physical address, so prior addresses are kept
			    if(cache_type == IS_L1D || cache_type == IS_L1I)
			    {
				MSHR.entry[mshr_index].address = prior_address;
                            	MSHR.entry[mshr_index].full_addr = prior_full_addr;
                            	MSHR.entry[mshr_index].full_physical_address = prior_full_physical_address;
			    }
                            
                            // in case request is already returned, we should keep event_cycle and retunred variables
                            MSHR.entry[mshr_index].returned = prior_returned;
                            MSHR.entry[mshr_index].event_cycle = prior_event_cycle;
                            MSHR.entry[mshr_index].data = prior_data;

			    //Neelu: set the late bit
			    if(cache_type == IS_L1D)
			    {
				////cout<<"Neelu: MSHR entry late_pref INC"<<endl;
	                        MSHR.entry[mshr_index].late_pref = 1;
				late_prefetch++;
			    } 
                        }


			   /*if(cache_type == IS_STLB)
			   {
			 	// Fill level of incoming request and prefetch packet should be same else STLB prefetch request(with instruction=1) might get          merged with DTLB/ITLB, making send_both_tlb=1 due to a msimatch in instruction variable. If this happens, data will be returned to           both ITLB and DTLB, incurring MSHR miss
				if(MSHR.entry[mshr_index].fill_level == 1 && RQ.entry[index].fill_level == 1)
				{
					 if((MSHR.entry[mshr_index].instruction != RQ.entry[index].instruction) && MSHR.entry[mshr_index].send_both_tlb == 0)
					 {
						MSHR.entry[mshr_index].send_both_tlb = 1;
					 }
				 }
			    }*/


						//@Vishal: Check if any translation is dependent on this read request
                    if(!merging_already_done && (cache_type == IS_ITLB || cache_type ==  IS_DTLB || cache_type == IS_STLB))
                    {
                            if(RQ.entry[index].read_translation_merged)
                            {
                                MSHR.entry[mshr_index].read_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_rq_index_depend_on_me.join(RQ.entry[index].l1_rq_index_depend_on_me, RQ_SIZE);
                            }

                            if(RQ.entry[index].write_translation_merged)
                            {
                                MSHR.entry[mshr_index].write_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_wq_index_depend_on_me.join(RQ.entry[index].l1_wq_index_depend_on_me, WQ_SIZE);
                            }

                            if(RQ.entry[index].prefetch_translation_merged)
                            {
                                MSHR.entry[mshr_index].prefetch_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_pq_index_depend_on_me.join(RQ.entry[index].l1_pq_index_depend_on_me, PQ_SIZE);
                            }

                            if(RQ.entry[index].l1_rq_index != -1)
                            {
                                assert((RQ.entry[index].l1_wq_index == -1) && (RQ.entry[index].l1_pq_index == -1));
                                MSHR.entry[mshr_index].read_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_rq_index_depend_on_me.insert(RQ.entry[index].l1_rq_index);
                            }

                            if(RQ.entry[index].l1_wq_index != -1)
                            {
                                assert((RQ.entry[index].l1_rq_index == -1) && (RQ.entry[index].l1_pq_index == -1));
                                MSHR.entry[mshr_index].write_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_wq_index_depend_on_me.insert(RQ.entry[index].l1_wq_index);
                            }

                            if(RQ.entry[index].l1_pq_index != -1)
                            {
                                assert((RQ.entry[index].l1_wq_index == -1) && (RQ.entry[index].l1_rq_index == -1));
                                MSHR.entry[mshr_index].prefetch_translation_merged = 1;
                                MSHR.entry[mshr_index].l1_pq_index_depend_on_me.insert(RQ.entry[index].l1_pq_index);
                            }
                    }

		#if defined(PERFECT_DTLB) || defined(PERFECT_DTLB_EXCLUDING_COLD_MISSES) || defined(PERFECT_STLB) || defined(PERFECT_STLB_EXCLUDING_COLD_MISSES)
		#if defined(DETECT_CRITICAL_IPS) || defined(DETECT_CRITICAL_TRANSLATIONS)
		if (cache_type == IS_DTLB)
		{
			if (RQ.entry[index].sent == 0)
				MSHR.entry[mshr_index].sent = 0;
		}
		else if (cache_type == IS_STLB)
		{
			if (RQ.entry[index].sent == 0)
				MSHR.entry[mshr_index].sent = 0;
		}
		#endif
		#endif

                        MSHR_MERGED[RQ.entry[index].type]++;

                       DP ( if (warmup_complete[read_cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id]){	// || RQ.entry[index].address) {
                        cout << "[" << NAME << "] " << __func__ << " mshr merged";
                        cout << " instr_id: " << (RQ.entry[index].instr_id >> LOG2_THREADS) << " prior_id: " << MSHR.entry[mshr_index].instr_id; 
                        cout << " address: " << hex << RQ.entry[index].address;
                        cout << " full_addr: " << RQ.entry[index].full_addr << dec << " translation sent (mshr)="<< MSHR.entry[mshr_index].sent ;
			if(MSHR.entry[mshr_index].read_translation_merged)
			        cout << " read_translation_merged ";
			if(MSHR.entry[mshr_index].write_translation_merged)
				cout << " write_translation_merged ";
			if(MSHR.entry[mshr_index].prefetch_translation_merged)
				cout << " prefetch_translation_merged ";

                        cout << " cycle: " << RQ.entry[index].event_cycle << endl; });
                    }
                    else { // WE SHOULD NOT REACH HERE
                        cerr << "[" << NAME << "] MSHR errors" << endl;
                        assert(0);
                    }
                }

                if (miss_handled) {
                    // update prefetcher on load instruction
		    if (RQ.entry[index].type == LOAD) {
			assert(cache_type != IS_ITLB || cache_type != IS_DTLB || cache_type != IS_STLB);
			 
			if(cache_type == IS_L1I)
			    l1i_prefetcher_cache_operate(read_cpu, RQ.entry[index].ip, 0, 0);
                        if (cache_type == IS_L1D) 
                            l1d_prefetcher_operate(RQ.entry[index].full_addr, RQ.entry[index].ip, 0, RQ.entry[index].type, RQ.entry[index].critical_ip_flag);	//RQ.entry[index].instr_id);
                       else if ((cache_type == IS_L2C) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].instruction == 0) && (RQ.entry[index].type != LOAD_TRANSLATION) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].type != TRANSLATION_FROM_L1D))
			  l2c_prefetcher_operate(RQ.entry[index].address<<LOG2_BLOCK_SIZE, RQ.entry[index].ip, 0, RQ.entry[index].type, 0);	// RQ.entry[index].instr_id);
                       else if (cache_type == IS_LLC)
			  {
			    cpu = read_cpu;
			    llc_prefetcher_operate(RQ.entry[index].address<<LOG2_BLOCK_SIZE, RQ.entry[index].ip, 0, RQ.entry[index].type, 0);
			    cpu = 0;
			  }
		    }
		    else if (RQ.entry[index].type == LOAD_TRANSLATION) {
			   assert (cache_type != IS_L1D || cache_type != IS_L1I || cache_type != IS_L2C || cache_type != IS_LLC); 
		      if (cache_type == IS_ITLB)
		     {
		      	itlb_prefetcher_operate(RQ.entry[index].address<<LOG2_PAGE_SIZE, RQ.entry[index].ip, 0, RQ.entry[index].type, RQ.entry[index].instr_id, RQ.entry[index].instruction);
		      	
		      }
		      else if (cache_type == IS_DTLB)
		      {
		      #ifdef SANITY_CHECK
    			if(RQ.entry[index].instruction)
    			{
    				//cout << "DTLB prefetch packet should not prefetch address translation of instruction " << endl;
    				assert(0);
    			}
			#endif
		      	dtlb_prefetcher_operate(RQ.entry[index].address, RQ.entry[index].ip, 0, RQ.entry[index].type, RQ.entry[index].instr_id, RQ.entry[index].instruction, RQ.entry[index].critical_ip_flag);
		      	
		      }
		      else if (cache_type == IS_STLB)
		      {
			      //Neelu: Sending l1d prefetcher accuracy and l2c mpki to stlb prefetcher. 
			      /*uint32_t l2c_mpki; // = (ooo_cpu[fill_cpu].L2C.sim_access[fill_cpu][0]*1000)/(ooo_cpu[fill_cpu].num_retired);
	                      if(warmup_complete[read_cpu])
		                      if(ooo_cpu[read_cpu].num_retired - ooo_cpu[read_cpu].warmup_instructions > 0)
			                      l2c_mpki = (ooo_cpu[read_cpu].L2C.sim_miss[read_cpu][0]*1000)/(ooo_cpu[read_cpu].num_retired - ooo_cpu[read_cpu].warmup_instructions);
	                      else
		                      if(ooo_cpu[read_cpu].num_retired > 0)
			                      l2c_mpki = (ooo_cpu[read_cpu].L2C.sim_miss[read_cpu][0]*1000)/(ooo_cpu[read_cpu].num_retired);
				*/
	   		int temp_type = LOAD_TRANSLATION;
                        if(RQ.entry[index].prefetch_translation_merged == true || RQ.entry[index].l1_pq_index != -1)
	                        temp_type = TRANSLATION_FROM_L1D;			
		      	stlb_prefetcher_operate(RQ.entry[index].address, RQ.entry[index].ip, 0, temp_type, RQ.entry[index].instr_id, RQ.entry[index].instruction);
		      	
		      }
                    }

                    MISS[RQ.entry[index].type]++;
                    ACCESS[RQ.entry[index].type]++;

                    // remove this entry from RQ
                    RQ.remove_queue(&RQ.entry[index]);
		    reads_available_this_cycle--;

#ifdef IDEAL_CACHE_FOR_TRANSLATION_ACCESS
 	            unordered_set<uint64_t>::iterator it =  stall_rq_index.find(index);
          	    if(it != stall_rq_index.end())
		    {
            		stall_rq_index.erase(it);
          	    }	
#endif

                }
            }
        }
	else
	  {
	    return;
	  }

	if(reads_available_this_cycle == 0)
	  {
	    return;
	  }
    }
}

void CACHE::handle_prefetch()
{
    // handle prefetch

    for (uint32_t i=0; i<MAX_READ; i++) {
      
      uint32_t prefetch_cpu = PQ.entry[PQ.head].cpu;
      if (prefetch_cpu == NUM_CPUS)
        return;
	
        // handle the oldest entry
        if ((PQ.entry[PQ.head].event_cycle <= current_core_cycle[prefetch_cpu]) && (PQ.occupancy > 0))
        {

        	if(cache_type == IS_L1D && (PQ.entry[PQ.head].translated != COMPLETED)) //@Vishal: Check if the translation is done for that prefetch request or not.
		{	
			return;
		}

		//Neelu: Translation complete check for L1I prefetch requsts
		if((cache_type == IS_L1I) && (PQ.entry[PQ.head].translated != COMPLETED))
		{
			return;
		}
            	int index = PQ.head;
		if((cache_type == IS_L1D || cache_type == IS_L1I) && (PQ.entry[PQ.head].full_physical_address >> LOG2_PAGE_SIZE) == (UINT64_MAX >> LOG2_PAGE_SIZE))
		{
			pf_dropped++;
			//Due to page fault, prefetch request should be dropped
			PQ.remove_queue(&PQ.entry[index]);
			continue;
		}


            // access cache
            uint32_t set = get_set(PQ.entry[index].address);
            int way = check_hit(&PQ.entry[index], set);
            
 	    int way_vb = -1;
	    #ifdef PUSH_VICTIMS_DTLB_VB
	    if (cache_type == IS_DTLB && way < 0)
	    {
		    way_vb = ooo_cpu[prefetch_cpu].DTLB_VB.check_hit( &PQ.entry[index], 0);
		    if (way_vb >= 0)
		    {
			    ooo_cpu[prefetch_cpu].DTLB_VB.block[0][way_vb].used = 1;
			    PQ.entry[index].data_pa = ooo_cpu[prefetch_cpu].DTLB_VB.block[0][way_vb].data;
			    ooo_cpu[prefetch_cpu].DTLB_VB.sim_hit[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                	    ooo_cpu[prefetch_cpu].DTLB_VB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
			    ooo_cpu[prefetch_cpu].DTLB_VB.HIT[PQ.entry[index].type]++;
                	    ooo_cpu[prefetch_cpu].DTLB_VB.ACCESS[PQ.entry[index].type]++;
	   	    	    (&ooo_cpu[prefetch_cpu].DTLB_VB->*update_replacement_state)(prefetch_cpu, 0, way_vb, ooo_cpu[prefetch_cpu].DTLB_VB.block[0][way_vb].full_addr, PQ.entry[index].ip, 0, PQ.entry[index].type, 1);
		    	    PQ.entry[index].data = ooo_cpu[prefetch_cpu].DTLB_VB.block[0][way_vb].data;
		      	    //if (PQ.entry[index].type == PREFETCH_TRANSLATION)
			    //	dtlb_prefetcher_operate(PQ.entry[index].address, PQ.entry[index].ip, 1, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction, PQ.entry[index].critical_ip_flag);
                	    PQ.remove_queue(&PQ.entry[index]);
			    //ooo_cpu[read_cpu].DTLB_VB.pf_useful++;
		    	    // If DTLB victim buffer gets hit, fill DTLB and then proceed
	    	    	    //way = find_victim(read_cpu, RQ.entry[index].instr_id, set, block[set], RQ.entry[index].ip, RQ.entry[index].full_addr, RQ.entry[index].type);
		      	    //RQ.entry[index].type = PREFETCH_TRANSLATION;
			    //fill_cache(set, way, &RQ.entry[index]);
		    }
		    else
		    {
			    //DTLB_VB MISS
			    ooo_cpu[prefetch_cpu].DTLB_VB.MISS[PQ.entry[index].type]++;
                    	    ooo_cpu[prefetch_cpu].DTLB_VB.ACCESS[PQ.entry[index].type]++;
			    ooo_cpu[prefetch_cpu].DTLB_VB.sim_miss[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_VB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
		    }

	    }
	    #endif
	     
	    int way_pb = -1;

	    #ifdef PUSH_DTLB_PB
            //If DTLB misses, check DTLB Prefetch Buffer
            if(cache_type == IS_DTLB && way < 0 && way_vb < 0)
            {
                    way_pb = ooo_cpu[prefetch_cpu].DTLB_PB.check_hit( &PQ.entry[index], 0);
                    if(way_pb >= 0)
                    {
                            ooo_cpu[prefetch_cpu].DTLB_PB.block[0][way_pb].used = 1;
                            PQ.entry[index].data_pa = ooo_cpu[prefetch_cpu].DTLB_PB.block[0][way_pb].data;
                            ooo_cpu[prefetch_cpu].DTLB_PB.sim_hit[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.HIT[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.ACCESS[PQ.entry[index].type]++;
                            (&ooo_cpu[prefetch_cpu].DTLB_PB->*update_replacement_state)(prefetch_cpu, 0, way_pb, ooo_cpu[prefetch_cpu].DTLB_PB.block[0][way_pb].full_addr, PQ.entry[index].ip, 0, PQ.entry[index].type, 1);
                            PQ.entry[index].data = ooo_cpu[prefetch_cpu].DTLB_PB.block[0][way_pb].data;
                            ooo_cpu[prefetch_cpu].DTLB_PB.pf_useful++;
                	    PQ.remove_queue(&PQ.entry[index]);
		    }
                    else
                    {
                            //DTLB_PB MISS
                              ooo_cpu[prefetch_cpu].DTLB_PB.MISS[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.ACCESS[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.sim_miss[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_PB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                    }
            }
	    #endif
	    
	    int way_ub = -1;
	    #ifdef PUSH_DTLB_UB
            //If DTLB VB and PB misses, check DTLB Utility Buffer
            if(cache_type == IS_DTLB && way < 0 && way_vb < 0 && way_pb < 0)
            {
                    way_ub = ooo_cpu[prefetch_cpu].DTLB_UB.check_hit( &PQ.entry[index] , 0);
                    if(way_ub >= 0)
                    {
                            ooo_cpu[prefetch_cpu].DTLB_UB.block[0][way_ub].used = 1;
                            PQ.entry[index].data_pa = ooo_cpu[prefetch_cpu].DTLB_UB.block[0][way_ub].data;
                            ooo_cpu[prefetch_cpu].DTLB_UB.sim_hit[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.HIT[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.ACCESS[PQ.entry[index].type]++;
                            (&ooo_cpu[prefetch_cpu].DTLB_UB->*update_replacement_state)(prefetch_cpu, 0, way_ub, ooo_cpu[prefetch_cpu].DTLB_UB.block[0][way_ub].full_addr, PQ.entry[index].ip, 0, PQ.entry[index].type, 1);
                            PQ.entry[index].data = ooo_cpu[prefetch_cpu].DTLB_UB.block[0][way_ub].data;
                	    PQ.remove_queue(&PQ.entry[index]);
                            
		    }
                    else
                    {
                            //DTLB_UB MISS
                            ooo_cpu[prefetch_cpu].DTLB_UB.MISS[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.ACCESS[PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.sim_miss[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                            ooo_cpu[prefetch_cpu].DTLB_UB.sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                    }
            }
	    #endif


            if (way >= 0) { // prefetch hit
           
                // update replacement policy
                    (this->*update_replacement_state)(prefetch_cpu, set, way, block[set][way].full_addr, PQ.entry[index].ip, 0, PQ.entry[index].type, 1);

                // COLLECT STATS
                sim_hit[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;
                sim_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].type]++;

#ifdef PT_STATS
      		if ((cache_type == IS_L1D || cache_type == IS_L2C || cache_type == IS_LLC) && PQ.entry[index].type == LOAD_TRANSLATION)
		{
        		assert(PQ.entry[index].translation_level > 0 && PQ.entry[index].translation_level < 6);
        		sim_pt_hit[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].translation_level-1]++;
		        sim_pt_access[prefetch_cpu][((1 << LOG2_THREADS) - 1) & PQ.entry[index].instr_id][PQ.entry[index].translation_level-1]++;
      		}
#endif		

		// run prefetcher on prefetches from higher caches
		if(PQ.entry[index].pf_origin_level < fill_level)
		  {
		    if (cache_type == IS_L1D)
		    {
			    //@Vishal: This should never be executed as fill_level of L1 is 1 and minimum pf_origin_level is 1
			    assert(0);
		      l1d_prefetcher_operate(PQ.entry[index].full_addr, PQ.entry[index].ip, 1, PREFETCH, PQ.entry[index].critical_ip_flag);	//, PQ.entry[index].prefetch_id);
		    }
                    else if ((cache_type == IS_L2C) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].instruction == 0) && (RQ.entry[index].type != LOAD_TRANSLATION) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].type != TRANSLATION_FROM_L1D))
		    {
                      PQ.entry[index].pf_metadata = l2c_prefetcher_operate(block[set][way].address<<LOG2_BLOCK_SIZE, PQ.entry[index].ip, 1, PREFETCH, PQ.entry[index].pf_metadata);	//PQ.entry[index].prefetch_id);
			if((((PQ.entry[index].pf_metadata >> 17) & 1) | ((PQ.entry[index].pf_metadata >> 18) & 1)) == 1)
				getting_hint_from_l2++;
		    }
                    else if (cache_type == IS_LLC)
		      	{
				cpu = prefetch_cpu;
				if((((PQ.entry[index].pf_metadata >> 17) & 1) | ((PQ.entry[index].pf_metadata >> 18) & 1)) == 1)
					sending_hint_to_llc++;
					
				PQ.entry[index].pf_metadata = llc_prefetcher_operate(block[set][way].address<<LOG2_BLOCK_SIZE, PQ.entry[index].ip, 1, PREFETCH, PQ.entry[index].pf_metadata);
				cpu = 0;
		      }
		      else if (cache_type == IS_ITLB)
		      {
		      	itlb_prefetcher_operate(PQ.entry[index].address<<LOG2_PAGE_SIZE, PQ.entry[index].ip, 1, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction);
		      	//DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    //cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch on prefetch hit" << "instruction : " << PQ.entry[index].instruction << endl; });
		      }
		      else if (cache_type == IS_DTLB)
		      {
		      #ifdef SANITY_CHECK
    			if(PQ.entry[index].instruction)
    			{
    				//cout << "DTLB prefetch packet should not prefetch address translation of instruction " << endl;
    				assert(0);
    			}
			#endif
		      	dtlb_prefetcher_operate(PQ.entry[index].address, PQ.entry[index].ip, 1, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction, PQ.entry[index].critical_ip_flag);
		      //	DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch on prefetch hit" << " address : "<< PQ.entry[index].address << endl;//});
		      }
		      else if (cache_type == IS_STLB)
		      {
		      	stlb_prefetcher_operate(PQ.entry[index].address, PQ.entry[index].ip, 1, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction);
		      	//DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    //cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch on prefetch hit" << "instruction " << PQ.entry[index].instruction << "  address: "<< PQ.entry[index].address << endl;});
		      }
		  }

                // check fill level
                // data should be updated (for TLBs) in case of hit
                if (PQ.entry[index].fill_level < fill_level) {

		    if(cache_type == IS_STLB)
                        {
				////cout << " DTLB cannot send prefetch translation request to STLB as DTLB cannot " << endl;
				//assert(0);
                                if(PQ.entry[index].send_both_tlb)
                                {
					PQ.entry[index].data = block[set][way].data;
					upper_level_icache[prefetch_cpu]->return_data(&PQ.entry[index]);
					upper_level_dcache[prefetch_cpu]->return_data(&PQ.entry[index]);
                                }
				else if (PQ.entry[index].instruction)
				{
					PQ.entry[index].data = block[set][way].data;
					upper_level_icache[prefetch_cpu]->return_data(&PQ.entry[index]);
				}
				else // data
				{
					PQ.entry[index].data = block[set][way].data;
					upper_level_dcache[prefetch_cpu]->return_data(&PQ.entry[index]);
                        	}
			
				#ifdef SANITY_CHECK
				if(PQ.entry[index].data == 0)
					assert(0);
				#endif
			}
		    else if(fill_level == FILL_L2)
		    {
			    if(PQ.entry[index].fill_l1i)
			    {
				     PQ.entry[index].data = block[set][way].data;
				    upper_level_icache[prefetch_cpu]->return_data(&PQ.entry[index]);
			    }
			    if(PQ.entry[index].fill_l1d)
			    {
				     PQ.entry[index].data = block[set][way].data;
				    upper_level_dcache[prefetch_cpu]->return_data(&PQ.entry[index]);
			    }
		    }
                        else
                        {
			    if (PQ.entry[index].instruction)
			    {
				PQ.entry[index].data = block[set][way].data; 
				upper_level_icache[prefetch_cpu]->return_data(&PQ.entry[index]);
			    }
			    else // data
			    {
				PQ.entry[index].data = block[set][way].data;
				upper_level_dcache[prefetch_cpu]->return_data(&PQ.entry[index]);
			    }
			    
			   #ifdef SANITY_CHECK
			   if(cache_type == IS_ITLB || cache_type == IS_DTLB)
				if(PQ.entry[index].data == 0)
					assert(0);
			   #endif
			}
                }

		#ifdef PUSH_DTLB_PB
		//@Vasudha: In case of STLB prefetch hit, just fill DTLB Prefetch Buffer
		if( cache_type == IS_STLB && PQ.entry[index].fill_level == fill_level && PQ.entry[index].instruction == 0)
		{
			assert(0);	//Since, STLB is not using any prefetcher, it should not reach here
			PQ.entry[index].data = block[set][way].data;
			//Find in which way to fill the translation
			uint32_t victim_way;
			victim_way = (&ooo_cpu[prefetch_cpu].DTLB_PB->*find_victim)( prefetch_cpu, PQ.entry[index].instr_id, 0, ooo_cpu[prefetch_cpu].DTLB_PB.block[0] , PQ.entry[index].ip, PQ.entry[index].full_addr, PQ.entry[index].type);
			(&ooo_cpu[prefetch_cpu].DTLB_PB->*update_replacement_state)(prefetch_cpu, 0, victim_way, PQ.entry[index].full_addr, PQ.entry[index].ip, ooo_cpu[prefetch_cpu].DTLB_PB.block[0][victim_way].full_addr, PQ.entry[index].type, 0);
			ooo_cpu[PQ.entry[index].cpu].DTLB_PB.fill_cache( 0, victim_way, &PQ.entry[index] );
		}
		#endif

                HIT[PQ.entry[index].type]++;
                ACCESS[PQ.entry[index].type]++;
                
                // remove this entry from PQ
                PQ.remove_queue(&PQ.entry[index]);
		reads_available_this_cycle--;
            }
            else
	#ifdef PUSH_VICTIMS_DTLB_VB
		if (cache_type == IS_DTLB && way_vb >= 0)
		{
			reads_available_this_cycle--;
		}
		else
	#endif
	#ifdef PUSH_DTLB_PB
		if (cache_type == IS_DTLB && way_pb >= 0)
		{
			reads_available_this_cycle--;
		}
    		else	    
	#endif
	#ifdef PUSH_DTLB_UB
		if (cache_type == IS_DTLB && way_ub >= 0)
		{
			reads_available_this_cycle--;
		}
	        else
	#endif
	    { // prefetch miss

                //DP ( if (warmup_complete[prefetch_cpu] ) {
                //cout << "[" << NAME << "] " << __func__ << " prefetch miss" << "  IP: " << PQ.entry[index].ip;
                //cout << " instr_id: " <<dec <<  PQ.entry[index].prefetch_id << " address: " << hex << PQ.entry[index].address << endl;
                //cout << " full_addr: " << PQ.entry[index].full_addr << dec << " fill_level: " << PQ.entry[index].fill_level;
                //cout << " cycle: " << PQ.entry[index].event_cycle << endl; });

                // check mshr
                uint8_t miss_handled = 1;
                int mshr_index = check_nonfifo_queue(&MSHR, &PQ.entry[index],false); //@Vishal: Updated from check_mshr

                if ((mshr_index == -1) && (MSHR.occupancy < MSHR_SIZE)) { // this is a new miss
	
			//Neelu: Calculting number of prefetches issued from L1D to L2C i.e. the next level
                        if(cache_type == IS_L1D)
				prefetch_count++;

					++pf_lower_level;	//@v Increment for new prefetch miss
                   //DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    //cout << "[" << NAME << "_PQ] " <<  __func__ << " want to add prefetch_id: " << PQ.entry[index].prefetch_id << " address: " << hex << PQ.entry[index].address;
                    //cout << " full_addr: " << PQ.entry[index].full_addr << dec;
                    //if(lower_level)
                    //cout << " occupancy: " << lower_level->get_occupancy(3, PQ.entry[index].address) << " SIZE: " << lower_level->get_size(3, PQ.entry[index].address) << endl; });

                    // first check if the lower level PQ is full or not
                    // this is possible since multiple prefetchers can exist at each level of caches
                    if (lower_level) {
		      if (cache_type == IS_LLC) {
			if (lower_level->get_occupancy(1, PQ.entry[index].address) == lower_level->get_size(1, PQ.entry[index].address))
			  miss_handled = 0;
			else {
			  
			  // run prefetcher on prefetches from higher caches
			  if(PQ.entry[index].pf_origin_level < fill_level)
			    {
			      if (cache_type == IS_LLC)
				{
				  cpu = prefetch_cpu;
				  if((((PQ.entry[index].pf_metadata >> 17) & 1) | ((PQ.entry[index].pf_metadata >> 18) & 1)) == 1)
	                                  sending_hint_to_llc++;
				  PQ.entry[index].pf_metadata = llc_prefetcher_operate(PQ.entry[index].address<<LOG2_BLOCK_SIZE, PQ.entry[index].ip, 0, PREFETCH, PQ.entry[index].pf_metadata);
				  cpu = 0;
				}
			    }
			  
			  // add it to MSHRs if this prefetch miss will be filled to this cache level
			  if (PQ.entry[index].fill_level <= fill_level)
			    add_nonfifo_queue(&MSHR, &PQ.entry[index]); //@Vishal: Updated from add_mshr

			  lower_level->add_rq(&PQ.entry[index]); // add it to the DRAM RQ
			}
		      }
		      else {
			if (lower_level->get_occupancy(3, PQ.entry[index].address) == lower_level->get_size(3, PQ.entry[index].address))
			  miss_handled = 0;
			else {

			  // run prefetcher on prefetches from higher caches
			  if(PQ.entry[index].pf_origin_level < fill_level)
			    {
			     // if (cache_type == IS_L1D)
			       //@Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)			       
                              if ((cache_type == IS_L1D) && (RQ.entry[index].instruction == 0) &&
			          (RQ.entry[index].type != LOAD_TRANSLATION) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].type != TRANSLATION_FROM_L1D))
#else
                              if (cache_type == IS_L1D)
#endif
				l1d_prefetcher_operate(PQ.entry[index].full_addr, PQ.entry[index].ip, 0, PREFETCH, PQ.entry[index].critical_ip_flag);	// PQ.entry[index].prefetch_id);
			      else if ((cache_type == IS_L2C) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].instruction == 0) && (RQ.entry[index].type != LOAD_TRANSLATION) && (RQ.entry[index].type != PREFETCH_TRANSLATION) && (RQ.entry[index].type != TRANSLATION_FROM_L1D))
			      {
				PQ.entry[index].pf_metadata = l2c_prefetcher_operate(PQ.entry[index].address<<LOG2_BLOCK_SIZE, PQ.entry[index].ip, 0, PREFETCH, PQ.entry[index].pf_metadata);	// PQ.entry[index].prefetch_id);
				if((((PQ.entry[index].pf_metadata >> 17) & 1) | ((PQ.entry[index].pf_metadata >> 18) & 1)) == 1)
	                                getting_hint_from_l2++;
			      }
				else if (cache_type == IS_ITLB)
		      		{
		      			itlb_prefetcher_operate(PQ.entry[index].address<<LOG2_PAGE_SIZE, PQ.entry[index].ip, 0, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction);
		      			//DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    			//cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch" << "instruction: " << PQ.entry[index].instruction << endl; });
		      		}
		      		else if (cache_type == IS_DTLB)
		      		{
		      			#ifdef SANITY_CHECK
    					if(PQ.entry[index].instruction)
    					{
    					//cout << "DTLB prefetch packet should not prefetch address translation of instruction" << endl;
    					assert(0);
    					}
					#endif
		      			dtlb_prefetcher_operate(PQ.entry[index].address, PQ.entry[index].ip, 0, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction, PQ.entry[index].critical_ip_flag);
		      			//DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    			//cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch" << "instruction: " << PQ.entry[index].instruction << endl; });
		      		}
		      		else if (cache_type == IS_STLB)
		      		{
		      			stlb_prefetcher_operate(PQ.entry[index].address, PQ.entry[index].ip, 0, PQ.entry[index].type, PQ.entry[index].prefetch_id, PQ.entry[index].instruction);
		      			//DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
                    			//cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << "from handle prefetch" <<  " instruction : "<< PQ.entry[index].instruction << endl; });
		      		}
			    }
			  
			    if(cache_type == IS_L1D || cache_type == IS_L1I)
			    {
			    	 assert(PQ.entry[index].full_physical_address != 0);
				  	 PACKET new_packet = PQ.entry[index];
				  	 //@Vishal: Send physical address to lower level and track physical address in MSHR  
				  	 new_packet.address = PQ.entry[index].full_physical_address >> LOG2_BLOCK_SIZE;
				  	 new_packet.full_addr = PQ.entry[index].full_physical_address; 

				  	 if (PQ.entry[index].fill_level <= fill_level)
				  	 	add_nonfifo_queue(&MSHR, &new_packet); //@Vishal: Updated from add_mshr
					 lower_level->add_pq(&new_packet);
			    }
			    else
			    {

			  	// add it to MSHRs if this prefetch miss will be filled to this cache level
				  if (PQ.entry[index].fill_level <= fill_level)
				    add_nonfifo_queue(&MSHR, &PQ.entry[index]); //@Vishal: Updated from add_mshr

				  lower_level->add_pq(&PQ.entry[index]); // add it to the DRAM RQ
				
				}
			}
		      }
		   }
		   else {
#ifdef INS_PAGE_TABLE_WALKER
			   assert(0);
#else
		
			   if(PQ.entry[index].fill_level <= fill_level)
			   	add_nonfifo_queue(&MSHR, &PQ.entry[index]);
			   if(cache_type == IS_STLB) {
				//emulate page table walk
				uint64_t pa = va_to_pa(PQ.entry[index].cpu, PQ.entry[index].instr_id, PQ.entry[index].full_addr, PQ.entry[index].address);
				PQ.entry[index].data = pa >> LOG2_PAGE_SIZE;
				PQ.entry[index].event_cycle = current_core_cycle[cpu];
				if(PQ.entry[index].l1_pq_index != -1)
				{
					assert(PQ.entry[index].l1_pq_index == -1 && PQ.entry[index].l1_wq_index == -1);
					PQ.entry[index].data_pa = pa >> LOG2_PAGE_SIZE;
					if(PROCESSED.occupancy < PROCESSED.SIZE)
						PROCESSED.add_queue(&PQ.entry[index]);
					else
						assert(0);
				}
				return_data(&PQ.entry[index]);
			   }
#endif
		   }
                }
                else {
                    if ((mshr_index == -1) && (MSHR.occupancy == MSHR_SIZE)) { // not enough MSHR resource

                        // TODO: should we allow prefetching with lower fill level at this case?
                        
                        // cannot handle miss request until one of MSHRs is available
                        miss_handled = 0;
                        STALL[PQ.entry[index].type]++;
                    }
                    else if (mshr_index != -1) { // already in-flight miss

			    //When demand request got merged with prefetch request -> late_prefetch
			    if (MSHR.entry[mshr_index].type == LOAD_TRANSLATION && PQ.entry[index].type == PREFETCH_TRANSLATION)
			    {
				    ++pf_late;
				    #ifdef  DEBUG
				    if (cache_type == IS_DTLB)
				    {
					    cout << "LATE PREFETCH2: T"<< (((1<<LOG2_THREADS)-1)&MSHR.entry[mshr_index].instr_id) ;
					    cout <<hex << " ADDR: "<< MSHR.entry[mshr_index].address << " REGION: " << (MSHR.entry[mshr_index].address >> 13) ;
					    cout << " initiated by: "  << PQ.entry[index].pf_metadata << endl;
				    }
				    #endif
			    }	    

                        // no need to update request except fill_level
                        // update fill_level
                        if (PQ.entry[index].fill_level < MSHR.entry[mshr_index].fill_level)
			{
				//@Vasudha:STLB Prefetch packet can have instruction variable as 1 or 0. Update instruction variable when upper level TLB sends request. 
				MSHR.entry[mshr_index].instruction = PQ.entry[index].instruction;
			    	MSHR.entry[mshr_index].fill_level = PQ.entry[index].fill_level;
			}
                        
			//request coming from both DTLB and ITLB should be returned to both
			if(cache_type == IS_STLB)
			{
				if(PQ.entry[index].fill_level == 1 && MSHR.entry[mshr_index].fill_level == 1)
					if(PQ.entry[index].instruction != MSHR.entry[mshr_index].instruction)
						MSHR.entry[mshr_index].send_both_tlb = 1;
			}	
                            

			 if((PQ.entry[index].fill_l1i) && (MSHR.entry[mshr_index].fill_l1i != 1))
			 {
				 MSHR.entry[mshr_index].fill_l1i = 1;
			 }
 			 if((PQ.entry[index].fill_l1d) && (MSHR.entry[mshr_index].fill_l1d != 1))
			 {
				 MSHR.entry[mshr_index].fill_l1d = 1;
			 }
                        
                        MSHR_MERGED[PQ.entry[index].type]++;

                        DP ( if (warmup_complete[prefetch_cpu][((1<<LOG2_THREADS)-1)&MSHR.entry[mshr_index].instr_id] ) {
                        cout << "[" << NAME << "] " << __func__ << " mshr merged";
                        cout << " instr_id: " << (PQ.entry[index].instr_id >> LOG2_THREADS) << " prior_id: " << MSHR.entry[mshr_index].instr_id; 
                        cout << " address: " << hex << PQ.entry[index].address;
                        cout << " full_addr: " << PQ.entry[index].full_addr << dec << " fill_level: " << MSHR.entry[mshr_index].fill_level;
                        cout << " cycle: " << MSHR.entry[mshr_index].event_cycle << endl; });
                    }
                    else { // WE SHOULD NOT REACH HERE
                        cerr << "[" << NAME << "] MSHR errors" << endl;
                        assert(0);
                    }
                }

                if (miss_handled) {

                  //DP ( if (warmup_complete[prefetch_cpu] ) {
                    //cout << "[" << NAME << "] " << __func__ << " prefetch miss handled";
                    //cout << " instr_id: " << PQ.entry[index].instr_id << " address: " << hex << PQ.entry[index].address;
                    //cout << " full_addr: " << PQ.entry[index].full_addr << dec << " fill_level: " << PQ.entry[index].fill_level;
                    //cout << " cycle: " << PQ.entry[index].event_cycle << endl; });

                    MISS[PQ.entry[index].type]++;
                    ACCESS[PQ.entry[index].type]++;

                    // remove this entry from PQ
                    PQ.remove_queue(&PQ.entry[index]);
		    reads_available_this_cycle--;
                }
            }
        }
	else
	  {
	    return;
	  }

	if(reads_available_this_cycle == 0)
	  {
	    return;
	  }
    }
}

void CACHE::operate()
{
    handle_fill();
    handle_writeback();
    reads_available_this_cycle = MAX_READ;

    //@Vishal: VIPT
    if(cache_type == IS_L1I || cache_type == IS_L1D)
    	handle_processed();
    handle_read();

    if (PQ.occupancy && (reads_available_this_cycle > 0))
        handle_prefetch();

	if(PQ.occupancy && ((current_core_cycle[cpu] - PQ.entry[PQ.head].cycle_enqueued) > DEADLOCK_CYCLE))
	{
		//cout << "DEADLOCK, PQ entry is not serviced for " << DEADLOCK_CYCLE << " cycles cache_type: " << NAME << " prefetch_id: "<<PQ.entry[PQ.head].prefetch_id<<  endl;
		//cout << PQ.entry[PQ.head];	
		assert(0);
	}
}

uint32_t CACHE::get_set(uint64_t address)
{
	#ifdef PUSH_DTLB_PB
	if(cache_type == IS_DTLB_PB)
		return 0;
    
	else
	#endif
	
	#ifdef PUSH_DTLB_UB
	if(cache_type == IS_DTLB_UB)
		return 0;
    
	else
	#endif
	
	#ifdef PUSH_VICTIMS_DTLB_VB	
	if (cache_type == IS_DTLB_VB)
		return 0;
	else
	#endif
	
	if (knob_cloudsuite && (cache_type == IS_DTLB || cache_type == IS_ITLB || cache_type == IS_STLB))
                return (uint32_t) ((address >> 9) & ((1 << lg2(NUM_SET)) - 1));
	else
		return (uint32_t) (address & ((1 << lg2(NUM_SET)) - 1)); 
}

uint32_t CACHE::get_way(uint64_t address, uint32_t set, uint64_t instr_id)
{
	uint8_t thread_num = ((1 << LOG2_THREADS) - 1) & instr_id;
        for (uint32_t way=0; way<NUM_WAY; way++) {
        	if (block[set][way].valid && (block[set][way].tag == address) && ((((1 << LOG2_THREADS) - 1) & block[set][way].instr_id) == thread_num)) 
        	    return way;
    }

    return NUM_WAY;
}

void CACHE::fill_cache(uint32_t set, uint32_t way, PACKET *packet)
{
#ifdef SANITY_CHECK

    #ifdef PUSH_DTLB_PB
    if(cache_type == IS_DTLB_PB) {
	    if(packet->data == 0)
	    {
	    	//cout << "Inside DTLB_PB, current = " << current_core_cycle[cpu] << " instr_id = " << packet->instr_id << endl;
		assert(0);
	    }
    }
    #endif
    #ifdef PUSH_DTLB_UB
    if(cache_type == IS_DTLB_UB) {
	    if(packet->data == 0)
	    {
		assert(0);
	    }
    }
    #endif
   #ifdef PUSH_VICTIMS_DTLB_VB
   if(cache_type == IS_DTLB_VB)
   {
	   if(packet->data == 0)
		   assert(0);
   }
   #endif 
    if (cache_type == IS_ITLB) {
        if (packet->data == 0)
        {
            //cout << "current = " << current_core_cycle[cpu] << " instr_id = "<< packet->instr_id << endl;
            assert(0);
        }
    }

    if (cache_type == IS_DTLB) {
        if (packet->data == 0)
        {
           //cout << "current = " << current_core_cycle[cpu] << " instr_id = "<< packet->instr_id << endl;
           assert(0);
        }
    }

    if (cache_type == IS_STLB) {
        if (packet->data == 0)
            assert(0);
    }

    if (cache_type == IS_PSCL5) {
        if (packet->data == 0)
            assert(0);
    }

    if (cache_type == IS_PSCL4) {
        if (packet->data == 0)
            assert(0);
    }

    if (cache_type == IS_PSCL3) {
        if (packet->data == 0)
            assert(0);
    }

    if (cache_type == IS_PSCL2) {
        if (packet->data == 0)
            assert(0);
    }
#endif
    if (block[set][way].prefetch && (block[set][way].used == 0))
    {
	    pf_useless++;
	    if (cache_type == IS_DTLB)
	    {
	        #ifdef DEBUG
	        cout << "USELESS PREFETCH: T" <<(((1<<LOG2_THREADS)-1)&block[set][way].instr_id) << " ADDR: "<<hex<< block[set][way].address << " Region: " << (block[set][way].address >> 13) 
			<< " initiated by, region: " << block[set][way].pf_metadata << endl;
	        #endif
		uint16_t asid = block[set][way].address & 0x1FF;	//stores ASID
		//@Vasudha: removing 9 bits + region-offset bits from address
		uint64_t region_ID = ((block[set][way].address >> 13) << LOG2_THREADS) | (((1 << LOG2_THREADS) - 1) & block[set][way].instr_id) ;
		region_ID = (region_ID << 9) | asid;

		if (block[set][way].pf_metadata != 0)
		dtlb_prefetcher_send_feedback(block[set][way].pf_metadata, region_ID);	
	    }

	    if (cache_type == IS_STLB)
	    {
		    stlb_prefetcher_send_feedback(block[set][way].pf_metadata, block[set][way].address, block[set][way].instr_id);
		    #ifdef DEBUG_PREF
		    if (all_warmup_complete > ooo_cpu[cpu].thread)
		 	cout << "USELESS PERFETCH: Cycle:" << dec << current_core_cycle[cpu] << " address:" << hex << block[set][way].address<< dec << endl;
	    	    #endif
	    }
    }

    if (block[set][way].valid == 0)
        block[set][way].valid = 1;
    block[set][way].dirty = 0;
    block[set][way].prefetch = (packet->type == PREFETCH || packet->type == PREFETCH_TRANSLATION || packet->type == TRANSLATION_FROM_L1D) ? 1 : 0;
    block[set][way].used = 0;

    block[set][way].pf_metadata = packet->pf_metadata;
    //Neelu: setting IPCP prefetch class
    block[set][way].pref_class = ((packet->pf_metadata & PREF_CLASS_MASK) >> NUM_OF_STRIDE_BITS);

    if (block[set][way].prefetch) 
    {
        pf_fill++;

	//Neelu: IPCP prefetch stats
        if(cache_type == IS_L1D)
     	{
	     if(block[set][way].pref_class < 5)						                     
	     {
		     pref_filled[cpu][block[set][way].pref_class]++;
	     }
     	}
    }

    //@Rahul: replacement policy
    block[set][way].type = packet->type;
    block[set][way].trip_count = 0;

    block[set][way].delta = packet->delta;
    block[set][way].depth = packet->depth;
    block[set][way].signature = packet->signature;
    block[set][way].confidence = packet->confidence;

    block[set][way].critical = packet->critical_ip_flag;
    block[set][way].tag = packet->address; //@Vishal: packet->address will be physical address for L1I, as it is only filled on a miss.
    block[set][way].address = packet->address;
    block[set][way].full_addr = packet->full_addr;
    block[set][way].data = packet->data;
    block[set][way].cpu = packet->cpu;
    if (packet->type == PREFETCH_TRANSLATION && (cache_type == IS_DTLB || cache_type == IS_ITLB || cache_type == IS_STLB))
	    block[set][way].instr_id = packet->prefetch_id;
    else 
    	    block[set][way].instr_id = packet->instr_id;

    if (cache_type == IS_STLB)
	    block[set][way].ip = packet->ip;
  //DP ( if (warmup_complete[packet->cpu] ) {
    //cout << "[" << NAME << "] " << __func__ << " set: " << set << " way: " << way;
    //cout << " lru: " << block[set][way].lru << " tag: " << hex << block[set][way].tag << " full_addr: " << block[set][way].full_addr;
    //cout << " data: " << block[set][way].data << dec << endl; });
}

int CACHE::check_hit(PACKET *packet, uint32_t set)
{
    //@Vasudha: Commented to implement extended map for DTLB
    //uint32_t set = get_set(packet->address);
    int match_way = -1;

    if (NUM_SET < set) {
        cerr << "[" << NAME << "_ERROR] " << __func__ << " invalid set index: " << set << " NUM_SET: " << NUM_SET;
        cerr << " address: " << hex << packet->address << " full_addr: " << packet->full_addr << dec;
        cerr << " event: " << packet->event_cycle << endl;
        assert(0);
    }

    uint64_t packet_tag;
    if(cache_type == IS_L1I || cache_type == IS_L1D) //@Vishal: VIPT
    {
	    assert(packet->full_physical_address != 0);
	    packet_tag = packet->full_physical_address >> LOG2_BLOCK_SIZE;
    }
    else
	    packet_tag = packet->address;

    // hit
    for (uint32_t way=0; way<NUM_WAY; way++) {

	//@Vasudha:SMT: check whether both packet and cache block are from the same thread
        if (block[set][way].valid && (block[set][way].tag == packet_tag)) {
		if (((( 1 << LOG2_THREADS ) - 1) & packet->instr_id) == ((( 1 << LOG2_THREADS ) - 1) & block[set][way].instr_id) || knob_cloudsuite)
		{
			match_way = way;
			break;
		}

           DP ( if (warmup_complete[packet->cpu][((1<<LOG2_THREADS)-1)&packet->instr_id] ) {
            cout << "[" << NAME << "] " << __func__ << " instr_id: " << (packet->instr_id >> LOG2_THREADS)<< " type: " << +packet->type << hex << " addr: " << packet->address;
            cout << " full_addr: " << packet->full_addr << " tag: " << block[set][way].tag << " data: " << block[set][way].data << dec;
            cout << " set: " << set << " way: " << way << " lru: " << block[set][way].lru;
            cout << " event: " << packet->event_cycle << " cycle: " << current_core_cycle[cpu] << endl; });

           
        }
    }

#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
                    //cout << "[" << NAME << "] " << __func__ << " instr_id: " << packet->instr_id << " type: " << +packet->type << hex << " addr: " << packet->address;
            //cout << " full_addr: " << packet->full_addr<<dec;
            //cout << " set: " << set << " way: " << match_way;
            //cout << " event: " << packet->event_cycle << " cycle: " << current_core_cycle[cpu]<<" cpu: "<<cpu<< endl;
            }
#endif



    return match_way;
}

int CACHE::invalidate_entry(uint64_t inval_addr, uint8_t thread_num)
{
    uint32_t set = get_set(inval_addr);
    int match_way = -1;

    if (NUM_SET < set) {
        cerr << "[" << NAME << "_ERROR] " << __func__ << " invalid set index: " << set << " NUM_SET: " << NUM_SET;
        cerr << " inval_addr: " << hex << inval_addr << dec << endl;
        assert(0);
    }

    // invalidate
    for (uint32_t way=0; way<NUM_WAY; way++) {
        if (block[set][way].valid && (block[set][way].tag == inval_addr) && (knob_cloudsuite || (((1 << LOG2_THREADS) - 1) & block[set][way].instr_id) == thread_num)) {

            block[set][way].valid = 0;

            match_way = way;

            //DP ( if (warmup_complete[cpu] ) {
            //cout << "[" << NAME << "] " << __func__ << " inval_addr: " << hex << inval_addr;  
            //cout << " tag: " << block[set][way].tag << " data: " << block[set][way].data << dec;
            //cout << " set: " << set << " way: " << way << " lru: " << block[set][way].lru << " cycle: " << current_core_cycle[cpu] << endl; });

            break;
        }
    }

    return match_way;
}

void CACHE::flush_TLB()
{
	for(uint32_t set=0; set<NUM_SET; set++)
	{
		for(uint32_t way=0; way<NUM_WAY; way++)
		{
			block[set][way].valid = 0;
		}
	}
}


int CACHE::add_rq(PACKET *packet)
{
    // check for the latest wirtebacks in the write queue 
    // @Vishal: WQ is non-fifo for L1 cache

	       	int wq_index;
    if(cache_type == IS_L1D || cache_type == IS_L1I)
	   wq_index = check_nonfifo_queue(&WQ,packet,false);
    else
	   wq_index = WQ.check_queue(packet, cache_type);

    if (wq_index != -1) {
     
	if(WQ.entry[wq_index].cpu != packet->cpu)
	{
		cout << " cache_type = " << cache_type << " address=" << packet->address << endl;
		//cout << "Read request from CPU " << packet->cpu << " merging with Write request from CPU " << WQ.entry[wq_index].cpu << endl;
		assert(0);
	}
 
	   //Neelu: 1 cycle WQ forwarding latency added. 
	if (packet->event_cycle < current_core_cycle[packet->cpu])
	        packet->event_cycle = current_core_cycle[packet->cpu] + 1;
    	else
	        packet->event_cycle += 1; 



        // check fill level
        if (packet->fill_level < fill_level) {

            packet->data = WQ.entry[wq_index].data;

	    if(fill_level == FILL_L2)
	    {
		    if(packet->fill_l1i)
		    {
			    upper_level_icache[packet->cpu]->return_data(packet);
		    }
		    if(packet->fill_l1d)
		    {
			    upper_level_dcache[packet->cpu]->return_data(packet);
		    }
	    }
	    else
	    {

            if (packet->instruction) 
                upper_level_icache[packet->cpu]->return_data(packet);
            else // data
                upper_level_dcache[packet->cpu]->return_data(packet);

	    }  
	}

#ifdef SANITY_CHECK
        if (cache_type == IS_ITLB)
            assert(0);
        else if (cache_type == IS_DTLB)
            assert(0);
		else if (cache_type ==  IS_STLB)
			assert(0);
        else if (cache_type == IS_L1I)
            assert(0);
#endif
        // update processed packets
        /*if ((cache_type == IS_L1D) && (packet->type != PREFETCH)) {
            if (PROCESSED.occupancy < PROCESSED.SIZE)
                PROCESSED.add_queue(packet);
        */

	 // @Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)
        if ((cache_type == IS_L1D) && (packet->type != PREFETCH) && (packet->type != LOAD_TRANSLATION))
#else
        if ((cache_type == IS_L1D) && (packet->type != PREFETCH))
#endif
        {
          if (PROCESSED.occupancy < PROCESSED.SIZE)
            PROCESSED.add_queue(packet);


          DP ( if (warmup_complete[packet->cpu][((1 << LOG2_THREADS) - 1) & packet->instr_id]) {
            cout << "[" << NAME << "_RQ] " << __func__ << " instr_id: " << (packet->instr_id >> LOG2_THREADS) << " found recent writebacks";
            cout << hex << " read: " << packet->address << " writeback: " << WQ.entry[wq_index].address << dec;
            cout << " index: " << MAX_READ << " rob_signal: " << packet->rob_signal << endl; });
        }

        HIT[packet->type]++;
        ACCESS[packet->type]++;

        WQ.FORWARD++;
        RQ.ACCESS++;

        return -1;
    }

    // check for duplicates in the read queue
    // @Vishal: RQ is non-fifo for L1 cache

    int index;
    if(cache_type == IS_L1D || cache_type == IS_L1I)
           index = check_nonfifo_queue(&RQ,packet,false);
    else
           index = RQ.check_queue(packet, cache_type);


#ifdef IDEAL_CACHE_FOR_TRANSLATION_ACCESS
 #ifdef IDEAL_L1D
    if(cache_type == IS_L1D && (packet->type == LOAD_TRANSLATION || packet->type == PREFETCH_TRANSLATION || packet->type == TRANSLATION_FROM_L1D) && index != -1)
 #endif

 #ifdef IDEAL_L2
    if(cache_type == IS_L2C && (packet->type == LOAD_TRANSLATION || packet->type == PREFETCH_TRANSLATION || packet->type == TRANSLATION_FROM_L1D) && index != -1)
 #endif

 #ifdef IDEAL_L3
    if(cache_type == IS_LLC && (packet->type == LOAD_TRANSLATION || packet->type == PREFETCH_TRANSLATION || packet->type == TRANSLATION_FROM_L1D) && index != -1)
 #endif
    {
        index = -1;
    }
#endif


    if (index != -1) {


		//@v send_both_tlb should be updated in STLB PQ if the entry needs to be serviced to both ITLB and DTLB
        if(cache_type == IS_STLB)
        {
        	/* Fill level of incoming request and prefetch packet should be same else STLB prefetch request(with instruction=1) might get 			merged with DTLB/ITLB, making send_both_tlb=1 due to a msimatch in instruction variable. If this happens, data will be returned to 			both ITLB and DTLB, incurring MSHR miss*/
        	
        	if(RQ.entry[index].fill_level == 1 && packet->fill_level == 1)
        	{
              		if((RQ.entry[index].instruction != packet-> instruction) && RQ.entry[index].send_both_tlb == 0)
               		{
			       RQ.entry[index].send_both_tlb = 1;
              		}
              }
        } 
        
        if(cache_type == IS_L2C)
	{
		if(RQ.entry[index].fill_level == 1 && packet->fill_level == 1)
		{
			if((RQ.entry[index].instruction != packet->instruction) && RQ.entry[index].send_both_cache == 0)
				RQ.entry[index].send_both_cache = 1;		
		}
	}
    
		 if(packet->fill_level < RQ.entry[index].fill_level)
            {
                RQ.entry[index].fill_level = packet->fill_level;
            }
   
  
	 if(RQ.entry[index].cpu != packet->cpu)
        {
                //cout << "Read request from CPU " << packet->cpu << " merging with Read request from CPU " << RQ.entry[index].cpu << endl;
                assert(0);
        }

 

	 
        if (packet->instruction) {
            uint32_t rob_index = packet->rob_index;
            RQ.entry[index].rob_index_depend_on_me.insert (rob_index);
            RQ.entry[index].instr_merged = 1;
	    RQ.entry[index].instruction = 1; // add as instruction type

	    //@Vishal: ITLB read miss request is getting merged with pending DTLB read miss request. when completed send to both caches
	    //@Vasudha: Done below
	    //if(cache_type == IS_STLB)
	    //	    RQ.entry[index].send_both_tlb = true;

            /*DP (if (warmup_complete[packet->cpu][((1 << LOG2_THREADS) - 1) & packet->instr_id]) {
            cout << "[" << NAME << "_INSTR_MERGED] " << __func__ << " cpu: " << packet->cpu << " instr_id: " << RQ.entry[index].instr_id;
            cout << " merged rob_index: " << rob_index << " instr_id: " << (packet->instr_id >> LOG2_THREADS) << endl; });
		*/
#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
		    //cout << "["<<NAME<<"_INSTR_MERGED] " << __func__ << " cpu: " << packet->cpu << " instr_id: " << RQ.entry[index].instr_id;
            //cout << " merged rob_index: " << rob_index << " instr_id: " << packet->instr_id << endl;
            }
#endif

		//Neelu: Commenting the following code and inserting modified code below as L1I prefetching is turned on now. 
		/*if(cache_type == IS_ITLB || cache_type == IS_DTLB || cache_type == IS_STLB)
		{
				RQ.entry[index].read_translation_merged = true;
				assert(packet->l1_rq_index != -1) 
				RQ.entry[index].l1_rq_index_depend_on_me.insert(packet->l1_rq_index);
		}*/

	    	if(cache_type == IS_DTLB)
		{
			RQ.entry[index].read_translation_merged = true;
                        assert(packet->l1_rq_index != -1); 
                        RQ.entry[index].l1_rq_index_depend_on_me.insert(packet->l1_rq_index);
		}

		//Neelu: packet can be PQ packet as well for ITLB and STLB as L1I prefetching can be enabled. 
		if(cache_type == IS_ITLB || cache_type == IS_STLB)
		{
			if(packet->l1_rq_index != -1){
                                RQ.entry[index].read_translation_merged = true;
                                RQ.entry[index].l1_rq_index_depend_on_me.insert(packet->l1_rq_index);

                                }
                        else if(packet->l1_pq_index != -1){
                		RQ.entry[index].prefetch_translation_merged = true;
        	        	RQ.entry[index].l1_pq_index_depend_on_me.insert(packet->l1_pq_index);
	                }
			
			if(packet->type==LOAD_TRANSLATION && RQ.entry[index].type==TRANSLATION_FROM_L1D)
			{
				assert(cache_type==IS_ITLB);
				RQ.entry[index].type = LOAD_TRANSLATION;
			}
		}
        }
        else 
        {
	/*@Vasudha: Not required
	 	if(packet->fill_level < fill_level)
			{
				RQ.entry[index].fill_level = packet->fill_level;
			}
	*/

            // mark merged consumer
			if(cache_type == IS_ITLB || cache_type == IS_DTLB || cache_type == IS_STLB)
			{
            			if (packet->l1_wq_index != -1) {
					RQ.entry[index].write_translation_merged = true;
        				RQ.entry[index].l1_wq_index_depend_on_me.insert(packet->l1_wq_index);
	        		}
        	    		else if(packet->l1_rq_index != -1){
					RQ.entry[index].read_translation_merged = true;
					RQ.entry[index].l1_rq_index_depend_on_me.insert(packet->l1_rq_index);
	
				}
				else if(packet->l1_pq_index != -1){
					assert(cache_type == IS_STLB);
                			RQ.entry[index].prefetch_translation_merged = true;
                			RQ.entry[index].l1_pq_index_depend_on_me.insert(packet->l1_pq_index);

                		}

				if(packet->type==LOAD_TRANSLATION && RQ.entry[index].type==TRANSLATION_FROM_L1D)
				{
					assert(cache_type==IS_STLB);
					RQ.entry[index].type = LOAD_TRANSLATION;
				}
			}
			else
			{
				if(packet->type == RFO)
				{	
					uint32_t sq_index = packet->sq_index;
                			RQ.entry[index].sq_index_depend_on_me.insert (sq_index);
                			RQ.entry[index].store_merged = 1;
             			}
            			else {
            				uint32_t lq_index = packet->lq_index; 
                			RQ.entry[index].lq_index_depend_on_me.insert (lq_index);
                			RQ.entry[index].load_merged = 1;
				}
	    RQ.entry[index].is_data = 1; // add as data type
			}



			if((packet->fill_l1i) && (RQ.entry[index].fill_l1i != 1))
			
	      		{

				RQ.entry[index].fill_l1i = 1;
			}
			if((packet->fill_l1d) && (RQ.entry[index].fill_l1d != 1))
			{
				RQ.entry[index].fill_l1d = 1;
			}


	    //@Vishal: DTLB read miss request is getting merged with pending ITLB read miss request. when completed send to both caches
            //@Vasudha: Done below
	    //if(cache_type == IS_STLB)
            //       RQ.entry[index].send_both_tlb = true;


               DP (if (warmup_complete[packet->cpu][((1<<LOG2_THREADS) - 1) & packet->instr_id] ) {
               cout << "["<<NAME<<"_DATA_MERGED] " << __func__ << " cpu: " << packet->cpu << " instr_id: " << (RQ.entry[index].instr_id>>3);
				cout << " Fill level: " << RQ.entry[index].fill_level;
				 if(RQ.entry[index].read_translation_merged)
                               cout << " read_translation_merged ";
                            if(RQ.entry[index].write_translation_merged)
                                cout << " write_translation_merged ";
                            if(RQ.entry[index].prefetch_translation_merged)
                                cout << " prefetch_translation_merged ";

                cout << " merged rob_index: " << packet->rob_index << " instr_id: " << (packet->instr_id >> LOG2_THREADS) << " lq_index: " << packet->lq_index << endl; });

#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
		    //cout << "["<<NAME<<"_DATA_MERGED] " << __func__ << " cpu: " << packet->cpu << " instr_id: " << RQ.entry[index].instr_id;
                //cout << " merged rob_index: " << packet->rob_index << " instr_id: " << packet->instr_id << " lq_index: " << packet->lq_index << endl;
            }
#endif
        }
        
        RQ.MERGED++;
        RQ.ACCESS++;

        return index; // merged index
    }
    //if(RQ.entry[index].instruction)
////cout << NAME << " NEW READ packet inserted with instructions, address =  \n\n" << hex << packet->address << endl ; 
    // check occupancy
    if (RQ.occupancy == RQ_SIZE) {
        RQ.FULL++;

        return -2; // cannot handle this request
    }

    bool translation_sent = false;
    int get_translation_index = -1;
    int get_translation_queue = IS_RQ;
    // if there is no duplicate, add it to RQ
    index = RQ.tail;

    //@Vishal: Since L1 RQ is non fifo, find empty index
    if(cache_type == IS_L1I || cache_type == IS_L1D)
    {
    	for (uint i = 0; i < RQ.SIZE; i++)
	    if(RQ.entry[i].address == 0)
	    {
		    index = i;
		    break;
	    }
    }

    //@Vishal: Check if pending translation sent to TLB
    if(cache_type == IS_L1I || cache_type == IS_L1D)
    {

	   if(cache_type == IS_L1I) // TODO: Check if extra interface can be used here?
	   {
			if(ooo_cpu[packet->cpu].ITLB.RQ.occupancy == ooo_cpu[packet->cpu].ITLB.RQ.SIZE)
			{
				ooo_cpu[packet->cpu].ITLB.RQ.FULL++;
				return -2; // cannot handle this request because translation cannot be sent to TLB
			}

			PACKET translation_packet = *packet;
			translation_packet.instruction = 1;
			translation_packet.fill_level = FILL_L1;
			translation_packet.l1_rq_index = index;
			translation_packet.type = LOAD_TRANSLATION;

			if (knob_cloudsuite)
				translation_packet.address = ((packet->ip >> LOG2_PAGE_SIZE) << 9) | ( 256 + packet->asid[0]);
			else
				translation_packet.address = packet->ip >> LOG2_PAGE_SIZE;

			ooo_cpu[packet->cpu].ITLB.add_rq(&translation_packet);
           }
           else 
           {
             // @Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)
             if(packet->type != LOAD_TRANSLATION && packet->type != PREFETCH_TRANSLATION && packet->type != TRANSLATION_FROM_L1D){
#endif

                if(ooo_cpu[packet->cpu].DTLB.RQ.occupancy == ooo_cpu[packet->cpu].DTLB.RQ.SIZE)
                {
                    ooo_cpu[packet->cpu].DTLB.RQ.FULL++;
                    return -2; // cannot handle this request because translation cannot be sent to TLB
                }

                PACKET translation_packet = *packet;
		translation_packet.instruction = 0;
		translation_packet.fill_level = FILL_L1;
                translation_packet.l1_rq_index = index;
		translation_packet.type = LOAD_TRANSLATION;
			
		if (knob_cloudsuite)
			translation_packet.address = ((packet->full_addr >> LOG2_PAGE_SIZE) << 9) | packet->asid[1];
		else
			translation_packet.address = packet->full_addr >> LOG2_PAGE_SIZE;

		//cout << "Cycle: " << dec << current_core_cycle[cpu] << " T" << (((1<<LOG2_THREADS)-1) & packet->instr_id) << " data address: " << hex << packet->address << " full_addr: " 
		 //      << packet->full_addr << " translation_address: " << translation_packet.address << endl;	
		ooo_cpu[packet->cpu].DTLB.add_rq(&translation_packet);

		//@Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)
             }
#endif
	   }
    }


#ifdef SANITY_CHECK
    if (RQ.entry[index].address != 0) {
        cerr << "[" << NAME << "_ERROR] " << __func__ << " is not empty index: " << index;
        cerr << " address: " << hex << RQ.entry[index].address;
        cerr << " full_addr: " << RQ.entry[index].full_addr << dec << endl;
        assert(0);
    }
#endif

    RQ.entry[index] = *packet;

    // ADD LATENCY
    if (RQ.entry[index].event_cycle < current_core_cycle[packet->cpu])
        RQ.entry[index].event_cycle = current_core_cycle[packet->cpu] + LATENCY;
    else
        RQ.entry[index].event_cycle += LATENCY;

    //@Rahul: PTW
#if defined(PTW_L1D)||defined(PTW_L1D_L2C)
    if (cache_type == IS_L1I || (cache_type == IS_L1D && (packet->type != LOAD_TRANSLATION && packet->type != PREFETCH_TRANSLATION && packet->type != TRANSLATION_FROM_L1D))) {
        RQ.entry[index].translated = INFLIGHT;
    }
#else
    if (cache_type == IS_L1I || cache_type == IS_L1D) {
        RQ.entry[index].translated = INFLIGHT;
    }
#endif

    /*
    if(cache_type == IS_L1I || cache_type == IS_L1D)
    {
	    RQ.entry[index].translated = INFLIGHT;
    	    //if (current_core_cycle[packet->cpu] >= 40)
		     //cout << "Cycle: "<< current_core_cycle[packet->cpu] << " index: " << index << " ip: " << RQ.entry[index].ip << " instr_id: " << RQ.entry[index].instr_id << " L1I RQ" <<  endl;
    }
    */

    RQ.occupancy++;
    RQ.tail++;
    if (RQ.tail >= RQ.SIZE)
        RQ.tail = 0;

  DP ( if (warmup_complete[RQ.entry[index].cpu][((1<<LOG2_THREADS)-1) & RQ.entry[index].instr_id] ) {
    cout << "[" << NAME << "_RQ] " <<  __func__ << " instr_id: " << (RQ.entry[index].instr_id >> LOG2_THREADS) << " address: " << hex << RQ.entry[index].address;
    cout << " full_addr: " << RQ.entry[index].full_addr << dec;
    cout << " type: " << +RQ.entry[index].type << " head: " << RQ.head << " tail: " << RQ.tail << " occupancy: " << RQ.occupancy;
    cout << " event: " << RQ.entry[index].event_cycle << " current: " << current_core_cycle[RQ.entry[index].cpu] << endl;});


#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
                    //cout << "[" << NAME << "_RQ] " <<  __func__ << " instr_id: " << RQ.entry[index].instr_id << " address: " << hex << RQ.entry[index].address;
    //cout << " full_addr: " << RQ.entry[index].full_addr << dec;
    //cout << " type: " << +RQ.entry[index].type << " head: " << RQ.head << " tail: " << RQ.tail << " occupancy: " << RQ.occupancy;
    //cout << " event: " << RQ.entry[index].event_cycle << " current: " << current_core_cycle[RQ.entry[index].cpu] << " cpu: "<<cpu<<endl;
            }
#endif


    if (packet->address == 0)
        assert(0);

    RQ.TO_CACHE++;
    RQ.ACCESS++;

    return -1;
}

int CACHE::add_wq(PACKET *packet)
{

	assert(cache_type != IS_L1I || cache_type != IS_ITLB || cache_type != IS_DTLB || cache_type != IS_STLB); //@Vishal: L1I cache does not have write packets

    // check for duplicates in the write queue
    int index;
    if(cache_type == IS_L1D)
    	index = check_nonfifo_queue(&WQ,packet,false);
    else 
    	index = WQ.check_queue(packet, cache_type);
    
    if (index != -1) {

	 if(WQ.entry[index].cpu != packet->cpu)
        {
                //cout << "Write request from CPU " << packet->cpu << " merging with Write request from CPU " << WQ.entry[index].cpu << endl;
                assert(0);
        }


        WQ.MERGED++;
        WQ.ACCESS++;

        return index; // merged index
    }

    // sanity check
    if (WQ.occupancy >= WQ.SIZE)
        assert(0);

   	bool translation_sent = false;
    int get_translation_index = -1;
    int get_translation_queue = IS_RQ;
    
   	// if there is no duplicate, add it to the write queue
    index = WQ.tail;

    //@Vishal: Since L1 WQ is non fifo, find empty index
    if(cache_type == IS_L1D)
    {
    	for (uint i = 0; i < WQ.SIZE; i++)
	    if(WQ.entry[i].address == 0)
	    {
		    index = i;
		    break;
	    }
    }

    //@Vishal: Check if pending translation sent to TLB
    if(cache_type == IS_L1D)
    {

		    if(ooo_cpu[packet->cpu].DTLB.RQ.occupancy == ooo_cpu[packet->cpu].DTLB.RQ.SIZE)
            {
                ooo_cpu[packet->cpu].DTLB.RQ.FULL++;
                return -2; // cannot handle this request because translation cannotbe sent to TLB
            }
            PACKET translation_packet = *packet;
	    translation_packet.instruction = 0;
            translation_packet.l1_wq_index = index;
	    translation_packet.fill_level = FILL_L1;
	    translation_packet.type = LOAD_TRANSLATION; 	
	    if (knob_cloudsuite)
           		translation_packet.address = ((packet->full_addr >> LOG2_PAGE_SIZE) << 9) | packet->asid[1];
            else
            	translation_packet.address = packet->full_addr >> LOG2_PAGE_SIZE;

            ooo_cpu[packet->cpu].DTLB.add_rq(&translation_packet);
    }


    if (WQ.entry[index].address != 0) {
        cerr << "[" << NAME << "_ERROR] " << __func__ << " is not empty index: " << index;
        cerr << " address: " << hex << WQ.entry[index].address;
        cerr << " full_addr: " << WQ.entry[index].full_addr << dec << endl;
        assert(0);
    }

    WQ.entry[index] = *packet;

    // ADD LATENCY
    if (WQ.entry[index].event_cycle < current_core_cycle[packet->cpu])
        WQ.entry[index].event_cycle = current_core_cycle[packet->cpu] + LATENCY;
    else
        WQ.entry[index].event_cycle += LATENCY;

    if(cache_type == IS_L1D)
	    WQ.entry[index].translated = INFLIGHT;

    WQ.occupancy++;
    WQ.tail++;
    if (WQ.tail >= WQ.SIZE)
        WQ.tail = 0;

  //DP (if (warmup_complete[WQ.entry[index].cpu]) {
    //cout << "[" << NAME << "_WQ] " <<  __func__ << " instr_id: " << WQ.entry[index].instr_id << " address: " << hex << WQ.entry[index].address;
    //cout << " full_addr: " << WQ.entry[index].full_addr << dec;
    //cout << " head: " << WQ.head << " tail: " << WQ.tail << " occupancy: " << WQ.occupancy;
    //cout << " data: " << hex << WQ.entry[index].data << dec;
    //cout << " event: " << WQ.entry[index].event_cycle << " current: " << current_core_cycle[WQ.entry[index].cpu] << endl;});


#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
                    //cout << "[" << NAME << "_WQ] " <<  __func__ << " instr_id: " << WQ.entry[index].instr_id << " address: " << hex << WQ.entry[index].address;
    //cout << " full_addr: " << WQ.entry[index].full_addr << dec;
    //cout << " head: " << WQ.head << " tail: " << WQ.tail << " occupancy: " << WQ.occupancy;
    //cout << " data: " << hex << WQ.entry[index].data << dec;
    //cout << " event: " << WQ.entry[index].event_cycle << " current: " << current_core_cycle[WQ.entry[index].cpu] << " cpu: "<<cpu<<endl;
            }
#endif

    WQ.TO_CACHE++;
    WQ.ACCESS++;

    return -1;
}

int CACHE::prefetch_line(uint64_t ip, uint64_t base_addr, uint64_t pf_addr, int pf_fill_level, uint32_t prefetch_metadata) /*, uint64_t prefetch_id)*/		//Neelu: commented. 
{
//	if(cache_type == IS_L2C)
//		//cout<<"Aye Aye, Captain, requested.";

	//Neelu: Todo: So, do all prefetches access STLB, even the same page ones? Is that correct? 
    pf_requested++;
	 //DP ( if (warmup_complete[cpu]) {//cout << "entered prefetch_line, occupancy = " << PQ.occupancy << "SIZE=" << PQ.SIZE << endl; });
    if (PQ.occupancy < PQ.SIZE) {
	    //if(cache_type == IS_L2C)
              //      //cout<<"Aye Aye, Captain, issued.";

             //DP ( if (warmup_complete[cpu]) {//cout << "packet entered in PQ" << endl; });
            PACKET pf_packet;
            pf_packet.fill_level = pf_fill_level;
	    pf_packet.pf_origin_level = fill_level;
	    if(pf_fill_level == FILL_L1)		   
	    {
		    pf_packet.fill_l1d = 1;
	    }

	    pf_packet.pf_metadata = prefetch_metadata;
            pf_packet.cpu = cpu;
            //pf_packet.data_index = LQ.entry[lq_index].data_index;
            //pf_packet.lq_index = lq_index;
            pf_packet.address = pf_addr >> LOG2_BLOCK_SIZE;
            pf_packet.full_addr = pf_addr;
	    pf_packet.full_virtual_address = pf_addr;
	    
#ifdef PUSH_PREFETCHES_FROM_L2_TO_L1

	    if(cache_type == IS_L1D)
	    {
		    //Neelu: Checking if the request is pushed from L2 or not,
		    if(((prefetch_metadata >> 16) & 1) == 1)
		    {
			pf_packet.translated = COMPLETED; 
			pf_packet.full_physical_address = pf_addr;
			assert(pf_packet.full_physical_address != 0);
			pf_pushed_from_L2C++;
		    }
	    }

#endif

            //pf_packet.instr_id = LQ.entry[lq_index].instr_id;
            //pf_packet.rob_index = LQ.entry[lq_index].rob_index;
            pf_packet.ip = ip;
            //pf_packet.prefetch_id = prefetch_id;		Neelu: commented, Vasudha was using for debugging. Assigning to zero for now.
	    pf_packet.prefetch_id = 0; 
            pf_packet.type = PREFETCH;
            pf_packet.event_cycle = current_core_cycle[cpu];

            // give a dummy 0 as the IP of a prefetch
            add_pq(&pf_packet);
			//DP ( if (warmup_complete[pf_packet.cpu]) {//cout << "returned from add_pq" << endl; });
            pf_issued++;

	    if(cache_type == IS_L1D)
	    {
		    if((base_addr >> LOG2_PAGE_SIZE) != (pf_addr >> LOG2_PAGE_SIZE))
			    cross_page_prefetch_requests++;
	    	    else
			    same_page_prefetch_requests++;
	    }

            return 1;
	
    }

    return 0;
}

int CACHE::prefetch_translation(uint64_t ip, uint64_t pf_addr, int pf_fill_level, uint64_t prefetch_metadata, uint64_t prefetch_id, uint8_t instruction)
{
    pf_requested++;
	 //DP ( if (warmup_complete[cpu]) {//cout << "entered prefetch_translation, occupancy = " << PQ.occupancy << "SIZE=" << PQ.SIZE << endl; });
    if (PQ.occupancy < PQ.SIZE) 
    {
        //DP ( if (warmup_complete[cpu]) {//cout << "packet entered in PQ" << endl; });
        PACKET pf_packet;
        pf_packet.fill_level = pf_fill_level;
	pf_packet.pf_origin_level = fill_level;
	pf_packet.pf_metadata = prefetch_metadata;
        pf_packet.cpu = cpu;
        pf_packet.instruction = instruction;
        //pf_packet.data_index = LQ.entry[lq_index].data_index;
        //pf_packet.lq_index = lq_index;
        pf_packet.address = pf_addr;
        if (knob_cloudsuite)
	{
		pf_packet.full_addr = (pf_addr >> 9) << LOG2_PAGE_SIZE;
		pf_packet.full_virtual_address = (pf_addr >> 9) << LOG2_PAGE_SIZE;
		pf_packet.asid[1] = pf_addr & 0x1FF;
	}
	else
	{
		pf_packet.full_addr = pf_addr << LOG2_PAGE_SIZE;
		pf_packet.full_virtual_address = pf_addr << LOG2_PAGE_SIZE;
	}
	//pf_packet.instr_id = LQ.entry[lq_index].instr_id;
        //pf_packet.rob_index = LQ.entry[lq_index].rob_index;
        pf_packet.ip = ip;
        
	pf_packet.prefetch_id = prefetch_id;
        pf_packet.type = PREFETCH_TRANSLATION;
        pf_packet.event_cycle = current_core_cycle[cpu];
        // give a dummy 0 as the IP of a prefetch
        add_pq(&pf_packet);
		//DP ( if (warmup_complete[pf_packet.cpu]) {//cout << "returned from add_pq" << endl; });
        pf_issued++;

	#ifdef DEBUG_PREF
	cout << "Cycle: " << current_core_cycle[cpu] <<" PREFETCH REQ. GENERATED: " << hex << pf_addr << dec << endl;
	#endif
	return 1;
    }

    return 0;
}

int CACHE::kpc_prefetch_line(uint64_t base_addr, uint64_t pf_addr, int pf_fill_level, int delta, int depth, int signature, int confidence, uint32_t prefetch_metadata)
{

	assert(0); //@Vishal: This should not be called


    if (PQ.occupancy < PQ.SIZE) {
       
        PACKET pf_packet;
        pf_packet.fill_level = pf_fill_level;
	    pf_packet.pf_origin_level = fill_level;
	    pf_packet.pf_metadata = prefetch_metadata;
        pf_packet.cpu = cpu;
        //pf_packet.data_index = LQ.entry[lq_index].data_index;
        //pf_packet.lq_index = lq_index;
        pf_packet.address = pf_addr >> LOG2_BLOCK_SIZE;
        pf_packet.full_addr = pf_addr;
        //pf_packet.instr_id = LQ.entry[lq_index].instr_id;
        //pf_packet.rob_index = LQ.entry[lq_index].rob_index;
        pf_packet.ip = 0;
        pf_packet.type = PREFETCH;
        pf_packet.delta = delta;
        pf_packet.depth = depth;
        pf_packet.signature = signature;
        pf_packet.confidence = confidence;
        pf_packet.event_cycle = current_core_cycle[cpu];

         if ((base_addr>>LOG2_PAGE_SIZE) == (pf_addr>>LOG2_PAGE_SIZE))
	 { 
         	pf_packet.full_physical_address = pf_addr;
		pf_packet.translated = COMPLETED;
	 }
        else
        	pf_packet.full_physical_address = 0;

        // give a dummy 0 as the IP of a prefetch
        int return_val = add_pq(&pf_packet);

        if(return_val > -2) //@Vishal: In some cases, even if the PQ is empty, request cannot be serviced.
        	pf_issued++;

        return 1;
    }

    return 0;
}

int CACHE::add_pq(PACKET *packet)
{

	assert(packet->type == PREFETCH || packet->type == PREFETCH_TRANSLATION);

	// @Vishal: L1I cache does not send prefetch request
	// Neelu: Added instruction prefetching support, so commenting this assert. 
	//assert(cache_type != IS_L1I);

    // check for the latest wirtebacks in the write queue 
    // @Vishal: WQ is non-fifo for L1 cache

    int wq_index;
    if(cache_type == IS_L1D || cache_type == IS_L1I)
	   wq_index = check_nonfifo_queue(&WQ,packet,false);
    else
	   wq_index = WQ.check_queue(packet, cache_type);

    if (wq_index != -1) {

	if(WQ.entry[wq_index].cpu != packet->cpu)
        {
                //cout << "Prefetch request from CPU " << packet->cpu << " merging with Write request from CPU " << WQ.entry[wq_index].cpu << endl;
                assert(0);
        }
      
	    //Neelu: Adding 1 cycle WQ forwarding latency
	   if (packet->event_cycle < current_core_cycle[packet->cpu])
		   packet->event_cycle = current_core_cycle[packet->cpu] + 1;
	   else
	           packet->event_cycle += 1; 


	   //Neelu: Todo: Is this sanity check sane? Removed check for L1-I 
#ifdef SANITY_CHECK

	if(cache_type == IS_ITLB || cache_type == IS_DTLB || cache_type == IS_STLB)
		assert(0);

#endif

 
        // check fill level
        if (packet->fill_level < fill_level) {

            packet->data = WQ.entry[wq_index].data;

	    if(fill_level == FILL_L2)
	    {
		    if(packet->fill_l1i)
		    {
			    upper_level_icache[packet->cpu]->return_data(packet);
		    }
		    if(packet->fill_l1d)
		    {
			    upper_level_dcache[packet->cpu]->return_data(packet);
		    }
	    }
	    else
	    {

            if (packet->instruction) 
                upper_level_icache[packet->cpu]->return_data(packet);
            else // data
                upper_level_dcache[packet->cpu]->return_data(packet);
            }
	}

        HIT[packet->type]++;
        ACCESS[packet->type]++;

        WQ.FORWARD++;
        PQ.ACCESS++;

        return -1;
    }

    // check for duplicates in the PQ
    int index = PQ.check_queue(packet, cache_type);
    if (index != -1) {
	if(PQ.entry[index].cpu != packet->cpu)
        {
                //cout << "Prefetch request from CPU " << packet->cpu << " merging with Prefetch request from CPU " << PQ.entry[index].cpu << endl;
                assert(0);
        }

	//@v send_both_tlb should be updated in STLB PQ if the entry needs to be serviced to both ITLB and DTLB
        if(cache_type == IS_STLB)
        {
        	/* Fill level of incoming request and prefetch packet should be same else STLB prefetch request(with instruction=1) might get 			merged with DTLB/ITLB, making send_both_tlb=1 due to a msimatch in instruction variable. If this happens, data will be returned to 			both ITLB and DTLB, incurring MSHR miss*/
        
        	if(PQ.entry[index].fill_level==1 && packet -> fill_level == 1)
        	{
              		if((PQ.entry[index].instruction != packet-> instruction) && PQ.entry[index].send_both_tlb == 0)
              		{        PQ.entry[index].send_both_tlb = 1;
              		}
              	}
        }
        
        if (packet->fill_level < PQ.entry[index].fill_level)
        {
            PQ.entry[index].fill_level = packet->fill_level;
            PQ.entry[index].instruction = packet->instruction; 
        }
        
	//@Vasudha: Fails when DTLB prefetch with instructions 0, STLB prefetch with instruction 0 and STLB prefetch with instruction 1 gets merged
	/*if((packet->instruction == 1) && (PQ.entry[index].instruction != 1))
	{
		PQ.entry[index].instruction = 1;
	}*/
	if((packet->is_data == 1) && (PQ.entry[index].is_data != 1))
	{
		PQ.entry[index].is_data = 1;
	}
	if((packet->fill_l1i) && (PQ.entry[index].fill_l1i != 1))
	{
		PQ.entry[index].fill_l1i = 1;
	}
	if((packet->fill_l1d) && (PQ.entry[index].fill_l1d != 1))
	{
		PQ.entry[index].fill_l1d = 1;
	}

        PQ.MERGED++;
        PQ.ACCESS++;

        return index; // merged index
    }
    
    // check occupancy
    if (PQ.occupancy == PQ_SIZE) {
        PQ.FULL++;

        //DP ( if (warmup_complete[packet->cpu]) {
        //cout << "[" << NAME << "] cannot process add_pq since it is full" << endl; });
        return -2; // cannot handle this request
    }

    // if there is no duplicate, add it to PQ
    index = PQ.tail;

#ifdef SANITY_CHECK
    if (PQ.entry[index].address != 0) {
        cerr << "[" << NAME << "_ERROR] " << __func__ << " is not empty index: " << index;
        cerr << " address: " << hex << PQ.entry[index].address;
        cerr << " full_addr: " << PQ.entry[index].full_addr << dec << endl;
        assert(0);
    }
#endif


    bool translation_sent = false;
    int get_translation_index = -1;
    int get_translation_queue = IS_RQ;


	//Neelu: Not adding any addition condition for skipping translation for prefetches pushed from L2 to L1 because full_phy_addr != 0.

    //@Vishal: Check if pending translation sent to TLB if its need to be translated
    if(cache_type == IS_L1D && packet->full_physical_address == 0)
    {
			if(ooo_cpu[packet->cpu].STLB.RQ.occupancy == ooo_cpu[packet->cpu].STLB.RQ.SIZE)
            {
                ooo_cpu[packet->cpu].STLB.RQ.FULL++;
                return -2; // cannot handle this request because translation cannot be sent to TLB
            }

            PACKET translation_packet = *packet;
            translation_packet.l1_pq_index = index;
            translation_packet.fill_level = FILL_L2;
	    translation_packet.type = TRANSLATION_FROM_L1D;		
		pf_requested++;
			if (knob_cloudsuite)
                translation_packet.address = ((packet->full_addr >> LOG2_PAGE_SIZE) << 9) | packet -> asid[1]; //@Vishal: TODO Check this address, will be wrong when L1I prefetcher is used
            else
                translation_packet.address = packet->full_addr >> LOG2_PAGE_SIZE;

	    //@Vasudha: To separate translation requests coming from L1D prefetcher and STLB prefetcher, change type to LOAD
	    translation_packet.type = 0;
            //@Vishal: Add translation packet from PQ to L2 cache.
            ooo_cpu[packet->cpu].STLB.add_rq(&translation_packet);
	    pf_issued++;
	}

    //Neelu: Adding translation request to ITLB for instruction prefetch requests.
    if(cache_type == IS_L1I && packet->full_physical_address == 0)
    {
	if(ooo_cpu[packet->cpu].ITLB.RQ.occupancy == ooo_cpu[packet->cpu].ITLB.RQ.SIZE)
	{
		ooo_cpu[packet->cpu].ITLB.RQ.FULL++;
		return -2; //cannot handle this request as ITLB read queue is full.
	}

	//ITLB RQ occupancy is not full.
	PACKET translation_packet = *packet;
	translation_packet.l1_pq_index = index;
	translation_packet.fill_level = FILL_L1;
	translation_packet.instruction = 1;
	translation_packet.type = TRANSLATION_FROM_L1D;
	//Neelu: As pf_v_addr is assigned to ip as well as full_addr in prefetch_code_line function, either will work for assigning address.
	if (knob_cloudsuite)
		translation_packet.address = ((packet->ip >> LOG2_PAGE_SIZE) << 9) | ( 256 + packet->asid[0]);
	else
		translation_packet.address = packet->ip >> LOG2_PAGE_SIZE;

	//Neelu: Assigning full virtual address to the packet.
	//Todo: Not sure of the implications to cloudsuite.
	translation_packet.full_virtual_address = packet->ip;
	
	ooo_cpu[packet->cpu].ITLB.add_rq(&translation_packet);	

    }


    PQ.entry[index] = *packet;
	PQ.entry[index].cycle_enqueued = current_core_cycle[cpu];

    //@Vasudha - if any TLB calls add_pq
    if(knob_cloudsuite && (cache_type==IS_ITLB || cache_type==IS_DTLB || cache_type==IS_STLB))
    {
	    if(PQ.entry[index].instruction == 1)
	    {
		PQ.entry[index].address = ((packet->ip >> LOG2_PAGE_SIZE) << 9) | ( 256 + packet->asid[0]);
	    }
	    else
	    	PQ.entry[index].address = ((packet->full_addr >> LOG2_PAGE_SIZE) << 9) | packet -> asid[1];
    }

    // ADD LATENCY
    if (PQ.entry[index].event_cycle < current_core_cycle[packet->cpu])
        PQ.entry[index].event_cycle = current_core_cycle[packet->cpu] + LATENCY;
    else
        PQ.entry[index].event_cycle += LATENCY ;

//Neelu: Adding condition to mark translated as INFLIGHT only if it is COMPLETED.
    if(cache_type == IS_L1D)
    {
#ifdef PUSH_PREFETCHES_FROM_L2_TO_L1
	if(PQ.entry[index].translated != COMPLETED)
#endif	   
		PQ.entry[index].translated = INFLIGHT;
    }

    //Neelu: Marking translations as inflight for L1I as well.
    if(cache_type == IS_L1I)
	   PQ.entry[index].translated = INFLIGHT; 
    

    PQ.occupancy++;
    PQ.tail++;
    if (PQ.tail >= PQ.SIZE)
        PQ.tail = 0;

  //DP ( if (warmup_complete[PQ.entry[index].cpu] ) {
    //cout << "[" << NAME << "_PQ] " <<  __func__ << " prefetch_id: " << PQ.entry[index].prefetch_id << " address: " << hex << PQ.entry[index].address;
    //cout << " full_addr: " << PQ.entry[index].full_addr << dec;
    //cout << " type: " << +PQ.entry[index].type << " head: " << PQ.head << " tail: " << PQ.tail << " occupancy: " << PQ.occupancy;
    //cout << " event: " << PQ.entry[index].event_cycle << " current: " << current_core_cycle[PQ.entry[index].cpu] << endl; });

#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
		    //cout << "[" << NAME << "_PQ] " <<  __func__ << " instr_id: " << PQ.entry[index].instr_id << " address: " << hex << PQ.entry[index].address;
		    //cout << " full_addr: " << PQ.entry[index].full_addr << dec;
		    //cout << " type: " << +PQ.entry[index].type << " head: " << PQ.head << " tail: " << PQ.tail << " occupancy: " << PQ.occupancy;
		    //cout << " event: " << PQ.entry[index].event_cycle << " current: " << current_core_cycle[PQ.entry[index].cpu] << endl;
            }
#endif

    if (packet->address == 0)
        assert(0);

    PQ.TO_CACHE++;
    PQ.ACCESS++;

    return -1;
}

void CACHE::return_data(PACKET *packet)
{
    // check MSHR information
    int mshr_index = check_nonfifo_queue(&MSHR, packet, true); //@Vishal: Updated from check_mshr

    // sanity check
    if (mshr_index == -1) {
        cerr << "[" << NAME << "_MSHR] " << __func__ << " instr_id: " << packet->instr_id << " prefetch_id: " << packet->prefetch_id  << " cannot find a matching entry!";
        cerr << " full_addr: " << hex << packet->full_addr;
        cerr << " address: " << packet->address << dec;
        cerr << " event: " << packet->event_cycle << " current: " << current_core_cycle[packet->cpu] << endl;
        assert(0);
    }

    // MSHR holds the most updated information about this request
    // no need to do memcpy
    MSHR.num_returned++;
    MSHR.entry[mshr_index].returned = COMPLETED;
    if(cache_type == IS_STLB)
    {
		packet->data >>= LOG2_PAGE_SIZE; //@Vishal: Remove last 12 bits from the data coming from PTW
    }
    MSHR.entry[mshr_index].data = packet->data;

    //@Vasudha: To Bypass DTLB
    //if (cache_type == IS_DTLB && packet->type == PREFETCH_TRANSLATION)
	//    MSHR.entry[mshr_index].type = packet->type;


    if(cache_type==IS_ITLB||cache_type==IS_DTLB||cache_type==IS_STLB)
    {
    	if(MSHR.entry[mshr_index].data == 0)
    	{
    		//cout << "return_data writes 0 in TLB.data\n";
    		assert(0);
    	}
    }
    MSHR.entry[mshr_index].pf_metadata = packet->pf_metadata;

    // ADD LATENCY
    if (MSHR.entry[mshr_index].event_cycle < current_core_cycle[packet->cpu])
        MSHR.entry[mshr_index].event_cycle = current_core_cycle[packet->cpu] + LATENCY;
    else
        MSHR.entry[mshr_index].event_cycle += LATENCY;

    update_fill_cycle();

    DP (if (warmup_complete[packet->cpu][((1 << LOG2_THREADS) - 1) & MSHR.entry[mshr_index].instr_id] ) {
    cout << "[" << NAME << "_MSHR] " <<  __func__ << " instr_id: " << (MSHR.entry[mshr_index].instr_id >> LOG2_THREADS);
    cout << " address: " << hex << MSHR.entry[mshr_index].address << " full_addr: " << MSHR.entry[mshr_index].full_addr;
    cout << " data: " << MSHR.entry[mshr_index].data << dec << " num_returned: " << MSHR.num_returned;
    cout << " index: " << mshr_index << " occupancy: " << MSHR.occupancy;
	if(MSHR.entry[mshr_index].read_translation_merged)
	   cout << " read_translation_merged ";
	else if(MSHR.entry[mshr_index].write_translation_merged)
		cout << " write_translation_merged ";
	else if(MSHR.entry[mshr_index].prefetch_translation_merged)
		cout << " prefetch_translation_merged ";

    cout << " event: " << MSHR.entry[mshr_index].event_cycle << " current: " << current_core_cycle[packet->cpu] << " next: " << MSHR.next_fill_cycle << endl; });

#ifdef PRINT_QUEUE_TRACE
    if(packet->instr_id == QTRACE_INSTR_ID)
    {
        //cout << "[" << NAME << "_MSHR] " <<  __func__ << " instr_id: " << (MSHR.entry[mshr_index].instr_id >> LOG2_THREADS);
	    //cout << " address: " << hex << MSHR.entry[mshr_index].address << " full_addr: " << MSHR.entry[mshr_index].full_addr;
	    //cout << " data: " << MSHR.entry[mshr_index].data << dec << " num_returned: " << MSHR.num_returned;
	    //cout << " index: " << mshr_index << " occupancy: " << MSHR.occupancy;
	    //cout << " event: " << MSHR.entry[mshr_index].event_cycle << " current: " << current_core_cycle[packet->cpu] << " next: " << MSHR.next_fill_cycle << endl;

    }
#endif

}

void CACHE::update_fill_cycle()
{
    // update next_fill_cycle
    
    uint64_t min_cycle = UINT64_MAX;
    uint32_t min_index = MSHR.SIZE;
    for (uint32_t i=0; i<MSHR.SIZE; i++) {
        if ((MSHR.entry[i].returned == COMPLETED) && (MSHR.entry[i].event_cycle < min_cycle)) {
            min_cycle = MSHR.entry[i].event_cycle;
            min_index = i;
        }

        //DP (if (warmup_complete[MSHR.entry[i].cpu] ) {
        //cout << "[" << NAME << "_MSHR] " <<  __func__ << " checking instr_id: " << (MSHR.entry[i].instr_id >> LOG2_THREADS);
        //cout << " address: " << hex << MSHR.entry[i].address << " full_addr: " << MSHR.entry[i].full_addr;
        //cout << " data: " << MSHR.entry[i].data << dec << " returned: " << +MSHR.entry[i].returned << " fill_level: " << MSHR.entry[i].fill_level;
        //cout << " index: " << i << " occupancy: " << MSHR.occupancy;
        //cout << " event: " << MSHR.entry[i].event_cycle << " current: " << current_core_cycle[MSHR.entry[i].cpu] << " next: " << MSHR.next_fill_cycle << endl; });
    }
    
    MSHR.next_fill_cycle = min_cycle;
    MSHR.next_fill_index = min_index;
    if (min_index < MSHR.SIZE) {

       //DP (if (warmup_complete[MSHR.entry[min_index].cpu] ) {
        //cout << "[" << NAME << "_MSHR] " <<  __func__ << " instr_id: " << (MSHR.entry[min_index].instr_id >> LOG2_THREADS);
        //cout << " address: " << hex << MSHR.entry[min_index].address << " full_addr: " << MSHR.entry[min_index].full_addr;
        //cout << " data: " << MSHR.entry[min_index].data << dec << " num_returned: " << MSHR.num_returned;
        //cout << " event: " << MSHR.entry[min_index].event_cycle << " current: " << current_core_cycle[MSHR.entry[min_index].cpu] << " next: " << MSHR.next_fill_cycle << endl; });
    }
}

//@Vishal: Made check_mshr generic; packet_direction (Required only for MSHR) =>true, going to lower level else coming from lower level
int CACHE::check_nonfifo_queue(PACKET_QUEUE *queue, PACKET *packet, bool packet_direction)
{
    uint64_t check_address = packet->address;

    //@Vishal: packet_direction will be true only for return_data function. We don't need to check address translation for that.
    if(!packet_direction && (cache_type == IS_L1I || cache_type == IS_L1D) && queue->NAME.compare(NAME+"_MSHR") == 0)
    {
	    if(packet->full_physical_address == 0)
	    {
	    	assert(packet->full_physical_address != 0); //@Vishal: If MSHR is checked, then address translation should be present 
	    }

	    if(packet->address != (packet->full_physical_address >> LOG2_BLOCK_SIZE))
		    check_address = packet->full_physical_address >> LOG2_BLOCK_SIZE; //@Vishal: L1 MSHR has physical address
    }
    
    if(cache_type == IS_L1D && queue->NAME.compare(NAME+"_WQ") == 0)
    {
	// search queue
            for (uint32_t index=0; index < queue->SIZE; index++) {
                if (queue->entry[index].full_addr == packet->full_addr) {
		    //@Vasudha:SMT: check thread=ID of the packet being searched and the one residing in queue  
		    if ((((1 << LOG2_THREADS) - 1) & queue->entry[index].instr_id) == (((1 << LOG2_THREADS) - 1) & packet->instr_id) || knob_cloudsuite)
		    {
                    	//DP ( if (warmup_complete[packet->cpu]) {
                    	//cout << "[" << NAME << "_" << queue->NAME << "] " << __func__ << " same entry instr_id: " << (packet->instr_id >> LOG2_THREADS) << " prior_id: " << (queue->entry[index].instr_id >> LOG2_THREADS);
                    	//cout << " address: " << hex << packet->address;
                    	//cout << " full_addr: " << packet->full_addr << dec << endl;});

                    	return index;
		    }
                }
            }

    }
    else
    {
	    // search queue
	    for (uint32_t index=0; index < queue->SIZE; index++) {
		if (queue->entry[index].address == check_address) {
		    //@Vasudha:SMT: check thread-ID of the packet being searched and the one residing in the queue
		    if (((( 1 << LOG2_THREADS) - 1) & queue->entry[index].instr_id) == ((( 1 << LOG2_THREADS) - 1) & packet->instr_id) || knob_cloudsuite)	    
		    {
			    DP ( if (warmup_complete[packet->cpu][((1<<LOG2_THREADS)-1) & packet->instr_id]) {
			    cout << "[" << NAME << "_" << queue->NAME << "] " << __func__ << " same entry instr_id: " << (packet->instr_id >> LOG2_THREADS) << " prior_id: ";
			    cout  << (queue->entry[index].instr_id >> LOG2_THREADS);
			    cout << " address: " << hex << packet->address;
			    cout << " full_addr: " << packet->full_addr << dec << endl; });
			    //@Vasudha: If packet from L1D and L1I gets merged in L2C MSHR
			    if(cache_type == IS_L2C)
			       if(queue->entry[index].fill_level == 1 && packet->fill_level == 1 && queue->entry[index].send_both_cache == 0)
				if(queue->entry[index].instruction != packet->instruction)
					queue->entry[index].send_both_cache = 1;
		   
		 		return index;
		    }
		}
	    }
    }

    //DP ( if (warmup_complete[packet->cpu]) {
    //cout << "[" << NAME << "_" << queue->NAME << "] " << __func__ << " new address: " << hex << packet->address;
    //cout << " full_addr: " << packet->full_addr << dec << endl; });

    //DP ( if (warmup_complete[packet->cpu] && (queue->occupancy == queue->SIZE)) { 
    //cout << "[" << NAME << "_" << queue->NAME << "] " << __func__ << " mshr is full";
    //cout << " instr_id: " << (packet->instr_id >> LOG2_THREADS) << " occupancy: " << queue->occupancy;
    //cout << " address: " << hex << packet->address;
    //cout << " full_addr: " << packet->full_addr << dec;
    //cout << " cycle: " << current_core_cycle[packet->cpu] << endl;});

    return -1;
}

//@Vishal: Made add_mshr generic
void CACHE::add_nonfifo_queue(PACKET_QUEUE *queue, PACKET *packet)
{
    uint32_t index = 0;

    packet->cycle_enqueued = current_core_cycle[packet->cpu];

    // search queue
    for (index=0; index < queue->SIZE; index++) {
        if (queue->entry[index].address == 0) {
            
            queue->entry[index] = *packet;
            queue->entry[index].returned = INFLIGHT;
            queue->occupancy++;

            DP ( if (warmup_complete[packet->cpu][((1<<LOG2_THREADS)-1) & packet->instr_id]) {
            cout << "[" << NAME << "_" << queue->NAME << "] " << __func__ << " instr_id: " << (packet->instr_id >> LOG2_THREADS);
            cout << " address: " << hex << packet->address << " full_addr: " << packet->full_addr << dec;
			if(packet->read_translation_merged)
				cout << " read_translation_merged ";
			else if(packet->write_translation_merged)
				cout << " write_translation_merged ";
			else if(packet->prefetch_translation_merged)
				cout << " prefetch_translation_merged ";
			cout << " fill_level: " << queue->entry[index].fill_level;
            cout << " index: " << index << " occupancy: " << queue->occupancy << endl; });


#ifdef PRINT_QUEUE_TRACE
            if(packet->instr_id == QTRACE_INSTR_ID)
            {
                //cout << "[" << NAME << "_MSHR] " << __func__ << " instr_id: " << packet->instr_id;
                //cout << " address: " << hex << packet->address << " full_addr: " << packet->full_addr << dec<<endl;
                //cout << " index: " << index << " occupancy: " << MSHR.occupancy << " cpu: "<<cpu<<endl;
            }
#endif

            break;
        }
    }
}

uint32_t CACHE::get_occupancy(uint8_t queue_type, uint64_t address)
{
    if (queue_type == 0)
        return MSHR.occupancy;
    else if (queue_type == 1)
        return RQ.occupancy;
    else if (queue_type == 2)
        return WQ.occupancy;
    else if (queue_type == 3)
        return PQ.occupancy;

    return 0;
}

uint32_t CACHE::get_size(uint8_t queue_type, uint64_t address)
{
    if (queue_type == 0)
        return MSHR.SIZE;
    else if (queue_type == 1)
        return RQ.SIZE;
    else if (queue_type == 2)
        return WQ.SIZE;
    else if (queue_type == 3)
        return PQ.SIZE;

    return 0;
}

void CACHE::increment_WQ_FULL(uint64_t address)
{
    WQ.FULL++;
}
