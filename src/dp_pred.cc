#ifdef ARKA_DP_PRED
// Implementation of Dead Page Predictor (HPCA 21)

#include "champsim.h"
#include "block.h"
#include "cache.h"
#include "ooo_cpu.h"

#define HASH_IP 6
#define HASH_VPN 4
#define pHIST_IP 64
#define pHIST_VPN 16
#define NUM_ENTRIES_ST 2
#define THRESHOLD 6

int index_st = 0;
uint16_t pHIST[MAX_THREADS][pHIST_IP][pHIST_VPN];
uint64_t shadow_table[MAX_THREADS][NUM_ENTRIES_ST][2];	//stores Virtual page and Physical page number
void insert_shadow_table(PACKET *packet);
//uint64_t corr_pred, 

// compute hash - for IP + VPN
uint64_t compute_hash(uint8_t num_bits, uint64_t address)
{
	uint64_t hash = 0;
	while (address != 0)
	{
		hash = hash ^ (address & ((1 << num_bits) - 1));
		address = address >> num_bits;
	}
	return hash;
}

// STLB Lookup
//STLB Miss - check_shadow_table (hit + miss handled, flush entry)
uint64_t check_shadow_table(PACKET *packet)
{
	uint8_t hit = 0;
	int i;
	uint16_t thread_id = ((1 << LOG2_THREADS) - 1) & packet->instr_id;
	for (i = 0; i < NUM_ENTRIES_ST; i++)
	{
		if (shadow_table[thread_id][i][0] == packet->full_virtual_address >> LOG2_PAGE_SIZE)
		{
			hit = 1;
			break;
		}
	}

	if (hit == 0)	// Shadow table miss
		return 0;
	else		// Shadow table hit
	{
		packet->data = shadow_table[thread_id][i][1];
		//cout << "Shadow table hit: " << hex << (packet->full_virtual_address >> LOG2_PAGE_SIZE) << dec << endl; 	
		//Misprediction - Flush columns of hash of virtual page number
		uint64_t hash = compute_hash(HASH_VPN, packet->full_virtual_address >> LOG2_PAGE_SIZE);
		for (int j = 0; j < pHIST_IP; j++)
			pHIST[thread_id][j][hash] = 0;

		shadow_table[thread_id][i][0] = 0;
		return 1;
	}
}

// STLB Fill
//lookup pHIST - check threshold & handle
uint64_t lookup_pHIST(PACKET *packet)
{
	uint64_t hash_ip = compute_hash(HASH_IP, packet->ip);
	uint64_t hash_vpn = compute_hash(HASH_VPN, packet->full_virtual_address >> LOG2_PAGE_SIZE);
	uint16_t thread_id = packet->instr_id & ((1 << LOG2_THREADS) - 1);
	if (pHIST[thread_id][hash_ip][hash_vpn] > THRESHOLD)
	{
		//Bypass STLB and store in Shadow table
		//cout <<hex<< hash_ip << " "<<hash_vpn << " inserted in shadow table " << (packet->full_virtual_address >> LOG2_PAGE_SIZE) << dec<<endl; 
		insert_shadow_table(packet);
		return 1;
	}
	else
		return 0;
}

void insert_shadow_table(PACKET *packet)
{
	//check Shadow table
	//check availability of invalid entries
	//remove one entry
	//INSERT
	uint16_t thread_id = ((1 << LOG2_THREADS) - 1) & packet->instr_id;
	int flag = 0, way;
	for (way = 0; way < NUM_ENTRIES_ST; way++)
	{
		if (shadow_table[thread_id][way][0] == packet->full_virtual_address >> LOG2_PAGE_SIZE)
		{
			//cout << "Entry already in Shadow thable\n"; 
			flag = 1;
			return;
		}
	}	

	if (flag == 0)	// not found in Shadow table
	{
		for (way = 0; way < NUM_ENTRIES_ST; way++)
		{
			if (shadow_table[thread_id][way][0] == 0)
			{
				flag = 1;	//invalid entry found
				break;
			}
		}
	}

	if (flag == 0)
	{
		way = index_st;
		PACKET evicted_st;
		evicted_st.instr_id = thread_id;
		evicted_st.full_virtual_address = shadow_table[thread_id][way][0] << LOG2_PAGE_SIZE;
		evicted_st.data = shadow_table[thread_id][way][1];
		uint32_t set, victim_way;
		set = ooo_cpu[0].VICTIM_ST.get_set(evicted_st.full_virtual_address >> LOG2_PAGE_SIZE);
		victim_way = ooo_cpu[0].VICTIM_ST.base_find_victim(0, thread_id, set, ooo_cpu[0].VICTIM_ST.block[set], 0, evicted_st.full_virtual_address, 4);

		//Check if victim block was correctly predicted as DOA or not
		if (all_warmup_complete > ooo_cpu[0].thread)
		{
			if (ooo_cpu[0].VICTIM_ST.block[set][victim_way].used == 0 && ooo_cpu[0].VICTIM_ST.block[set][way].valid == 1)
				corrpred_stlb_bypass++;
			else
				mispred_stlb_bypass++;
		}
	        ooo_cpu[0].VICTIM_ST.base_update_replacement_state(0, set, victim_way, evicted_st.full_virtual_address, 0, 0, 4, 0); 	
		ooo_cpu[0].VICTIM_ST.fill_cache(set, victim_way, &evicted_st);
		index_st++;
		if (index_st == NUM_ENTRIES_ST)
			index_st = 0;
	}

	shadow_table[thread_id][way][0] = packet->full_virtual_address >> LOG2_PAGE_SIZE;
	shadow_table[thread_id][way][1] = packet->data;
	return;
}

// STLB eviction
void modify_pHIST(BLOCK *block)
{
		uint64_t hash_ip = compute_hash(HASH_IP, block->ip);
		uint64_t hash_vpn = compute_hash(HASH_VPN, block->full_addr >> LOG2_PAGE_SIZE);
		uint16_t thread_id = ((1 << LOG2_THREADS) - 1) & block->instr_id;
		if (block->used == 0)
		{
			if (pHIST[thread_id][hash_ip][hash_vpn] != 0x7)	
				pHIST[thread_id][hash_ip][hash_vpn]++;
		}
		else
		{
			pHIST[thread_id][hash_ip][hash_vpn] = 0;
		}
		return;
}
#endif
