<p align="center">
  <h1 align="center"> ChampSim </h1>
  <p> ChampSim is a trace-based simulator for a microarchitecture study. It is easy to modify and a new replacement policy, branch predictor or prefetcher can be added to the existing codebase. We are using ChampSim for single-core simulation however it can also be used for multicore workloads. It takes application traces as input which can be generated using PIN tool.  Our group has enhanced the original ChampSim available at repository(https://github.com/ChampSim/ChampSim). <br>
  The enhancements as follows: <br>
          1. A 5-level Page Table and a Page Table Walker. <br>
          2. VIPT L1I and L1D Cache. <br>
          3. Paging-Structure Caches. <br>
          4. Support for SMT (simultaneous multithreading). <br>
  <p>
</p>


# Compile
ChampSim takes 17 parameters: Branch Predictor, L1I Prefetcher, L1D Prefetcher, L2C Prefetcher, LLC Prefetcher, ITLB Prefetcher, DTLB Prefetcher, STLB Prefetcher, BTB Replacement Policy, L1I Replacement Policy, L1D Replacement Policy, L2C Replacement Policy, LLC Replacement Policy, ITLB Replacement Policy, DTLB Replacement Policy, STLB Replacement Policy and the number of cores. For example following command builds single core processor with hashed-perceptron branch predictor, <br>
no prefetcher for L1I, L1D, L2C, LLC, ITLB, DTLB and STLB, <br>
LRU replacement policy for BTB, L1I, L1D, L2C, LLC, ITLB, DTLB and STLB. <br>

```
$ ./build_champsim.sh hashed_perceptron no no no no no no no lru lru lru lru lru lru lru lru 1

Usage: ./build_champsim.sh [branch_pred] [l1i_pref] [l1d_pref] [l2c_pref] [llc_pref] [itlb_pref] [dtlb_pref] [stlb_pref] [btb_repl] [l1i_repl] [l1d_repl] [l2c_repl] [llc_repl] [itlb_repl] [dtlb_repl] [stlb_repl] [num_core]

```

We have optimized LRU and SHiP replacement policy for L2 and LLC. They can be compiled with following command:

```
$ ./build_champsim.sh hashed_perceptron no no no no no no no lru lru lru opt_lru opt_ship lru lru lru 1

```

# Run Simulation
Execute `run_SMT_1T.sh` for single-thread execution and `run_SMT_2T.sh` for 2-thread SMT execution.

Following command executes previously compiled binary with single thread.
```
$ ./run_SMT_1T.sh hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-opt_lru-opt_ship-lru-lru-lru-1core 1 1 /home/rahul/Documents/traces BFS_61B.trace.gz

Usage: ./run_SMT_1T.sh [BINARY] [N_WARM] [N_SIM] [TRACE_DIR] [TRACE_T1]

${BINARY}: ChampSim binary compiled by "build_champsim.sh" (hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-opt_lru-opt_ship-lru-lru-lru-1core)
${N_WARM}: number of instructions for warmup (50 million)
${N_SIM}:  number of instructinos for detailed simulation (50 million)
$[TRACE_DIR]: location of of trace directory
${TRACE_T1}: trace name for single thread (BFS_61B.trace.gz)

```


Following command executes previously compiled binary with 2-thread SMT execution.
```
$ ./run_SMT_2T.sh hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-opt_lru-opt_ship-lru-lru-lru-1core 50 50 /home/rahul/Documents/traces BFS_61B.trace.gz PageRank_1B.trace.gz

Usage: ./run_SMT_2T.sh [BINARY] [N_WARM] [N_SIM] [TRACE_DIR] [TRACE_T1] [TRACE_T2]

${TRACE_T2}:  trace name for second thread (PageRank_1B.trace.gz)
```
Simulation results will be stored under "results_${N_SIM}M" directory.<br>


Source code for all the replacemnent policy used in our project are available in folder `replacement`. New replacement policies can be added or existing ones can be edited. We have modified LRU and SHiP replacement polices for our project as `opt_lru.l2c_repl` and `opt_ship.llc_repl`.

# Evaluate Simulation

ChampSim measures the IPC (Instruction Per Cycle) value as a performance metric. <br>
We also measure cache metrics such as hit rate, miss rate and MPKI which are needed for our project. Other useful metrics are also printed out at the end of simulation. <br>
