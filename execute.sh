
echo " -warmup_instructions 10000000 -simulation_instructions 10000000 -threads 2 -cloudsuite -traces tracer/cloudsuite/cassandra_phase0_core0.trace.gz tracer/cloudsuite/cassandra_phase0_core1.trace.gz > check1.txt "

#echo " -warmup_instructions 10000000 -simulation_instructions 10000000 -threads 2 -traces tracer/spec_2006/mcf_158B.trace.gz tracer/spec_2006/GemsFDTD_716B.trace.gz > check2.txt &"

# -warmup_instructions 10000000 -simulation_instructions 10000000 -threads 2 -traces tracer/spec/605.mcf_s-782B.champsimtrace.xz tracer/spec/620.omnetpp_s-874B.champsimtrace.xz > check3.txt &


#./run_SMT_2T.sh hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-srrip-drrip-lru-lru-lru-1core  50 10 tracer/cloudsuite cassandra_phase0_core0.trace.gz cassandra_phase0_core1.trace.gz  -cloudsuite  &
