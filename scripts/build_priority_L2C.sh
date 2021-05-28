prev_priority=0
for ((priority=1; priority<8; priority++));
do
   echo "Data Insertion Priority: $priority"
   sed -i.bak 's/\<L2C_DATA_INSERTION_PRIORITY '${prev_priority}'\>/L2C_DATA_INSERTION_PRIORITY '${priority}'/g' replacement/priority.l2c_repl
   head -6 replacement/priority.l2c_repl
   ./build_champsim.sh bimodal no no no no no no no lru lru lru priority lru lru lru lru 1
   mv bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority-lru-lru-lru-lru-1core bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority_0_${priority}-lru-lru-lru-lru-1core

   prev_priority=$priority
done

echo "Data Insertion Priority: 0"
sed -i.bak 's/\<L2C_DATA_INSERTION_PRIORITY 7\>/L2C_DATA_INSERTION_PRIORITY 0/g' replacement/priority.l2c_repl
head -6 replacement/priority.l2c_repl
./build_champsim.sh bimodal no no no no no no no lru lru lru priority lru lru lru lru 1
mv bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority-lru-lru-lru-lru-1core bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority_0_0-lru-lru-lru-lru-1core
