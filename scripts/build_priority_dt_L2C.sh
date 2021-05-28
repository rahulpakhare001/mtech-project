prev_translation_priority=0
prev_data_priority=0
for translation_priority in 0 1 2 3 4 5 6 7
do
    echo "Translation Insertion Priority: $translation_priority"
    sed -i.bak 's/\<L2C_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/L2C_TRANSLATION_INSERTION_PRIORITY '${translation_priority}'/g' replacement/priority.l2c_repl


    for data_priority in 0 1 2 3 4 6 5 7
    do
	if [ "$translation_priority" -le "$data_priority" ]
	then

        	echo "Data Insertion Priority: $data_priority"
	        sed -i.bak 's/\<L2C_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/L2C_DATA_INSERTION_PRIORITY '${data_priority}'/g' replacement/priority.l2c_repl
        	head -6 replacement/priority.l2c_repl
	        ./build_champsim.sh hashed_perceptron no no no no no no no lru lru lru priority ship lru lru lru 1
       		 mv bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-priority-ship-lru-lru-lru-1core bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-priority_${translation_priority}_${data_priority}-ship-lru-lru-lru-1core

	        prev_data_priority=$data_priority
        fi

    done

    prev_translation_priority=$translation_priority
done
sed -i.bak 's/\<L2C_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/L2C_TRANSLATION_INSERTION_PRIORITY '0'/g' replacement/priority.l2c_repl
sed -i.bak 's/\<L2C_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/L2C_DATA_INSERTION_PRIORITY '0'/g' replacement/priority.l2c_repl
