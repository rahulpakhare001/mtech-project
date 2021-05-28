prev_translation_priority=0
prev_data_priority=0
for translation_priority in 0 2 4 8 6 10 11
do
    echo "Translation Insertion Priority: $translation_priority"
    sed -i.bak 's/\<L1D_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/L1D_TRANSLATION_INSERTION_PRIORITY '${translation_priority}'/g' replacement/priority.l1d_repl


    for data_priority in 0 2 4 8 6 10 11
    do
        echo "Data Insertion Priority: $data_priority"
        sed -i.bak 's/\<L1D_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/L1D_DATA_INSERTION_PRIORITY '${data_priority}'/g' replacement/priority.l1d_repl
        head -6 replacement/priority.l1d_repl
        ./build_champsim.sh hashed_perceptron no no no no no no no lru lru priority lru ship lru lru lru 1
        mv bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-priority-lru-ship-lru-lru-lru-1core bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-priority_${translation_priority}_${data_priority}-lru-ship-lru-lru-lru-1core

        prev_data_priority=$data_priority
    done

    prev_translation_priority=$translation_priority
done
sed -i.bak 's/\<L1D_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/L1D_TRANSLATION_INSERTION_PRIORITY '0'/g' replacement/priority.l1d_repl
sed -i.bak 's/\<L1D_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/L1D_DATA_INSERTION_PRIORITY '0'/g' replacement/priority.l1d_repl
