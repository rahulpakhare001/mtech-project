prev_translation_priority=0
prev_data_priority=0
for translation_priority in 0 1 2
do
    echo "Translation Insertion Priority: $translation_priority"
    sed -i.bak 's/\<LLC_TRANSLATION_INSERTION_RRPV '${prev_translation_priority}'\>/LLC_TRANSLATION_INSERTION_RRPV '${translation_priority}'/g' replacement/ship.llc_repl


    for data_priority in 2
    do
        echo "Data Insertion Priority: $data_priority"
        sed -i.bak 's/\<LLC_DATA_INSERTION_RRPV '${prev_data_priority}'\>/LLC_DATA_INSERTION_RRPV '${data_priority}'/g' replacement/ship.llc_repl
        head -6 replacement/ship.llc_repl
        ./build_champsim.sh hashed_perceptron no no no no no no no lru lru lru priority ship lru lru lru 1
        mv bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-priority-ship-lru-lru-lru-1core bin/hashed_perceptron-no-no-no-no-no-no-no-lru-lru-lru-priority-ship_b_${translation_priority}_${data_priority}-lru-lru-lru-1core

        prev_data_priority=$data_priority
    done

    prev_translation_priority=$translation_priority
done
sed -i.bak 's/\<LLC_TRANSLATION_INSERTION_RRPV '${prev_translation_priority}'\>/LLC_TRANSLATION_INSERTION_RRPV '0'/g' replacement/ship.llc_repl
sed -i.bak 's/\<LLC_DATA_INSERTION_RRPV '${prev_data_priority}'\>/LLC_DATA_INSERTION_RRPV '0'/g' replacement/ship.llc_repl
