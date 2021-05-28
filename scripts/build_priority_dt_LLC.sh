prev_translation_priority=0
prev_data_priority=0
for translation_priority in 0 4 8 12 15
do
    echo "Translation Insertion Priority: $translation_priority"
    sed -i.bak 's/\<LLC_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/LLC_TRANSLATION_INSERTION_PRIORITY '${translation_priority}'/g' replacement/priority.llc_repl


    for data_priority in 0 4 8 12 15
    do
        echo "Data Insertion Priority: $data_priority"
        sed -i.bak 's/\<LLC_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/LLC_DATA_INSERTION_PRIORITY '${data_priority}'/g' replacement/priority.llc_repl
        head -6 replacement/priority.llc_repl
       ./build_champsim.sh bimodal no no no no no no no lru lru lru priority priority lru lru lru 1
       mv bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority-priority-lru-lru-lru-1core bin/bimodal-no-no-no-no-no-no-no-lru-lru-lru-priority-priority_${translation_priority}_${data_priority}-lru-lru-lru-1core

        prev_data_priority=$data_priority
    done

    prev_translation_priority=$translation_priority
done
sed -i.bak 's/\<LLC_TRANSLATION_INSERTION_PRIORITY '${prev_translation_priority}'\>/LLC_TRANSLATION_INSERTION_PRIORITY '0'/g' replacement/priority.llc_repl
sed -i.bak 's/\<LLC_DATA_INSERTION_PRIORITY '${prev_data_priority}'\>/LLC_DATA_INSERTION_PRIORITY '0'/g' replacement/priority.llc_repl
