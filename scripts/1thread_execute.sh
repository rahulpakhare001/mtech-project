BINARY=${1}
TRACE_LIST=$(cat ${2})
TRACE_DIR=${3}

count=0

for trace in $TRACE_LIST;
do
        count=`expr $count + 1`
        if [ $count -lt 2 ]
        then
                echo "Number of traces simulated - $count in iteration $iterator"
                ./run_SMT_1T.sh $BINARY 50 50 $TRACE_DIR $trace  &
        else
                ./run_SMT_1T.sh $BINARY 50 50 $TRACE_DIR $trace
                count=0
        fi
done

echo "Done"
