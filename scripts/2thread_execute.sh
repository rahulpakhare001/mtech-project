BINARY=${1}
TRACE_LIST=$(cat ${2})
TRACE_DIR=${3}

count=0

for trace1 in $TRACE_LIST;
do

  for trace2 in $TRACE_LIST;
  do
          echo "$trace1 $trace2"
          # count=`expr $count + 1`
          # if [ $count -lt 2 ]
          # then
          #         ./run_SMT_2T.sh $BINARY 50 50 $TRACE_DIR $trace1 $trace2  &
          # else
          #         ./run_SMT_2T.sh $BINARY 50 50 $TRACE_DIR $trace1 $trace2
          #         count=0
          # fi
  done

done

echo "Done"
