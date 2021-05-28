BINARY=${1}
TRACE_LIST=${2}
TRACE_DIR=${3}

count=0
cat $TRACE_LIST | \
while read args;
do
  	 echo $args
	 count=`expr $count + 1`
          if [ $count -lt 12 ]
          then
                  ./run_SMT_2T.sh $BINARY 0 1 $TRACE_DIR $args &
          else
                  ./run_SMT_2T.sh $BINARY 0 1 $TRACE_DIR $args
                  count=0
          fi
done 
