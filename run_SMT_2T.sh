#!/bin/bash

if [ "$#" -lt 5 ] ; then
    echo "Illegal number of parameters"
    echo "Usage: ./run_SMT_1T.sh [BINARY] [N_WARM] [N_SIM] [TRACE_DIR] [TRACE_T1] [TRACE_T2]"
    exit 1
fi

DEFAULT_THREAD=1

BINARY=${1}
N_WARM=${2}
N_SIM=${3}
TRACE_DIR=${4}
THREAD_0=${5}
THREAD_1=${6}
OPTION=${7}


# Sanity check
if [ -z $TRACE_DIR ] || [ ! -d "$TRACE_DIR" ] ; then
    echo "[ERROR] Cannot find a trace directory: $TRACE_DIR"
    exit 1
fi

if [ ! -f "bin/$BINARY" ] ; then
    echo "[ERROR] Cannot find a ChampSim binary: bin/$BINARY"
    exit 1
fi

re='^[0-9]+$'
if ! [[ $N_WARM =~ $re ]] || [ -z $N_WARM ] ; then
    echo "[ERROR]: Number of warmup instructions is NOT a number" >&2;
    exit 1
fi

re='^[0-9]+$'
if ! [[ $N_SIM =~ $re ]] || [ -z $N_SIM ] ; then
    echo "[ERROR]: Number of simulation instructions is NOT a number" >&2;
    exit 1
fi

if [ ! -f "$TRACE_DIR/$THREAD_0" ] ; then
    echo "[ERROR] Cannot find a trace0 file: $TRACE_DIR/$THREAD_0"
    exit 1
fi

if [ ! -f "$TRACE_DIR/$THREAD_1" ] ; then
   echo "[ERROR] Cannot find a trace1 file: $TRACE_DIR/$THREAD_1"
    exit 1
fi



mkdir -p results_${N_SIM}M
(./bin/${BINARY} -warmup_instructions ${N_WARM}000000 -simulation_instructions ${N_SIM}000000   -threads 2 ${OPTION}  -traces   ${TRACE_DIR}/${THREAD_0} ${TRACE_DIR}/${THREAD_1}  )  &> results_${N_SIM}M/${THREAD_0}-${THREAD_1}-${BINARY}.txt
