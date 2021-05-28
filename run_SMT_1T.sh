#!/bin/bash

if [ "$#" -lt 5 ] ; then
    echo "Illegal number of parameters"
    echo "Usage: ./run_4core.sh [BINARY] [N_WARM] [N_SIM] [N_MIX] [OPTION] [THREAD_CORE_1] [THREAD_CORE_2] [TRACE]"
    exit 1
fi

DEFAULT_THREAD=1
TRACE_DIR=${4}
BINARY=${1}
N_WARM=${2}
N_SIM=${3}
#N_MIX=${4}

OPTION=${7}
THREAD_0=${5}
#THREAD_1=${6}
#TRACE=${8}

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

#if [ ! -f "$TRACE_DIR/$THREAD_1" ] ; then
#   echo "[ERROR] Cannot find a trace1 file: $TRACE_DIR/$THREAD_1"
#    exit 1
#fi



mkdir -p results_1core_${N_SIM}M
(./bin/${BINARY} -warmup_instructions ${N_WARM}000000 -simulation_instructions ${N_SIM}000000   -threads 1 ${OPTION}  -traces   ${TRACE_DIR}/${THREAD_0}  )  &> results_1core_${N_SIM}M/${THREAD_0}-${BINARY}.txt
