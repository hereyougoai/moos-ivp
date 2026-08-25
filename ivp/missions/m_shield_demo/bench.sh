#!/bin/bash
#------------------------------------------------------------
#   Script: bench.sh
#  Mission: m_shield_demo
#  Purpose: Run the mission headless, untouched, N times, and keep the
#           logs of each run under bench_results/<label>/run<i>.
#
#           Every number this mission has been tuned on so far came from
#           a single run that somebody was watching -- and a run that is
#           being watched is a run where a button might get pressed.
#           These runs take no input after the scenario is laid in, run
#           for a fixed stretch of SIM time, and are killed on a clock,
#           so two of them differ only by the asynchrony of the helms.
#
#           MOOS is not deterministic across runs, so a single run is
#           not evidence. Three is a minimum.
#
#   Usage:  ./bench.sh <label> [trials] [sim_seconds] [warp]
#           ./bench.sh block 3 900 10
#
#           Then:  ./bench.py bench_results/<label>/run*
#------------------------------------------------------------
LABEL=${1:-run}
TRIALS=${2:-3}
SIMSECS=${3:-900}
WARP=${4:-10}

HERE=$(cd $(dirname $0) && pwd)
cd $HERE

# The scenario. Fixed on purpose: the same region, the same start
# positions (vpositions.txt), the same intruder patrol every time.
REGION="pts={-70,-10:210,-10:210,-130:-70,-130}"

kill_all() {
    ps -eo pid,comm | awk '$2 ~ /^(pAntler|MOOSDB|pLogger|pHelmIvP|uSimMarine|pTargetCoo|pRegionDivi|pNodeReport|pShare|pMarinePID|pContactMgr|uFldNodeBro|uFldShoreBr|pHostInfo|uProcessWat|uMemWatch|uLoadWatch|pRealm|uFldNodeCom|uTimerScrip|pTargetPath|iSay|uFldCollisi|pMarineView)/ {print $1}' | xargs -r kill -9 2>/dev/null
    sleep 2
}

REAL=$(echo "$SIMSECS $WARP" | awk '{printf "%d", $1/$2}')
OUT=$HERE/bench_results/$LABEL
mkdir -p $OUT

echo "bench: label=$LABEL trials=$TRIALS sim=${SIMSECS}s warp=$WARP (~${REAL}s real each)"

for i in $(seq 1 $TRIALS); do
    echo "--- trial $i/$TRIALS ---"
    kill_all
    ./clean.sh >/dev/null 2>&1
    ./launch.sh --nogui $WARP >/dev/null 2>&1 &
    sleep 25

    uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
    sleep 3
    uPokeDB targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false >/dev/null 2>&1

    # From here the run is left alone. No pokes, no buttons.
    sleep $REAL

    kill_all
    RUN=$OUT/run$i
    rm -rf $RUN; mkdir -p $RUN
    mv LOG_* XLOG_* $RUN/ 2>/dev/null
    echo "    -> $RUN"
done

echo
echo "done. report with:"
echo "    ./bench.py bench_results/$LABEL/run*"
