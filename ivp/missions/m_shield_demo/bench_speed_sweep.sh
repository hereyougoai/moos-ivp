#!/bin/bash
#------------------------------------------------------------
#   Script: bench_speed_sweep.sh
#  Mission: m_shield_demo
#  Purpose: Run systematic benchmark sweep across target speeds
#           in headless background mode, storing logs per speed.
#------------------------------------------------------------
SPEEDS=${1:-"0.4 0.8 1.2 1.6 2.0"}
TRIALS=${2:-3}
SIMSECS=${3:-900}
WARP=${4:-20}

HERE=$(cd $(dirname $0) && pwd)
cd $HERE
export PATH="/home/yoei/moos-ivp/bin:$PATH"

REGION="pts={-70,-10:210,-10:210,-130:-70,-130}"

kill_all() {
    ps -eo pid,comm | awk '$2 ~ /^(pAntler|MOOSDB|pLogger|pHelmIvP|uSimMarine|pTargetCoo|pRegionDivi|pNodeReport|pShare|pMarinePID|pContactMgr|uFldNodeBro|uFldShoreBr|pHostInfo|uProcessWat|uMemWatch|uLoadWatch|pRealm|uFldNodeCom|uTimerScrip|pTargetPath|iSay|uFldCollisi|pMarineView)/ {print $1}' | xargs -r kill -9 2>/dev/null
    sleep 2
}

REAL=$(echo "$SIMSECS $WARP" | awk '{printf "%d", $1/$2}')
echo "============================================================"
echo " Starting Target Speed Benchmark Sweep"
echo " Speeds:  $SPEEDS"
echo " Trials:  $TRIALS per speed"
echo " SimSecs: ${SIMSECS}s at Warp ${WARP} (~${REAL}s real each)"
echo "============================================================"

for SPD in $SPEEDS; do
    LABEL="spd_${SPD}"
    OUT=$HERE/bench_results/$LABEL
    mkdir -p $OUT
    echo
    echo "========================================"
    echo " >>> Testing Target Speed: ${SPD} m/s ($LABEL)"
    echo "========================================"

    for i in $(seq 1 $TRIALS); do
        echo "--- [Speed ${SPD} m/s] trial $i/$TRIALS ---"
        kill_all
        ./clean.sh >/dev/null 2>&1
        ./launch.sh --nogui --spd=$SPD $WARP >/dev/null 2>&1 &
        sleep 12

        uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
        sleep 3
        uPokeDB targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false >/dev/null 2>&1

        # Run unattended for duration of trial
        sleep $REAL

        kill_all
        RUN=$OUT/run$i
        rm -rf $RUN; mkdir -p $RUN
        mv LOG_* XLOG_* $RUN/ 2>/dev/null
        echo "    -> Saved to $RUN"
    done
done

echo
echo "============================================================"
echo " Sweep Complete! Generating Statistical Report..."
echo "============================================================"
python3 ./bench_speed_stats.py $HERE/bench_results/spd_*
