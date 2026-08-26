#!/bin/bash
#------------------------------------------------------------
#   Script: bench_cant_sweep.sh
#  Mission: m_shield_demo
#  Purpose: Run systematic benchmark sweep across inward cant angles
#           (0, 10, 20, 30, 45 deg) with larger sample size.
#------------------------------------------------------------
CANTS=${1:-"0 10 20 30 45"}
TRIALS=${2:-5}
SIMSECS=${3:-600}
WARP=${4:-35}
TGT_SPD=${5:-"1.4"}

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
echo " Starting Inward Cant Angle Benchmark Sweep"
echo " Cant Angles: $CANTS deg"
echo " TgtSpeed:    $TGT_SPD m/s"
echo " Trials:      $TRIALS per angle"
echo " SimSecs:     ${SIMSECS}s at Warp ${WARP} (~${REAL}s real each)"
echo "============================================================"

for CANT in $CANTS; do
    LABEL="cant_${CANT}"
    OUT=$HERE/bench_results/$LABEL
    mkdir -p $OUT
    echo
    echo "========================================"
    echo " >>> Testing Cant Angle: ${CANT} deg ($LABEL)"
    echo "========================================"

    for i in $(seq 1 $TRIALS); do
        echo "--- [Cant Angle ${CANT} deg] trial $i/$TRIALS ---"
        kill_all
        ./clean.sh >/dev/null 2>&1
        ./launch.sh --nogui --spd=$TGT_SPD $WARP >/dev/null 2>&1 &
        sleep 8

        uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
        sleep 1
        uPokeDB targ_shoreside.moos EVICT_CANT="$CANT" >/dev/null 2>&1
        uPokeDB targ_mothership.moos EVICT_CANT="$CANT" >/dev/null 2>&1
        uPokeDB targ_shoreside.moos EVICT_FORMATION="mode=herd,cant=${CANT}" >/dev/null 2>&1
        uPokeDB targ_mothership.moos EVICT_FORMATION="mode=herd,cant=${CANT}" >/dev/null 2>&1
        sleep 1
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
echo " Cant Sweep Complete! Generating Statistical Report..."
echo "============================================================"
python3 ./bench_speed_stats.py $HERE/bench_results/cant_*

