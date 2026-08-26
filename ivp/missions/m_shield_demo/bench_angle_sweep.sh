#!/bin/bash
#------------------------------------------------------------
#   Script: bench_angle_sweep.sh
#  Mission: m_shield_demo
#  Purpose: Run systematic benchmark sweep across pincer spread angles
#           (40, 90, 120, 150 deg) with target speed = 2.0 m/s.
#------------------------------------------------------------
ANGLES=${1:-"40 90 120 150"}
TRIALS=${2:-10}
SIMSECS=${3:-900}
WARP=${4:-35}
TGT_SPD="2.0"

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
echo " Starting Pincer Spread Angle Benchmark Sweep"
echo " Angles:   $ANGLES deg"
echo " TgtSpeed: $TGT_SPD m/s (equal to USV stock speed)"
echo " Trials:   $TRIALS per angle"
echo " SimSecs:  ${SIMSECS}s at Warp ${WARP} (~${REAL}s real each)"
echo "============================================================"

for ANG in $ANGLES; do
    LABEL="ang_${ANG}"
    OUT=$HERE/bench_results/$LABEL
    mkdir -p $OUT
    echo
    echo "========================================"
    echo " >>> Testing Pincer Angle: ${ANG} deg ($LABEL)"
    echo "========================================"

    for i in $(seq 1 $TRIALS); do
        echo "--- [Pincer Angle ${ANG} deg] trial $i/$TRIALS ---"
        kill_all
        ./clean.sh >/dev/null 2>&1
        ./launch.sh --nogui --spd=$TGT_SPD $WARP >/dev/null 2>&1 &
        sleep 8

        uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
        sleep 1
        uPokeDB targ_shoreside.moos EVICT_FORMATION="mode=herd,spread=${ANG}" >/dev/null 2>&1
        uPokeDB targ_mothership.moos EVICT_FORMATION="mode=herd,spread=${ANG}" >/dev/null 2>&1
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
echo " Angle Sweep Complete! Generating Statistical Report..."
echo "============================================================"
python3 ./bench_speed_stats.py $HERE/bench_results/ang_*
