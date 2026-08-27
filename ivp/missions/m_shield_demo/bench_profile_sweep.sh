#!/bin/bash
#------------------------------------------------------------
#   Script: bench_profile_sweep.sh
#  Mission: m_shield_demo
#  Purpose: Run systematic benchmark sweep across target behavior profiles
#           (evader, merchant, sprinter, zigzag, lumbering, loiterer)
#           with 5 trials per profile, accelerated simulation.
#------------------------------------------------------------
PROFILES=${1:-"evader merchant sprinter zigzag lumbering loiterer"}
TRIALS=${2:-5}
SIMSECS=${3:-550}
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
echo " Starting Target Scenario Profile Benchmark Sweep"
echo " Profiles:  $PROFILES"
echo " TgtSpeed:  $TGT_SPD m/s"
echo " Trials:    $TRIALS per profile"
echo " SimSecs:   ${SIMSECS}s at Warp ${WARP} (~${REAL}s real each)"
echo "============================================================"

for PROF in $PROFILES; do
    LABEL="prof_${PROF}"
    OUT=$HERE/bench_results/$LABEL
    mkdir -p $OUT
    echo
    echo "========================================"
    echo " >>> Testing Target Profile: ${PROF} ($LABEL)"
    echo "========================================"

    ACTUAL_PROF="$PROF"
    ACTUAL_SPD="$TGT_SPD"
    if [ "$PROF" = "default_baseline" -o "$PROF" = "default" ]; then
        ACTUAL_PROF="evader"
        ACTUAL_SPD="0.8"
    fi

    for i in $(seq 1 $TRIALS); do
        echo "--- [Profile ${PROF}] trial $i/$TRIALS ---"
        kill_all
        ./clean.sh >/dev/null 2>&1
        ./launch.sh --nogui --spd=$ACTUAL_SPD --tgt_profile=$ACTUAL_PROF $WARP >/dev/null 2>&1 &
        sleep 8

        uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
        sleep 1
        uPokeDB targ_shoreside.moos TGT_PROFILE="$ACTUAL_PROF" >/dev/null 2>&1
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
echo " Profile Sweep Complete! Generating Escape Path Analysis..."
echo "============================================================"
python3 ./bench_escape_path_analyzer.py $HERE/bench_results/prof_*

