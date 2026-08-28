#!/bin/bash
#------------------------------------------------------------
# bench_land_sweep.sh -- repeat the mission and report, per run,
# every way the shoreline work can go wrong.
#
# Not a performance benchmark. It answers one question: over N runs, did any
# hull end up on land, get halted, or need recovering. Those are the three
# failure modes the land work introduced or was meant to remove, and none of
# them shows up reliably in a single run.
#
# Usage: ./bench_land_sweep.sh [runs] [warp] [seconds_per_run]
#------------------------------------------------------------
RUNS=${1:-10}
WARP=${2:-35}
DUR=${3:-60}

ME=$(basename "$0")
MDIR=$(cd "$(dirname "$0")" && pwd)
cd "$MDIR" || exit 1

OUT="$MDIR/bench_results/land_sweep_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.txt"

REGION="pts={-140,40:210,40:210,-160:-140,-160}"

kill_all() {
    pkill -f pAntler   >/dev/null 2>&1
    pkill -f MOOSDB    >/dev/null 2>&1
    sleep 2
}

echo "$ME: $RUNS runs, warp $WARP, ${DUR}s each" | tee "$SUMMARY"
echo "region: $REGION" | tee -a "$SUMMARY"
echo "" | tee -a "$SUMMARY"

kill_all
./launch.sh "$WARP" -j >/dev/null 2>&1

for i in $(seq 1 "$RUNS"); do
    echo "$ME: --- run $i/$RUNS ---"
    rm -rf "$MDIR"/LOG_* "$MDIR"/XLOG_*
    kill_all

    for f in shoreside abe ben target mothership; do
        setsid nohup pAntler "targ_$f.moos" >/dev/null 2>&1 </dev/null &
        sleep 2
    done
    sleep 8

    timeout 20 uPokeDB targ_shoreside.moos MISSION_POLY="$REGION" >/dev/null 2>&1
    sleep 3
    timeout 20 uPokeDB targ_shoreside.moos DEPLOY_ALL=true \
        MOOS_MANUAL_OVERRIDE_ALL=false AVOID_ALL=true RETURN_ALL=false \
        STATION_KEEP_ALL=false SURVEY_ALL=true LOITER_ALL=false \
        TARGET_ALERT_ALL=false >/dev/null 2>&1

    # Refuse to collect data from a mission whose helms are not running. A
    # MALCONFIG helm leaves every vehicle at its start position, and the
    # resulting numbers look perfect -- zero groundings, generous minimum
    # clearance -- because nothing ever moved. Ten such runs were collected
    # before this check existed.
    BAD=""
    for P in 9001 9002 9003; do
        uQueryDB --host=localhost --port=$P \
                 --condition="IVPHELM_STATE != MALCONFIG" --wait=12 \
                 >/dev/null 2>&1 || BAD="$BAD $P"
    done
    if [ -n "$BAD" ]; then
        echo "$ME: ABORT - helm not running on port(s):$BAD (MALCONFIG?)." \
            | tee -a "$SUMMARY"
        echo "$ME: no data collected. Fix the behavior file and re-run." \
            | tee -a "$SUMMARY"
        kill_all
        exit 1
    fi

    START=$(date +%s)
    until [ $(( $(date +%s) - START )) -ge "$DUR" ]; do sleep 5; done

    kill_all

    RD="$OUT/run$i"
    mkdir -p "$RD"
    for d in "$MDIR"/LOG_* "$MDIR"/XLOG_*; do
        [ -d "$d" ] && mv "$d" "$RD/"
    done

    python3 "$MDIR/bench_land_report.py" "$RD" "$i" | tee -a "$SUMMARY"
done

echo "" | tee -a "$SUMMARY"
echo "$ME: results in $OUT" | tee -a "$SUMMARY"
