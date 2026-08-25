#!/bin/bash
cd $(dirname $0)
POKE="uPokeDB test.moos"
nr() { echo "NAME=$1,TYPE=kayak,TIME=$(date +%s),X=$2,Y=$3,SPD=2,HDG=$4,MODE=DRIVE"; }
hold() { for i in $(seq $1); do $POKE NODE_REPORT="$(nr target $2 $3 $4)" >/dev/null; done; }
$POKE REGION_POLY="pts={-70,-10:210,-10:210,-130:-70,-130}" REGION_CENTER="x=70,y=-70" DEPLOY_ALL=true >/dev/null
$POKE NODE_REPORT="$(nr abe 40 -60 90)" >/dev/null
$POKE NODE_REPORT="$(nr ben 100 -60 90)" >/dev/null

# CASE A: 10 m inside the NORTH edge, pointing SOUTH into the region
$POKE TEST_CASE=A_north_edge_hdg180 >/dev/null
$POKE NODE_REPORT="$(nr target 70 -20 180)" >/dev/null
$POKE TARGET_ALERT_ALL=true >/dev/null
hold 4 70 -20 180
$POKE TARGET_ALERT_ALL=false >/dev/null
hold 6 70 -20 180

# CASE B: southern half, pointing NORTH
$POKE TEST_CASE=B_south_half_hdg000 >/dev/null
$POKE NODE_REPORT="$(nr target 70 -100 0)" >/dev/null
$POKE TARGET_ALERT_ALL=true >/dev/null
hold 5 70 -100 0
$POKE TEST_CASE=done >/dev/null
