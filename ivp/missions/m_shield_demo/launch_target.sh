#!/bin/bash
#------------------------------------------------------------
#   Script: launch_target.sh
#  Mission: m_shield_demo
#  Purpose: Launch the simulated intruder ("target") vessel that
#           abe/ben search for and trail.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT
trap on_exit SIGTERM

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
CMD_ARGS=""
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"

IP_ADDR="localhost"
MOOS_PORT="9003"
PSHARE_PORT="9203"
SHORE_IP="localhost"
SHORE_PSHARE="9200"
MMOD=""

VNAME="target"
COLOR="orange"
START_POS="x=90,y=-140,heading=0"
# Deliberately slower than the USVs (stock 2.0, max 3.0) so the
# interceptors can close from any part of the field and then hold
# their pincer stations through the target's evasive turns.
TGT_SPD="0.8"
# Ceiling of the helm's speed domain AND of uSimMarine, NOT the patrol
# speed -- the vessel still cruises at TGT_SPD.
#
# uSimMarine's max_speed is a hard clamp applied to the final reported
# NAV_SPEED (see USM_MOOSApp.cpp: "if new_speed > m_max_speed, clamp").
# It is NOT the gain in the thrust-to-speed mapping -- that is a fixed
# thrust_factor (20), set independently of this value. So raising this
# ceiling does not change how the vessel behaves at any speed below it;
# measured on the patrol loop at full settle, commanding 0.78 m/s
# converges to ~0.70-0.77 m/s whether the ceiling is 1.2 or 3.0 -- the
# difference is noise from where in the patrol corner the sample landed,
# not from the ceiling.
#
# What the ceiling DOES change is the top speed reachable during a hard
# push -- evasion under BHV_AvoidCollision, or the "sprinter" profile's
# dash -- and the logged runs show the target riding this ceiling during
# evasion at the old value of 1.2. Set here at 3.0 so that regime is not
# artificially capped and so the shoreside Action/Target Speed menu's
# 1.4 / 2.0 / 3.0 entries (and the sprinter profile) all have room to
# actually go faster. Lower it with --max_spd for a run that needs to
# reproduce numbers taken under the old 1.2 ceiling.
MAX_SPD="3.0"
TGT_HOME="90,-200"
# Patrol loop that crosses both survey lanes
TGT_PATROL="30,-120:150,-120:150,-40:30,-40"
# Which kind of vessel the intruder behaves as. See the PROFILE CATALOG
# in meta_target.bhv. The operator can switch this live from the
# shoreside Action/Target Profile menu; this is only the startup value.
#   evader | merchant | sprinter | zigzag | lumbering | loiterer |
#   harasser | dead_ship
TGT_PROFILE="evader"

#------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                     "
	echo "                                               "
	echo "Options:                                       "
	echo "  --help, -h         Show this help message    "
	echo "  --just_make, -j    Only create targ files    "
	echo "  --verbose, -v      Verbose, confirm launch   "
	echo "  --auto, -a         Auto-launched by a script "
	echo "                                               "
	echo "  --ip=<localhost>       pHostInfo IP address  "
	echo "  --mport=<9003>         This community MOOSDB "
	echo "  --pshare=<9203>        This community pShare "
	echo "  --shore_pshare=<9200>  Shoreside pShare port "
	echo "  --vname=<target>       Target vessel name    "
	echo "  --color=<orange>       Target vessel color   "
	echo "  --start_pos=<..>       Target start position "
	echo "  --spd=<0.8>            Target patrol speed   "
	echo "  --max_spd=<1.2>        Helm/sim speed ceiling."
	echo "                         Raise for the sprinter"
	echo "                         profile and the GUI's "
	echo "                         faster speed choices; "
	echo "                         changes the tuned      "
	echo "                         dynamics, see script.  "
	echo "  --profile=<evader>     Target behavior profile:"
	echo "                         evader, merchant,     "
	echo "                         sprinter, zigzag,     "
	echo "                         lumbering, loiterer,  "
	echo "                         harasser, dead_ship   "
	echo "                         (operator can switch  "
	echo "                          it live in the GUI)  "
	echo "  --patrol=<x,y:x,y:..>  Starting patrol route "
	echo "                         (operator can redraw  "
	echo "                          it live in the GUI)  "
	echo "  --mmod=<mod>           Mission variation/mod "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:12}" = "--start_pos=" ]; then
        START_POS="${ARGI#--start_pos=*}"
    elif [ "${ARGI:0:6}" = "--spd=" ]; then
        TGT_SPD="${ARGI#--spd=*}"
    elif [ "${ARGI:0:10}" = "--max_spd=" ]; then
        MAX_SPD="${ARGI#--max_spd=*}"
    elif [ "${ARGI:0:10}" = "--profile=" ]; then
        TGT_PROFILE="${ARGI#--profile=*}"
    elif [ "${ARGI:0:9}" = "--patrol=" ]; then
        TGT_PATROL="${ARGI#--patrol=*}"
    elif [ "${ARGI:0:7}" = "--mmod=" ]; then
        MMOD="${ARGI#--mmod=*}"
    else
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then
    echo "=================================="
    echo "  launch_target.sh SUMMARY        "
    echo "=================================="
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "VNAME =         [${VNAME}]        "
    echo "START_POS =     [${START_POS}]    "
    echo "TGT_SPD =       [${TGT_SPD}]      "
    echo "MAX_SPD =       [${MAX_SPD}]      "
    echo "TGT_PATROL =    [${TGT_PATROL}]   "
    echo "TGT_PROFILE =   [${TGT_PROFILE}]  "
    echo -n "Hit any key to continue launch "
    read ANSWER
fi

#------------------------------------------------------------
#  Part 6: Create the target mission files
#------------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_target.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR             MOOS_PORT=$MOOS_PORT   \
       PSHARE_PORT=$PSHARE_PORT     SHORE_IP=$SHORE_IP     \
       SHORE_PSHARE=$SHORE_PSHARE   VNAME=$VNAME           \
       COLOR=$COLOR                 START_POS=$START_POS   \
       MAX_SPD=$MAX_SPD             MMOD=$MMOD             \
       TGT_PATROL=$TGT_PATROL                              \
       LAND_ALERT_RANGE=${LAND_ALERT_RANGE:-35} \
       MAX_TIME_STEP=${MAX_TIME_STEP:-1.0}

nsplug meta_target.bhv targ_$VNAME.bhv $NSFLAGS \
       VNAME=$VNAME                 TGT_SPD=$TGT_SPD   \
       TGT_PATROL=$TGT_PATROL       TGT_HOME=$TGT_HOME \
       TGT_PROFILE=$TGT_PROFILE     MAX_SPD=$MAX_SPD   \
       MMOD=$MMOD                                      \
       LAND_ALLSTOP=${LAND_ALLSTOP:-false} \
       LAND_SLOW_SPD=${LAND_SLOW_SPD:-0.8} \
       LAND_ESCAPE_SPD=${LAND_ESCAPE_SPD:-1.2}

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 7: Launch the target MOOS community
#------------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
echo "Done Launching $VNAME MOOS Community"

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targ_$VNAME.moos
trap "" SIGINT
kill -- -$$
