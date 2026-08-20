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
MAX_SPD="1.2"
TGT_HOME="90,-200"
# Patrol loop that crosses both survey lanes
TGT_PATROL="30,-120:150,-120:150,-40:30,-40"

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
    echo "TGT_PATROL =    [${TGT_PATROL}]   "
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
       TGT_PATROL=$TGT_PATROL

nsplug meta_target.bhv targ_$VNAME.bhv $NSFLAGS \
       VNAME=$VNAME                 TGT_SPD=$TGT_SPD   \
       TGT_PATROL=$TGT_PATROL       TGT_HOME=$TGT_HOME \
       MMOD=$MMOD

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
