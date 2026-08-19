#!/bin/bash 
#------------------------------------------------------------
#   Script: launch_mothership.sh    
#  Mission: m_shield_demo
#   Author: M.Benjamin (adapted for shield demo)
#   LastEd: Aug 2026
#------------------------------------------------------------
#  Part 1: Set convenience functions for producing terminal
#          debugging output, and catching SIGINT (ctrl-c).
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
MOOS_PORT="9005"
PSHARE_PORT="9205"
SHORE_PSHARE="9200"
MMOD=""

VNAMES="abe:ben"

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
	echo "                                               "
        echo "  --auto, -a                                   "
        echo "    Auto-launched by a script.                 "
        echo "    Will not launch uMAC as the final step.    "
	echo "                                               "
	echo "  --ip=<localhost>                             "
	echo "    Force pHostInfo to use this IP Address     "
	echo "  --mport=<9005>                               "
	echo "    Port number of this community's MOOSDB     "
	echo "  --pshare=<9205>                              "
	echo "    Port number of this community's pShare     "
	echo "  --shore_pshare=<9200>                        "
	echo "    Port number of shoreside's pShare          "
        echo "  --mmod=<mod>                                 "
        echo "    Identify a mission variation/mod           "
	echo "                                               "
        echo "  --vnames=<vnames>                            "
        echo "    Colon-separate list of all vehicle names   "
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
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:7}" = "--mmod=" ]; then
        MMOD="${ARGI#--mmod=*}"

    elif [ "${ARGI:0:9}" = "--vnames=" ]; then
        VNAMES="${ARGI#--vnames=*}"
    else
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: If not auto_launched, try to get IP address
#------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "no" -a "${IP_ADDR}" = "localhost" ]; then
    MAYBE_IP_ADDR=`ipaddrs.sh --blunt`
    if [ $? = 0 ]; then
	IP_ADDR=$MAYBE_IP_ADDR
    fi
fi

#------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "=================================="
    echo "  launch_mothership.sh SUMMARY    "
    echo "=================================="
    echo "$ME                               "
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "MMOD =          [${MMOD}]         "
    echo "----------------------------------"
    echo "VNAMES =        [${VNAMES}]       "
    echo "----------------------------------"
    echo -n "Hit any key to continue launch "
    read ANSWER
fi

#------------------------------------------------------------
#  Part 6: Create the mothership mission file
#------------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_mothership.moos targ_mothership.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR          MOOS_PORT=$MOOS_PORT       \
       PSHARE_PORT=$PSHARE_PORT  SHORE_PSHARE=$SHORE_PSHARE \
       MMOD=$MMOD                VNAMES=$VNAMES

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 7: Launch the Mothership MOOS community
#------------------------------------------------------------
echo "Launching Mothership MOOS Community. WARP="$TIME_WARP
pAntler targ_mothership.moos >& /dev/null &
echo "Done Launching Mothership Community"

#------------------------------------------------------------
#  Part 8: If launched from script, we're done, exit now
#------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#------------------------------------------------------------
# Part 9: Launch uMAC until the mission is quit
#------------------------------------------------------------
uMAC targ_mothership.moos
trap "" SIGINT
kill -- -$$
