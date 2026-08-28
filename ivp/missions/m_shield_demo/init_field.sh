#!/bin/bash 
#------------------------------------------------------------
#   Script: init_field.sh
#   Author: M.Benjamin
#   LastEd: May 26 2024
#------------------------------------------------------------
#  Part 1: A convenience function for producing terminal
#          debugging/status output depending on verbosity.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
VEHICLE_AMT="1"
VERBOSE=""
RAND_VPOS="no"

# custom
FAST_START="no"

#------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                      "
	echo "                                               "
	echo "Options:                                       "
	echo "  --amt=N            Num vehicles to launch    "
	echo "  --verbose, -v      Verbose, confirm values   "
	echo "  --rand, -r         Rand vehicle positions    "
	echo "                                               "
	echo "Options (custom):                              "
	echo "  --fast, -f         Start pos head-to-head    "
       exit 0;
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        VEHICLE_AMT="${ARGI#--amt=*}"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RAND_VPOS="yes"
    elif [ "${ARGI}" = "--fast" -o "${ARGI}" = "-f" ]; then
        FAST="yes"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: Source local coordinate grid if it exits
#------------------------------------------------------------

#------------------------------------------------------------
#  Part 5: Set starting positions, speeds, vnames, colors
#------------------------------------------------------------
vecho "Setting starting position, speeds, vnames, colors"

# Start positions, in open water. Clearances below are to the charted
# shoreline in land.txt; check any new value with the land model rather than
# by eye, since the west shore steps east sharply north of y=-40.
#
# The old pair was 0,0 and 180,0. 0,0 sits 20 m off the shoreline, which puts
# a vehicle inside pLandGuard's throttle band (25 m) and inside the land
# avoidance behavior's outer range (30 m) from the moment it launches --
# throttled and being nudged sideways before it has done anything.
echo "x=40,y=-60,heading=180"  >  vpositions.txt    # abe, 76 m clear
echo "x=150,y=-60,heading=180" >> vpositions.txt    # ben, 83 m clear
echo "x=0,y=-75"               >  vloiterpos.txt    # 74 m clear
echo "x=125,y=-50"             >> vloiterpos.txt    # 109 m clear

if [ "${FAST}" != "" ]; then
    # Head-to-head start. Pulled off the west shore as well: the old -10,-80
    # was fine, but 150,-80 / -10,-80 leave the pair converging along y=-80,
    # which is open water for its whole length.
    echo "x=150,y=-80,heading=270" >  vpositions.txt
    echo "x=-10,y=-80,heading=90"  >> vpositions.txt
    echo "x=0,y=-95"               >  vloiterpos.txt
    echo "x=125,y=-65"             >> vloiterpos.txt
fi

pickpos --amt=$VEHICLE_AMT --spd=2:2 > vspeeds.txt 
pickpos --amt=$VEHICLE_AMT --vnames  > vnames.txt
pickpos --amt=$VEHICLE_AMT --colors  > vcolors.txt

#------------------------------------------------------------
#  Part 6: Set other aspects of the field, e.g., obstacles
#------------------------------------------------------------

#------------------------------------------------------------
#  Part 7: If verbose, show file contents
#------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "--------------------------------------"
    echo "VEHICLE_AMT = $VEHICLE_AMT"
    echo "RAND_VPOS   = $RAND_VPOS"
    echo "--------------------------------------(pos/spd)"
    echo "vpositions.txt:"; cat  vpositions.txt
    echo "vspeeds.txt:";    cat  vspeeds.txt
    echo "--------------------------------------(vprops)"
    echo "vnames.txt:";     cat  vnames.txt
    echo "vcolors.txt:";    cat  vcolors.txt
    echo "--------------------------------------(custom)"
    echo "vloiterpos.txt:"; cat  vloiterpos.txt
    echo -n "Hit any key to continue"
    read ANSWER
fi

exit 0
