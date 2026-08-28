/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: LandGuard_Info.cpp                                   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "LandGuard_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pLandGuard keeps one vehicle off the charted shoreline, and   ");
  blk("  gets it back off if it ever ends up on it.                    ");
  blk("                                                                ");
  blk("  BHV_AvoidObstacleV24 covers the ordinary case, but has        ");
  blk("  nothing to say once the vehicle is inside an obstacle: its    ");
  blk("  objective function answers \"which way is least likely to hit  ");
  blk("  this\", which has no answer after the fact. The behavior goes  ");
  blk("  quiet and the running waypoint behavior takes over, steering  ");
  blk("  by a plan that knows nothing about land.                      ");
  blk("                                                                ");
  blk("  Three outputs, for three jobs:                                ");
  blk("                                                                ");
  blk("    LAND_CLEARANCE  metres to the nearest shoreline, always.    ");
  blk("    LAND_SLOW       true while close, for a speed behavior to   ");
  blk("                    throttle back on. Running out of turning    ");
  blk("                    room IS the unavoidable case, and turning   ");
  blk("                    room is bought with speed.                  ");
  blk("    LAND_BREACH     true once on or nearly on land, with        ");
  blk("                    LAND_ESCAPE_UPDATE giving a waypoint        ");
  blk("                    behavior somewhere to go.                   ");
  blk("                                                                ");
  blk("  The escape point is the last position at which the vehicle    ");
  blk("  had real open water around it -- not the nearest water, which ");
  blk("  can lie across a spit and be reachable only by crossing more  ");
  blk("  land. The vehicle came from the breadcrumb, so it can return  ");
  blk("  to it: for recovery, reachability beats distance.             ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pLandGuard file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pLandGuard with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pLandGuard.                    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pLandGuard Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pLandGuard                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // Shoreline, as written by gen_land.py. A missing file       ");
  blk("  // leaves the guard inert rather than failing the launch.     ");
  blk("  land_file  = land.txt                                         ");
  blk("                                                                ");
  blk("  // Throttle band. Hysteresis matters: without it a lane run   ");
  blk("  // parallel to the shore at the trigger distance chatters.    ");
  blk("  slow_dist        = 25                                         ");
  blk("  slow_hysteresis  = 8                                          ");
  blk("                                                                ");
  blk("  // Recovery trips before contact, not after: once the hull is ");
  blk("  // inside a tile the avoidance behavior has already gone      ");
  blk("  // quiet, which leaves recovery nothing to work with.         ");
  blk("  breach_dist      = 3                                          ");
  blk("                                                                ");
  blk("  // Breadcrumbs. crumb_dist must exceed breach_dist, or the    ");
  blk("  // escape point would not itself clear the breach on arrival. ");
  blk("  crumb_dist       = 20                                         ");
  blk("  crumb_spacing    = 10                                         ");
  blk("  arrive_dist      = 8                                          ");
  blk("                                                                ");
  blk("  post_visuals     = true                                       ");
  blk("  slow_var         = LAND_SLOW                                  ");
  blk("  breach_var       = LAND_BREACH                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pLandGuard INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y                                                  ");
  blk("  APPCAST_REQ                                                   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  LAND_CLEARANCE      metres to the nearest shoreline           ");
  blk("  LAND_SLOW           true/false, on change                     ");
  blk("  LAND_BREACH         true/false, on change                     ");
  blk("  LAND_ESCAPE_PT      x=<x>,y=<y>                               ");
  blk("  LAND_ESCAPE_UPDATE  points = <x>,<y>   (for BHV_Waypoint)     ");
  blk("  VIEW_POINT, VIEW_SEGLIST                                      ");
  blk("  APPCAST                                                       ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pLandGuard", "gpl");
  exit(0);
}
