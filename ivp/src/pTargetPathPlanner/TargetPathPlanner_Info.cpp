/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetPathPlanner_Info.cpp                           */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TargetPathPlanner_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pTargetPathPlanner lets the operator draw the route a vessel  ");
  blk("  patrols, rather than having it fixed in the launch script.    ");
  blk("                                                                ");
  blk("  It collects mouse-click waypoints (TARGET_PATH_VERTEX) in the ");
  blk("  order they are made -- this is a route, not an area, so no    ");
  blk("  convex hull is taken -- and posts the point list to the       ");
  blk("  vessel's waypoint behavior as TGT_WPT_UPDATE. CLEAR restarts  ");
  blk("  the route and UNDO drops the last click.                      ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTargetPathPlanner file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTargetPathPlanner with the given process name.    ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pTargetPathPlanner.        ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTargetPathPlanner Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTargetPathPlanner                              ");
  blk("{                                                               ");
  blk("  AppTick   = 2                                                 ");
  blk("  CommsTick = 2                                                 ");
  blk("                                                                ");
  blk("  update_var  = TGT_WPT_UPDATE  // behavior's updates variable  ");
  blk("  route_label = target_route    // label of the drawn route     ");
  blk("                                                                ");
  blk("  auto_apply  = true   // apply on each click, not only APPLY   ");
  blk("  close_loop  = true   // draw the closing leg of the circuit   ");
  blk("  speed       = 0      // 0 = leave the behavior's own speed    ");
  blk("                                                                ");
  blk("  // Route in use until the operator draws one.                 ");
  blk("  default_route = 30,-120:150,-120:150,-40:30,-40               ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTargetPathPlanner INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  TARGET_PATH_VERTEX = x=90,y=-120   (operator mouse click)     ");
  blk("  TARGET_PATH_ROUTE  = 30,-120:150,-120:150,-40  (whole route)  ");
  blk("  TARGET_PATH_CLEAR  = true                                     ");
  blk("  TARGET_PATH_UNDO   = true                                     ");
  blk("  TARGET_PATH_APPLY  = true                                     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TGT_WPT_UPDATE = points = pts={x,y:x,y:...}                   ");
  blk("  VIEW_SEGLIST   = (the drawn route)                            ");
  blk("  VIEW_POINT     = (marker on the first waypoint)               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTargetPathPlanner", "gpl");
  exit(0);
}
