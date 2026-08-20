/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: RegionDivider_Info.cpp                               */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "RegionDivider_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pRegionDivider owns the mission search region and plans the   ");
  blk("  fleet's sweep of it.                                          ");
  blk("                                                                ");
  blk("  The sweep axis is taken from the region's own shape (the long ");
  blk("  axis of its minimum-area bounding rectangle), so lanes run as ");
  blk("  long as the water allows and hard 180-degree turns are as few ");
  blk("  as possible. The region is then cut into one band per vehicle ");
  blk("  ACROSS that axis, keeping every lane full length.             ");
  blk("                                                                ");
  blk("  The pattern laid into each band is operator-selectable at run ");
  blk("  time via SEARCH_PATTERN: lawnmower, skip, spiral, perimeter.  ");
  blk("                                                                ");
  blk("  Lane spacing and lane-end trimming both come from the sensor  ");
  blk("  range (SENSOR_RADIUS), so no distance is spent driving over   ");
  blk("  water the sensor already sees.                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pRegionDivider file.moos [OPTIONS]                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pRegionDivider with the given process name rather  ");
  blk("      than pRegionDivider.                                      ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pRegionDivider.            ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pRegionDivider Example MOOS Configuration                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pRegionDivider                                  ");
  blk("{                                                               ");
  blk("  AppTick   = 2                                                 ");
  blk("  CommsTick = 2                                                 ");
  blk("                                                                ");
  blk("  vnames  = abe:ben          // colon-sep list, one band each  ");
  blk("                                                                ");
  blk("  pattern         = lawnmower // lawnmower|skip|spiral|perimeter");
  blk("  sweep_align     = auto      // auto|east-west|north-south     ");
  blk("                                                                ");
  blk("  sensor_radius   = 15        // metres of visual/sensor range  ");
  blk("  auto_lane_width = true      // lane = 2*radius*(1-overlap)    ");
  blk("  lane_overlap    = 0.10      // seam between adjacent lanes    ");
  blk("  endpoint_inset  = 0.70      // trim lane ends by this*radius  ");
  blk("  lane_width      = 20        // only if auto_lane_width=false  ");
  blk("                                                                ");
  blk("  coverage_shading = true     // shade swept cells on the chart ");
  blk("  cell_size        = 10                                         ");
  blk("                                                                ");
  blk("  // Used only if DEPLOY_ALL fires before any MISSION_POLY      ");
  blk("  // has ever been received from the operator.                 ");
  blk("  default_region = pts={-70,-10:210,-10:210,-130:-70,-130}      ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pRegionDivider INTERFACE                                        ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  MISSION_POLY   = pts={-70,-10:210,-10:210,-130:-70,-130}      ");
  blk("  REGION_VERTEX  = x=12,y=-40   (operator mouse click)          ");
  blk("  REGION_CLEAR   = true                                         ");
  blk("  SEARCH_PATTERN = lawnmower | skip | spiral | perimeter        ");
  blk("  SENSOR_RADIUS  = 20           (metres of sensor/visual range) ");
  blk("  DEPLOY_ALL     = true                                         ");
  blk("  NODE_REPORT / NODE_REPORT_LOCAL                               ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  WPT_UPDATE_<VNAME> = points = pts={x,y:x,y:...},label=...     ");
  blk("  REGION_CENTER      = x=..,y=..  (centroid of the region)      ");
  blk("  VIEW_POLYGON       = (mission region outline, for the GUI)    ");
  blk("  VIEW_SEGLIST       = (<vname>_survey_path, persistent copy of ");
  blk("                        each vehicle's planned sweep)           ");
  blk("  VIEW_GRID          = (coverage grid over the region)          ");
  blk("  VIEW_GRID_DELTA    = (swept-cell updates)                     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pRegionDivider", "gpl");
  exit(0);
}
