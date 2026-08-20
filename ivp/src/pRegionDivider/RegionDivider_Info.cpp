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
  blk("  pRegionDivider slices an incoming mission-region polygon      ");
  blk("  (e.g. MISSION_POLY) into N west->east vertical strips, one    ");
  blk("  per vehicle in the configured vnames list, and posts a        ");
  blk("  lawnmower-format WPT_UPDATE_<VNAME> for each strip.           ");
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
  blk("  vnames  = abe:ben          // colon-sep list, west->east order");
  blk("  lane_width = 20            // default                        ");
  blk("  rows       = east-west     // default                        ");
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
  blk("  MISSION_POLY = pts={-70,-10:210,-10:210,-130:-70,-130}        ");
  blk("  DEPLOY_ALL   = true                                           ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  WPT_UPDATE_<VNAME> = polygon = format=lawnmower, label=...,   ");
  blk("  REGION_CENTER      = x=..,y=..  (centroid of the region)      ");
  blk("                       x=.., y=.., height=.., width=..,         ");
  blk("                       lane_width=.., rows=.., startx=..,       ");
  blk("                       starty=..                                ");
  blk("  VIEW_POLYGON       = (mission region outline, for the GUI)    ");
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
