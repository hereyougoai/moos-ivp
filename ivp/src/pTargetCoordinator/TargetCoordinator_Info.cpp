/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetCoordinator_Info.cpp                           */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TargetCoordinator_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pTargetCoordinator tracks the intruder's NODE_REPORT and,     ");
  blk("  while intercept is active, spaces the configured USVs into    ");
  blk("  flanking pincer trail angles around the target's stern,       ");
  blk("  evenly spread and continuously re-posted as the target's      ");
  blk("  heading changes.                                              ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTargetCoordinator file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTargetCoordinator with the given process name     ");
  blk("      rather than pTargetCoordinator.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pTargetCoordinator.        ");
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
  blu("pTargetCoordinator Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTargetCoordinator                              ");
  blk("{                                                               ");
  blk("  AppTick   = 2                                                 ");
  blk("  CommsTick = 2                                                 ");
  blk("                                                                ");
  blk("  vnames      = abe:ben          // colon-sep list              ");
  blk("  target_name = target           // default                    ");
  blk("  spread_deg  = 90               // default, total fan spread  ");
  blk("  trail_range = 30               // default, meters            ");
  blk("  trigger_var = TARGET_ALERT_ALL // default                    ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTargetCoordinator INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT       = NAME=target,X=..,Y=..,HDG=..,SPD=..       ");
  blk("  TARGET_ALERT_ALL  = true                                      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TRAIL_UPDATE_<VNAME> = trail_angle=.. # trail_range=..        ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTargetCoordinator", "gpl");
  exit(0);
}
