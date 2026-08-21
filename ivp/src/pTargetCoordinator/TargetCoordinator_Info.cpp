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
  blk("  pTargetCoordinator runs the intruder engagement cycle.        ");
  blk("                                                                ");
  blk("  SEARCHING: the USVs fly their search pattern. An intruder     ");
  blk("  that enters the search area and comes inside the detection    ");
  blk("  range of any USV engages the fleet automatically.             ");
  blk("                                                                ");
  blk("  INTERCEPTING: the USVs are placed on a blocking arc between   ");
  blk("  the intruder and the middle of the search area, so the pair   ");
  blk("  stands between it and the water it wants and its own          ");
  blk("  collision avoidance drives it outward. Stations are posted    ");
  blk("  as absolute-bearing TRAIL_UPDATE_<VNAME> messages.            ");
  blk("                                                                ");
  blk("  Once the intruder has been pushed clear of the search AREA    ");
  blk("  and stayed clear for release_hold seconds, the eviction counts ");
  blk("  as a success: the alert drops, the fleet resumes its search    ");
  blk("  pattern, and after a short re-arm delay detection is live      ");
  blk("  again for the next intrusion.                                 ");
  blk("                                                                ");
  blk("  Losing sensor contact on the intruder does NOT by itself       ");
  blk("  release the engagement -- only crossing the region boundary    ");
  blk("  does. Once engaged, the fleet stays committed until the        ");
  blk("  intruder is confirmed outside the area.                        ");
  blk("                                                                ");
  blk("  The operator's INTERCEPT button starts the SAME engagement    ");
  blk("  early; it runs through the same release test rather than      ");
  blk("  latching the fleet into a mode of its own. Only an engagement ");
  blk("  that actually held the target counts as an eviction.          ");
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
  blk("  pincer_mode = herd             // default, or \"stern\"        ");
  blk("  spread_deg  = 120              // default, total arc spread  ");
  blk("  trail_range = 25               // default, meters            ");
  blk("  trigger_var = TARGET_ALERT_ALL // default                    ");
  blk("  swap_margin_deg = 30           // default, pairing hysteresis");
  blk("  center_deadzone = 20           // default, meters            ");
  blk("  repost_interval = 4            // default, seconds           ");
  blk("  draw_pincer     = true         // default                    ");
  blk("                                                                ");
  blk("  // Automatic engagement cycle                                 ");
  blk("  auto_engage        = true   // detect/evict without operator  ");
  blk("  detect_range       = 0      // 0 = sensor_radius * detect_scale");
  blk("  detect_scale       = 1.0    // multiplier on the sensor range ");
  blk("  sensor_radius      = 15     // fallback until SENSOR_RADIUS   ");
  blk("  release_range      = 0      // 0 = detect*release_scale, with ");
  blk("                              //     a floor of 2*trail_range   ");
  blk("  release_scale      = 1.5                                      ");
  blk("  release_hold       = 8      // secs clear before it counts    ");
  blk("  region_exit_buffer = 15     // metres outside the region      ");
  blk("  reengage_delay     = 10     // deaf period after a success    ");
  blk("  alert_var          = TARGET_ALERT_AUTO                        ");
  blk("  survey_var         = SURVEY_AUTO                              ");
  blk("  draw_detect_rings  = true                                     ");
  blk("  min_detect_range   = 45     // floor under the detection range;  ");
  blk("                              // must exceed the USVs\' COLREGs      ");
  blk("                              // standoff from the intruder, or the ");
  blk("                              // fleet keeps clear of the very      ");
  blk("                              // vessel it is meant to detect.      ");
  blk("  min_standoff       = 30     // floor under trail_range; the      ");
  blk("                              // interception station must sit      ");
  blk("                              // outside the band where the USV\'s   ");
  blk("                              // own COLREGs avoidance of the       ");
  blk("                              // intruder starts pulling (Rule 8d). ");
  blk("  bow_guard_deg      = 20     // half-width of the sector dead      ");
  blk("                              // ahead of the intruder that         ");
  blk("                              // stations are nudged out of.        ");
  blk("                              // 0 disables. The outward half-plane ");
  blk("                              // is open by construction, so no     ");
  blk("                              // larger clearance is needed.        ");
  blk("  operator_release   = true   // an INTERCEPT started by hand     ");
  blk("                              // ends the same way an automatic   ");
  blk("                              // one does: by the target being    ");
  blk("                              // driven off. false makes the      ");
  blk("                              // button a latch only SURVEY frees ");
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
  blk("  REGION_CENTER     = x=..,y=..   (from pRegionDivider)         ");
  blk("  REGION_POLY       = pts={..}    (from pRegionDivider)         ");
  blk("  SENSOR_RADIUS     = 20          (search-plan sensor footprint)");
  blk("  DETECT_RADIUS     = 60          (engagement range; wins over    ");
  blk("                                   sensor_radius when set)        ");
  blk("  DEPLOY_ALL        = true                                      ");
  blk("  TARGET_ALERT_ALL  = true        (operator INTERCEPT/SURVEY)   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TRAIL_UPDATE_<VNAME> = trail_angle=.. # trail_angle_type=..   ");
  blk("                         # trail_range=..                       ");
  blk("  VIEW_SEGLIST         = pincer arms (when draw_pincer)         ");
  blk("  VIEW_POINT           = <vname>_station markers                ");
  blk("  VIEW_SEGLIST         = <vname>_detect rings (detection range) ");
  blk("  TARGET_ALERT_AUTO    = true|false  (auto engage / stand down) ");
  blk("  SURVEY_AUTO          = false|true  (the matching survey flag) ");
  blk("  SHIELD_STATE         = intercepting | searching               ");
  blk("  SHIELD_DETECTIONS    = <count>                                ");
  blk("  SHIELD_EVICTIONS     = <count of successful evictions>        ");
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
