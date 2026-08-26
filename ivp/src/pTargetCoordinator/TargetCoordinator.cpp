/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetCoordinator.cpp                                */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cmath>
#include "TargetCoordinator.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "NodeRecordUtils.h"
#include "XYSegList.h"
#include "XYPoint.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

TargetCoordinator::TargetCoordinator()
{
  m_target_name = "target";
  m_spread_deg  = 90;
  m_trail_range = 25;
  m_trigger_var = "TARGET_ALERT_ALL";

  // The operator's Eviction Formation menu on the shoreside. Same
  // three formations pincer_mode selects at startup, switchable while
  // the engagement is running.
  m_formation_var = "EVICT_FORMATION";

  m_herd_mode       = true;
  m_swap_margin_deg = 30;
  m_center_deadzone = 20;
  m_repost_interval = 4;
  m_draw_pincer     = true;

  // Order stations as an offset from the intruder's heading rather
  // than as a compass bearing. See assignAndPost() for why.
  m_station_relative = true;

  m_intercept_active   = false;
  m_assignments_posted = 0;
  m_geodesy_ok         = false;

  m_center_x     = 0;
  m_center_y     = 0;
  m_center_known = false;

  m_last_post_utc = 0;
  m_basis         = "none";
  m_swaps         = 0;

  // Automatic engagement. detect_range=0 means "take it from the
  // operator's sensor range", which is the number the search plan is
  // already built around, so the range the USVs are said to see is the
  // same range they actually react at.
  m_auto_engage        = true;
  m_detect_range       = 0;
  m_detect_scale       = 1.0;
  m_sensor_radius      = 15;
  m_release_hold       = 8;
  m_region_exit_buffer = 15;
  m_reengage_delay     = 10;
  m_alert_var          = "TARGET_ALERT_AUTO";
  m_survey_var         = "SURVEY_AUTO";
  m_draw_detect_rings  = true;

  // An operator-started intercept ends the same way an automatic one
  // does: by the target being driven off. Set false to make INTERCEPT
  // a latch that only the SURVEY button releases.
  m_operator_release   = true;

  // CANT ANGLE. 0 = the stock geometry: on station BHV_Trail steers the
  // contact's own course, so each USV lies exactly PARALLEL to the
  // intruder. Above zero, each bow is turned that many degrees IN
  // toward the intruder's centreline. See cantAngle() and slotCant().
  m_cant_deg           = 0;
  m_cant_dead_deg      = 5;

  // Half-width of the sector dead ahead of the intruder that stations
  // are nudged out of. Small on purpose -- see bowGuard().
  m_bow_guard_deg      = 20;

  // Floor under trail_range. The interception station has to sit
  // outside the band where the USV's own COLREGs avoidance of the
  // intruder starts pulling, or the two fight and the pass ends up
  // closer than either intended. See trailRange().
  m_min_standoff       = 30;

  // Eviction direction. heading_bias is the price, in metres of
  // detour, the fleet will pay to avoid asking the intruder for a
  // turn: a big value means "push it out the way it is already going
  // even if that is the long way round", a small one means "out the
  // nearest side". At 0.35 a 90-degree turn is worth 31 m and a full
  // reversal 63 m, which on a field 120 m across means the geometry
  // normally wins. See exitCost() and updateExitDirection().
  m_heading_bias       = 0.35;
  m_exit_margin        = 15;

  // Once an engagement starts, the exit is LATCHED: a rival bearing
  // has to stay better by exit_margin for exit_commit seconds before
  // the wall is re-anchored. See updateExitDirection().
  m_exit_commit        = 5;

  // And the flanks are frozen with it. See assignAndPost().
  m_swap_lock          = true;

  // Blocking stations -- the mode that actually evicts. See
  // assignAndPostBlock() for why the trail arc cannot.
  m_block_mode         = true;
  m_block_lead         = 50;
  m_block_offset       = 18;
  m_block_span         = 34;
  m_block_lead2        = 0.6;
  m_block_slack        = 22;
  m_block_min_lead     = 25;
  m_block_interval     = 5;
  m_block_min_offset   = 14;
  m_block_dead_deg     = 15;
  m_block_abort        = 25;
  m_block_rearm        = 10;
  m_giveways           = 0;

  // Adaptive offset. block_offset is now the ceiling a fresh
  // engagement starts at; these govern how it is felt down from there.
  // See updateAdaptiveOffset().
  m_block_reaction_wait = 6;
  m_block_reaction_deg  = 8;
  m_block_shrink_step   = 3;

  m_block_side_commit   = 4;

  // Transit feasibility. See blockTransitHold() and feasibleLead().
  m_block_commit_transit = true;
  m_block_transit_frac   = 1.0;
  m_block_transit_cap    = 40;
  m_block_feasible       = true;
  m_block_usv_speed      = 2.0;

  m_block_posted       = false;
  m_block_side         = 1;
  m_block_side_challenge_since = -1;
  m_block_reissues     = 0;

  m_block_offset_cur   = -1;
  m_block_reaction_hdg = 0;
  m_block_reaction_utc = 0;
  m_block_reacted      = false;

  m_exit_dir           = 0;
  m_exit_valid         = false;
  m_exit_cost          = 0;
  m_exit_since         = 0;
  m_exit_challenge_since = -1;
  m_swaps_declined     = 0;

  m_last_suspect_report = "none yet";
  m_suspect_reports     = 0;

  m_engaged       = false;
  m_engage_source = "auto";
  m_had_contact   = false;
  m_deployed      = false;
  m_clear_since   = -1;
  m_rearm_utc     = 0;
  m_detections    = 0;
  m_evictions     = 0;
  m_last_range    = 0;
  m_range_known   = false;
  m_region_known  = false;
  m_rings_drawn   = false;
  m_last_ring_utc = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TargetCoordinator::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key  = msg.GetKey();
    string sval = msg.GetString();

    // pMarineViewer posts anything that parses as a number as a DOUBLE,
    // not a string -- so SENSOR_RADIUS off the Sensor Range menu arrives
    // with an empty GetString(). Without this the detection range would
    // silently ignore the operator's choice.
    if(!msg.IsString())
      sval = doubleToStringX(msg.GetDouble(), 4);

    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if(key == "REGION_CENTER")
      handleMailRegionCenter(sval);
    else if(key == "REGION_POLY")
      handleMailRegionPoly(sval);
    else if(key == "SUSPECT_REPORT")
      handleMailSuspectReport(sval);
    else if(key == "SENSOR_RADIUS") {
      double newval;
      if(setPosDoubleOnString(newval, stripBlankEnds(sval)) && (newval >= 1))
	m_sensor_radius = newval;
      else
	reportRunWarning("Bad SENSOR_RADIUS: " + sval);
    }
    else if(key == "EVICT_BUFFER") {
      // How far outside the region the intruder has to be pushed
      // before the run counts as an eviction (Action/Eviction Distance
      // on the shoreside). Same DOUBLE-vs-string caveat as
      // SENSOR_RADIUS above -- handled by the IsString() test.
      double newval;
      if(setNonNegDoubleOnString(newval, stripBlankEnds(sval))) {
	if(newval != m_region_exit_buffer) {
	  m_region_exit_buffer = newval;
	  // The target may already be sitting further out than the new
	  // buffer, so the clear timer has to be re-judged from now
	  // rather than credited with time held against the old one.
	  m_clear_since = -1;
	  reportEvent("Eviction distance set to " +
		      doubleToStringX(m_region_exit_buffer,1) +
		      " m outside the region.");
	}
      }
      else
	reportRunWarning("Bad EVICT_BUFFER: " + sval);
    }
    else if(key == "EVICT_CANT") {
      // How far each USV's bow is canted IN off the parallel while it
      // holds its station (Action/Cant Angle on the shoreside). Same
      // DOUBLE-vs-string caveat as SENSOR_RADIUS above.
      double newval;
      if(setNonNegDoubleOnString(newval, stripBlankEnds(sval))) {
	if(newval > 90)
	  newval = 90;
	if(newval != m_cant_deg) {
	  m_cant_deg = newval;
	  // The stations themselves have not moved, so the 1-degree
	  // deadband in assignAndPost() would swallow the change and
	  // the new cant would not reach the vehicles until something
	  // else happened to move the arc. Forget what was posted so
	  // the next pass re-sends it.
	  m_posted_angle.clear();
	  m_posted_cant.clear();
	  reportEvent("Cant angle set to " + doubleToStringX(m_cant_deg,1) +
		      " deg inward off the parallel.");
	}
      }
      else
	reportRunWarning("Bad EVICT_CANT: " + sval);
    }
    else if(key == "DEPLOY_ALL") {
      bool deployed = (tolower(sval) == "true");
      // Coming out of a deploy, start the engagement cycle from
      // scratch: no standing alert, detection armed immediately.
      if(deployed && !m_deployed) {
	m_engaged     = false;
	m_clear_since = -1;
	m_rearm_utc   = 0;
	m_contact_held.clear();
	m_exit_valid  = false;
      }
      if(!deployed && m_deployed)
	eraseDetectRings();
      m_deployed = deployed;
    }
    else if((key == m_formation_var) && (m_formation_var != ""))
      handleMailFormation(sval);
    else if(key == m_trigger_var) {
      // The operator's INTERCEPT / SURVEY buttons. INTERCEPT starts the
      // SAME engagement the detector starts -- it is a way to trigger
      // the cycle early, not a separate mode.
      //
      // It used to set a flag that switched the whole release test off,
      // on the reasoning that an operator-requested alert should not be
      // second-guessed. That was wrong: it meant an intercept started
      // from the button never ended, so the pair kept herding a target
      // that had long since been driven out of the area, and the whole
      // point of the mission -- drive it off, go back to searching --
      // could only ever be seen when nobody touched the button.
      bool active = (tolower(sval) == "true");

      if(active && !m_engaged) {
	m_engaged       = true;
	m_engage_source = "operator";
	m_clear_since   = -1;
	m_had_contact   = false;
	// Forget the previous pairing so the USVs are matched to
	// whichever flank they happen to be on right now.
	m_assignment.clear();
	m_posted_angle.clear();
	m_posted_cant.clear();
	m_exit_valid           = false;
	m_exit_challenge_since = -1;
	m_block_side_challenge_since = -1;
	resetAdaptiveOffset();
	postAlert(true);
      }
      else if(!active && m_engaged) {
	// Stood down by hand -- not an eviction, so nothing is counted.
	// Re-arm detection, but not instantly: the target is usually
	// still right next to the USVs at that moment, and re-engaging
	// in the same second would make the button look broken.
	m_engaged     = false;
	m_clear_since = -1;
	m_rearm_utc   = MOOSTime() + m_reengage_delay;
	postAlert(false);
      }
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TargetCoordinator::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool TargetCoordinator::Iterate()
{
  AppCastingMOOSApp::Iterate();

  updateExitDirection();
  updateEngagement();

  if(m_engaged != m_intercept_active) {
    if(!m_engaged) {
      clearPincerVisuals();
      standDownBlock();
    }
    m_intercept_active = m_engaged;
  }

  if(m_intercept_active) {
    if(m_block_mode)
      assignAndPostBlock();
    else
      assignAndPost();
  }

  if(m_draw_detect_rings && m_deployed)
    postDetectRings();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool TargetCoordinator::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // NODE_REPORTs from vehicles running pNodeReporter with
  // coord_policy_global carry LAT/LON but no X/Y, so we need the
  // datum to work out which flank each vehicle is on.
  double lat_origin, lon_origin;
  bool ok1 = m_MissionReader.GetValue("LatOrigin", lat_origin);
  bool ok2 = m_MissionReader.GetValue("LongOrigin", lon_origin);
  if(!ok1 || !ok2)
    reportConfigWarning("LatOrigin/LongOrigin not set in *.moos file.");
  else if(!m_geodesy.Initialise(lat_origin, lon_origin))
    reportConfigWarning("Geodesy init failed.");
  else
    m_geodesy_ok = true;

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {

    list<string>::reverse_iterator p;
    for(p=sParams.rbegin(); p!=sParams.rend(); p++) {
      string orig  = *p;
      string line  = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      bool handled = false;
      if(param == "vnames") {
	m_vnames = parseString(value, ':');
	handled = true;
      }
      else if(param == "target_name") {
	m_target_name = tolower(value);
	handled = true;
      }
      else if(param == "spread_deg")
	handled = setPosDoubleOnString(m_spread_deg, value);
      else if(param == "trail_range")
	handled = setPosDoubleOnString(m_trail_range, value);
      else if(param == "trigger_var") {
	m_trigger_var = value;
	handled = true;
      }
      else if(param == "formation_var") {
	m_formation_var = value;
	handled = true;
      }
      else if(param == "pincer_mode") {
	string mode = tolower(value);
	if(mode == "block") {
	  m_block_mode = true;
	  m_herd_mode  = true;
	  handled = true;
	}
	else if(mode == "herd") {
	  m_block_mode = false;
	  m_herd_mode  = true;
	  handled = true;
	}
	else if(mode == "stern") {
	  m_block_mode = false;
	  m_herd_mode  = false;
	  handled = true;
	}
      }
      else if(param == "block_lead")
	handled = setPosDoubleOnString(m_block_lead, value);
      else if(param == "block_offset")
	handled = setPosDoubleOnString(m_block_offset, value);
      else if(param == "block_span")
	handled = setNonNegDoubleOnString(m_block_span, value);
      else if(param == "block_lead2")
	handled = setPosDoubleOnString(m_block_lead2, value);
      else if(param == "block_slack")
	handled = setPosDoubleOnString(m_block_slack, value);
      else if(param == "block_min_lead")
	handled = setNonNegDoubleOnString(m_block_min_lead, value);
      else if(param == "block_interval")
	handled = setNonNegDoubleOnString(m_block_interval, value);
      else if(param == "block_min_offset")
	handled = setNonNegDoubleOnString(m_block_min_offset, value);
      else if(param == "block_dead_deg")
	handled = setNonNegDoubleOnString(m_block_dead_deg, value);
      else if(param == "block_abort")
	handled = setNonNegDoubleOnString(m_block_abort, value);
      else if(param == "block_rearm")
	handled = setNonNegDoubleOnString(m_block_rearm, value);
      else if(param == "block_reaction_wait")
	handled = setPosDoubleOnString(m_block_reaction_wait, value);
      else if(param == "block_reaction_deg")
	handled = setPosDoubleOnString(m_block_reaction_deg, value);
      else if(param == "block_shrink_step")
	handled = setPosDoubleOnString(m_block_shrink_step, value);
      else if(param == "block_side_commit")
	handled = setNonNegDoubleOnString(m_block_side_commit, value);
      else if(param == "block_commit_transit")
	handled = setBooleanOnString(m_block_commit_transit, value);
      else if(param == "block_transit_frac")
	handled = setPosDoubleOnString(m_block_transit_frac, value);
      else if(param == "block_transit_cap")
	handled = setPosDoubleOnString(m_block_transit_cap, value);
      else if(param == "block_feasible")
	handled = setBooleanOnString(m_block_feasible, value);
      else if(param == "block_usv_speed")
	handled = setPosDoubleOnString(m_block_usv_speed, value);
      else if(param == "cant_deg")
	handled = setNonNegDoubleOnString(m_cant_deg, value);
      else if(param == "cant_dead_deg")
	handled = setNonNegDoubleOnString(m_cant_dead_deg, value);
      else if(param == "swap_margin_deg")
	handled = setNonNegDoubleOnString(m_swap_margin_deg, value);
      else if(param == "center_deadzone")
	handled = setNonNegDoubleOnString(m_center_deadzone, value);
      else if(param == "repost_interval")
	handled = setPosDoubleOnString(m_repost_interval, value);
      else if(param == "draw_pincer")
	handled = setBooleanOnString(m_draw_pincer, value);
      else if(param == "station_frame") {
	string sval = tolower(value);
	if(sval == "relative")
	  m_station_relative = true;
	else if(sval == "absolute")
	  m_station_relative = false;
	else
	  handled = false;
      }
      else if(param == "auto_engage")
	handled = setBooleanOnString(m_auto_engage, value);
      else if(param == "detect_range")
	handled = setNonNegDoubleOnString(m_detect_range, value);
      else if(param == "detect_scale")
	handled = setPosDoubleOnString(m_detect_scale, value);
      else if(param == "sensor_radius")
	handled = setPosDoubleOnString(m_sensor_radius, value);
      else if(param == "release_hold")
	handled = setNonNegDoubleOnString(m_release_hold, value);
      else if(param == "region_exit_buffer")
	handled = setNonNegDoubleOnString(m_region_exit_buffer, value);
      else if(param == "reengage_delay")
	handled = setNonNegDoubleOnString(m_reengage_delay, value);
      else if(param == "alert_var")
	handled = setNonWhiteVarOnString(m_alert_var, value);
      else if(param == "survey_var")
	handled = setNonWhiteVarOnString(m_survey_var, value);
      else if(param == "draw_detect_rings")
	handled = setBooleanOnString(m_draw_detect_rings, value);
      else if(param == "operator_release")
	handled = setBooleanOnString(m_operator_release, value);
      else if((param == "bow_guard_deg") || (param == "escape_lane_deg")) {
	handled = setNonNegDoubleOnString(m_bow_guard_deg, value);
	if(handled && (m_bow_guard_deg > 90))
	  handled = false;
      }
      else if(param == "min_standoff")
	handled = setNonNegDoubleOnString(m_min_standoff, value);
      else if(param == "heading_bias")
	handled = setNonNegDoubleOnString(m_heading_bias, value);
      else if(param == "exit_margin")
	handled = setNonNegDoubleOnString(m_exit_margin, value);
      else if(param == "exit_commit")
	handled = setNonNegDoubleOnString(m_exit_commit, value);
      else if(param == "swap_lock")
	handled = setBooleanOnString(m_swap_lock, value);

      if(!handled)
	reportUnhandledConfigWarning(orig);
    }
  }

  if(m_vnames.size() == 0)
    reportConfigWarning("No vnames configured - nothing to assign stations to.");

  registerVariables();
  return(true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void TargetCoordinator::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("REGION_CENTER", 0);
  Register("REGION_POLY", 0);
  Register("SENSOR_RADIUS", 0);
  Register("EVICT_BUFFER", 0);
  Register("EVICT_CANT", 0);
  Register("SUSPECT_REPORT", 0);
  Register("DEPLOY_ALL", 0);
  Register(m_trigger_var, 0);
  if(m_formation_var != "")
    Register(m_formation_var, 0);
}

//------------------------------------------------------------
// Procedure: formationName()

string TargetCoordinator::formationName() const
{
  if(m_block_mode)
    return("block");
  if(m_herd_mode)
    return("herd");
  return("stern");
}

//------------------------------------------------------------
// Procedure: setFormation()
//   Purpose: Switch the eviction formation while the mission is
//            running, leaving no half of the old one behind.
//
//            The two formations drive DIFFERENT behaviors on the
//            vehicles -- herd/stern place BHV_Trail through
//            TRAIL_UPDATE, block places BHV_StationKeep through
//            BLOCK_UPDATE, and the two are made mutually exclusive by
//            BLOCK_ACTIVE. So a switch is not just a flag: whichever
//            side is being left has to be released, or a vehicle can
//            end up holding a station nobody is updating any more,
//            with a stale marker still drawn over it.

bool TargetCoordinator::setFormation(bool block_mode, bool herd_mode,
				     string why)
{
  if((block_mode == m_block_mode) && (herd_mode == m_herd_mode))
    return(false);

  string was = formationName();

  if(m_block_mode && !block_mode)
    standDownBlock();       // releases BLOCK_ACTIVE and its visuals
  if(!m_block_mode && block_mode)
    clearPincerVisuals();   // the trail arc's arms and station markers

  m_block_mode = block_mode;
  m_herd_mode  = herd_mode;

  // The arc (or the pair of blocking points) is about to be re-derived
  // from a different geometry, so the standing pairing and the "already
  // posted this angle" memory are both meaningless now. Dropping them
  // re-pairs each USV to the flank it is actually on and forces a fresh
  // order out immediately, rather than at the next heartbeat.
  m_assignment.clear();
  m_posted_angle.clear();
  m_posted_cant.clear();
  m_block_side_challenge_since = -1;
  resetAdaptiveOffset();

  reportEvent("Eviction formation: " + was + " -> " + formationName() +
	      (why == "" ? "" : " (" + why + ")"));
  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailFormation()
//   Purpose: The operator's Eviction Formation menu.
//
//   Accepts: a bare formation name --
//              block | herd | stern   (aliases: wall, pincer, tail)
//            or a comma-separated list that can also carry the
//            formation's shape, so one menu entry can say both which
//            formation and how wide:
//              mode=herd, spread_deg=150, trail_range=30, cant=25

void TargetCoordinator::handleMailFormation(string sval)
{
  string str = stripBlankEnds(sval);
  if(str == "")
    return;

  string mode;
  double spread = 0, range = 0, cant = 0;
  bool   got_spread = false, got_range = false, got_cant = false;
  bool   ok = true;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(stripBlankEnds(biteStringX(svector[i], '=')));
    string value = stripBlankEnds(svector[i]);

    if(value == "") {          // a bare word: the formation name
      mode = param;
    }
    else if((param == "mode") || (param == "formation"))
      mode = tolower(value);
    else if((param == "spread_deg") || (param == "spread"))
      got_spread = ok = setPosDoubleOnString(spread, value);
    else if(param == "trail_range")
      got_range = ok = setPosDoubleOnString(range, value);
    else if((param == "cant_deg") || (param == "cant"))
      got_cant = ok = setNonNegDoubleOnString(cant, value);
    else
      ok = false;

    if(!ok) {
      reportRunWarning("Bad " + m_formation_var + ": " + sval);
      return;
    }
  }

  bool block_mode = m_block_mode;
  bool herd_mode  = m_herd_mode;

  if(mode != "") {
    if((mode == "block") || (mode == "wall")) {
      block_mode = true;
      herd_mode  = true;
    }
    else if((mode == "herd") || (mode == "pincer")) {
      block_mode = false;
      herd_mode  = true;
    }
    else if((mode == "stern") || (mode == "tail") || (mode == "trail")) {
      block_mode = false;
      herd_mode  = false;
    }
    else {
      reportRunWarning("Unknown " + m_formation_var + " mode: " + mode);
      return;
    }
  }

  // Shape first, so the formation the switch turns on is built with the
  // width the same menu entry asked for rather than the previous one.
  if(got_spread && (spread != m_spread_deg)) {
    m_spread_deg = spread;
    m_posted_angle.clear();
    m_posted_cant.clear();
    reportEvent("Arc spread set to " + doubleToStringX(m_spread_deg,1) + " deg.");
  }
  if(got_range && (range != m_trail_range)) {
    m_trail_range = range;
    m_posted_angle.clear();
    m_posted_cant.clear();
    reportEvent("Station range set to " + doubleToStringX(trailRange(),1) + " m.");
  }
  if(got_cant) {
    if(cant > 90)
      cant = 90;
    if(cant != m_cant_deg) {
      m_cant_deg = cant;
      m_posted_angle.clear();
      m_posted_cant.clear();
      reportEvent("Cant angle set to " + doubleToStringX(m_cant_deg,1) +
		  " deg inward off the parallel.");
    }
  }

  setFormation(block_mode, herd_mode, "operator");
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()

void TargetCoordinator::handleMailNodeReport(string str)
{
  NodeRecord record = string2NodeRecord(str);
  if(!record.valid())
    return;

  string vname = tolower(record.getName());
  if(vname == "")
    return;

  m_records[vname] = record;
}

//------------------------------------------------------------
// Procedure: handleMailRegionCenter()
//   Format: "x=val,y=val", posted by pRegionDivider.

void TargetCoordinator::handleMailRegionCenter(string str)
{
  double cx = 0;
  double cy = 0;
  bool x_set = false;
  bool y_set = false;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string part  = stripBlankEnds(svector[i]);
    string left  = tolower(biteStringX(part, '='));
    string right = part;
    if((left == "x") && isNumber(right)) {
      cx = atof(right.c_str());
      x_set = true;
    }
    else if((left == "y") && isNumber(right)) {
      cy = atof(right.c_str());
      y_set = true;
    }
  }

  if(!x_set || !y_set) {
    reportRunWarning("Unparsable REGION_CENTER: " + str);
    return;
  }

  m_center_x     = cx;
  m_center_y     = cy;
  m_center_known = true;
}

//------------------------------------------------------------
// Procedure: recordPosition()
//   Purpose: Local-grid position of a node report. Prefers the X/Y
//            fields, and falls back to converting LAT/LON when the
//            reporter is configured for global coordinates only.

bool TargetCoordinator::recordPosition(const NodeRecord& record,
				       double& x, double& y) const
{
  if(record.isSetX() && record.isSetY()) {
    x = record.getX();
    y = record.getY();
    return(true);
  }

  if(!m_geodesy_ok || !record.isSetLatitude() || !record.isSetLongitude())
    return(false);

  double nav_x, nav_y;
  CMOOSGeodesy& geo = const_cast<CMOOSGeodesy&>(m_geodesy);
  geo.LatLong2LocalGrid(record.getLat(), record.getLon(), nav_y, nav_x);
  x = nav_x;
  y = nav_y;
  return(true);
}

//------------------------------------------------------------
// Procedure: targetPosition()

bool TargetCoordinator::targetPosition(double& x, double& y) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(false);
  return(recordPosition(t->second, x, y));
}

//------------------------------------------------------------
// Procedure: targetHeading()

bool TargetCoordinator::targetHeading(double& heading) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(false);
  heading = angle360(t->second.getHeading());
  return(true);
}

//------------------------------------------------------------
// Procedure: pincerBaseAngle()
//   Purpose: The bearing the blocking arc is centred on.
//
//            In herd mode that is the bearing from the target back
//            toward the middle of the mission region: park the USVs
//            symmetrically about it and they sit between the intruder
//            and the water it wants to hold, so its own collision
//            avoidance (pwt 250, above its patrol) has nowhere to run
//            but outward. That is the difference between a pincer and
//            a tail chase -- an absolute arc that stays put as the
//            target manoeuvres, rather than two stations glued to its
//            wake that just follow it wherever it goes.
//
//            Two fallbacks to dead astern: no region centre known, and
//            the target sitting almost on top of the centre, where the
//            "inboard" direction is meaningless. There we push it along
//            the course it is already on.

bool TargetCoordinator::pincerBaseAngle(double& base_angle, string& basis) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(false);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(false);

  // Preferred: put the wall on the reciprocal of the chosen exit, so
  // the USVs sit on the side the intruder is being pushed away FROM and
  // the open water is the way out. When the exit is aligned with the
  // intruder's own head -- which the heading_bias makes the usual case
  // -- that puts them on its quarters, shepherding it along the course
  // it is already steering rather than blocking its bow.
  if(m_herd_mode && m_exit_valid) {
    base_angle = angle360(m_exit_dir + 180);
    basis = "opposite the exit bearing " + doubleToStringX(m_exit_dir,0);
    return(true);
  }

  // No region polygon: fall back to pushing away from the centroid.
  if(m_herd_mode && m_center_known) {
    double dist = distPointToPoint(tx, ty, m_center_x, m_center_y);
    if(dist > m_center_deadzone) {
      base_angle = relAng(tx, ty, m_center_x, m_center_y);
      basis = "inboard (toward region center)";
      return(true);
    }
  }

  base_angle = angle360(t->second.getHeading() + 180);
  basis = "stern (target heading + 180)";
  return(true);
}

//------------------------------------------------------------
// Procedure: computeSlots()
//   Purpose: The blocking arc: n stations spread evenly over
//            m_spread_deg, centred on base_angle. Absolute compass
//            bearings from the target; assignAndPost() converts them
//            to the frame BHV_Trail is actually ordered in.

vector<double> TargetCoordinator::computeSlots(double base_angle) const
{
  unsigned int n = m_vnames.size();


  vector<double> slots;
  for(unsigned int i=0; i<n; i++) {
    double offset = 0;
    if(n > 1)
      offset = -(m_spread_deg / 2.0) + (i * (m_spread_deg / (double)(n-1)));
    slots.push_back(bowGuard(angle360(base_angle + offset)));
  }
  return(slots);
}

//------------------------------------------------------------
// Procedure: bowGuard()
//   Purpose: Keep a single station out of a narrow sector dead ahead
//            of the intruder, nudging it to the nearer edge.
//
//            This replaces an escape-lane rule that rotated the WHOLE
//            blocking arc off a 70-degree sector centred on the
//            intruder's heading plus 90. With spread_deg=120 that
//            demanded 60+35 = 95 degrees of clearance between the arc
//            centre and the lane centre, so any time the intruder was
//            tracking roughly tangentially -- which is most of the time
//            once it is being worked on -- the arc was rotated by up to
//            95 degrees. A blocking wall rotated 95 degrees is no
//            longer between the intruder and the water it wants: the
//            herding direction inverted and the intruder walked back
//            toward the region centre unopposed.
//
//            The sea room Rule 8(f) asks for is already there by
//            construction and does not need to be bought with a
//            rotation: stations occupy only spread_deg of arc centred
//            on the inboard bearing, so the entire outward half-plane
//            -- the direction we actually want the intruder to leave
//            by -- is open water. All that is left to avoid is parking
//            a USV bow-on in front of it, and that is a small nudge,
//            bounded by bow_guard_deg, not a wholesale rotation.

double TargetCoordinator::bowGuard(double slot_angle) const
{
  if(m_bow_guard_deg <= 0)
    return(slot_angle);

  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(slot_angle);

  // Station bearings are measured FROM the intruder, so a station dead
  // ahead of it sits on its own heading.
  double bow  = angle360(t->second.getHeading());
  double off  = angle180(slot_angle - bow);

  if(fabs(off) >= m_bow_guard_deg)
    return(slot_angle);

  return(angle360(bow + ((off >= 0) ? m_bow_guard_deg : -m_bow_guard_deg)));
}

//------------------------------------------------------------
// Procedure: trailRange()
//   Purpose: The station range actually ordered, never closer than
//            min_standoff.
//
//            COLREG Rule 8(d) requires action that results in "passing
//            at a safe distance", and the interception is the one
//            manoeuvre in this mission that deliberately closes on
//            another vessel, so it is the one that has to be held to
//            that. Measured at the old 25 m station, the pair got to
//            9.9 m of the intruder and the avoidance behavior reported
//            standon:inextremis -- Rule 17(b) territory, where the
//            give-way vessel's action alone can no longer prevent a
//            collision. That is not a safe distance by any reading.

double TargetCoordinator::trailRange() const
{
  if(m_trail_range < m_min_standoff)
    return(m_min_standoff);
  return(m_trail_range);
}

//------------------------------------------------------------
// Procedure: cantAngle()
//   Purpose: The configured cant angle, clamped to what the word can
//            still mean.
//
//   WHAT THE CANT ANGLE IS. Zero is "exactly parallel to the intruder":
//   on station BHV_Trail steers ownship at the contact's own course, so
//   with no cant the two USVs run alongside the intruder on the same
//   heading it has. The cant angle turns each bow off that baseline, IN
//   toward the intruder's centreline -- the USV stops running abreast
//   and starts cutting in.
//
//   It moves the BOW ONLY. The station itself is still trail_angle off
//   the intruder at trailRange(), and the speed is still matched, so
//   the arc keeps its shape: the cant changes what each USV is pointing
//   at, not where it is standing.
//
//   Past 90 the bow is no longer canting in toward the track, it is
//   turning back across it, so that is the ceiling.

double TargetCoordinator::cantAngle() const
{
  if(m_cant_deg <= 0)
    return(0);
  if(m_cant_deg > 90)
    return(90);
  return(m_cant_deg);
}

//------------------------------------------------------------
// Procedure: slotCant()
//   Purpose: Turn the cant MAGNITUDE into the signed bow offset for one
//            station, so that positive always means "canted inward".
//
//   Which way "inward" is depends on which side of the intruder's track
//   the station sits on, and that is what this decides. rel is the
//   station's bearing off the intruder's own head:
//
//     rel > 0  station on the intruder's STARBOARD side, so its
//              centreline lies to port of the USV -- cant to PORT.
//     rel < 0  station to PORT, centreline to starboard -- cant to
//              STARBOARD.
//
//   BHV_Trail's trail_cant is signed the other way round (+ = starboard
//   of the contact's course), hence the negation.
//
//   A station sitting ON the centreline -- dead ahead of the intruder or
//   dead astern of it -- has no inward side at all: both directions turn
//   the bow equally far off the track. Rather than let a fraction of a
//   degree of noise decide it and flip the bow from one beam to the
//   other, cant_dead_deg either side of the line means no cant.

double TargetCoordinator::slotCant(double slot_angle) const
{
  double cant = cantAngle();
  if(cant <= 0)
    return(0);

  double hdg = 0;
  if(!targetHeading(hdg))
    return(0);

  double rel = angle180(slot_angle - hdg);

  // Dead ahead, or dead astern: no inward side to cant toward.
  if(fabs(rel) <= m_cant_dead_deg)
    return(0);
  if(fabs(rel) >= (180 - m_cant_dead_deg))
    return(0);

  return((rel > 0) ? -cant : cant);
}

//------------------------------------------------------------
// Procedure: assignmentCost()
//   Purpose: Total angle each USV would have to travel around the
//            target to reach the slot it has been given.

double TargetCoordinator::assignmentCost(const vector<unsigned int>& order,
					 const vector<double>& slots) const
{
  double tx, ty;
  if(!targetPosition(tx, ty))
    return(0);

  double cost = 0;
  for(unsigned int slot=0; slot<order.size(); slot++) {
    if(order[slot] >= m_vnames.size())
      return(0);
    string vname = tolower(m_vnames[order[slot]]);
    map<string, NodeRecord>::const_iterator v = m_records.find(vname);
    if(v == m_records.end())
      return(0);

    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      return(0);

    double bearing = relAng(tx, ty, vx, vy);
    cost += fabs(angle180(bearing - slots[slot]));
  }
  return(cost);
}

//------------------------------------------------------------
// Procedure: bestAssignment()
//   Purpose: Pair USVs to slots by the flank each is already on, so
//            nobody is told to cross the target's track. Both sides
//            are sorted by the same angular sweep and paired in
//            order, which is the cheapest non-crossing pairing.
//
//   Returns: assignment[i] = index into m_vnames for slot i.

vector<unsigned int> TargetCoordinator::bestAssignment(const vector<double>& slots) const
{
  unsigned int n = m_vnames.size();

  vector<unsigned int> ident;
  for(unsigned int i=0; i<n; i++)
    ident.push_back(i);

  double tx, ty;
  if(!targetPosition(tx, ty) || (slots.size() != n))
    return(ident);

  // Sweep key: signed offset of each USV's current bearing from the
  // centre of the arc, in [-180,180]. One flank sorts before the other.
  double center = slots[0];
  if(n > 1)
    center = angle360(slots[0] + (angle180(slots[n-1] - slots[0]) / 2.0));

  vector<double> keys;
  for(unsigned int i=0; i<n; i++) {
    map<string, NodeRecord>::const_iterator v = m_records.find(tolower(m_vnames[i]));
    if(v == m_records.end())
      return(ident);

    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      return(ident);

    keys.push_back(angle180(relAng(tx, ty, vx, vy) - center));
  }

  // n is the fleet size (2 today), so an insertion sort on the
  // key/index pair is plenty and keeps the pairing easy to follow.
  vector<unsigned int> order = ident;
  for(unsigned int i=1; i<n; i++) {
    unsigned int idx = order[i];
    double       key = keys[idx];
    int j = (int)i - 1;
    while((j >= 0) && (keys[order[j]] > key)) {
      order[j+1] = order[j];
      j--;
    }
    order[j+1] = idx;
  }

  return(order);
}

//------------------------------------------------------------
// Procedure: assignAndPost()
//   Purpose: Put the USVs on the blocking arc and post a
//            TRAIL_UPDATE_<VNAME> for each.
//
//   Note on stickiness: the pairing used to be recomputed from
//         scratch at every iteration. Whenever the two USVs sat near
//         the crossover of the sweep -- which is exactly where they
//         end up once the pincer has closed -- millimetre-level noise
//         flipped the ordering back and forth at 2 Hz, each vehicle
//         was handed the other's station, and both spun on the spot.
//         Now a pairing is kept until swapping is better by a clear
//         margin.

void TargetCoordinator::assignAndPost()
{
  unsigned int n = m_vnames.size();
  if(n == 0)
    return;

  double base_angle = 0;
  string basis;
  if(!pincerBaseAngle(base_angle, basis))
    return;
  m_basis = basis;

  vector<double> slots = computeSlots(base_angle);

  vector<unsigned int> candidate = bestAssignment(slots);
  if(m_assignment.size() != n)
    m_assignment = candidate;
  else if(candidate != m_assignment) {
    // Flanks are frozen for the run. The pairing is taken once, at the
    // moment the engagement starts, from the flank each USV is already
    // on -- which is the only moment the question has a clean answer.
    // After that a swap means both USVs leave station and cross the
    // intruder's track to change places, and for the seconds that
    // takes there is no wall at all. The old cost margin was sized for
    // sensor noise; it cannot tell that from a genuine re-anchoring of
    // the arc, and a large swing of the arc (the boundary case) clears
    // it easily. So: not while engaged.
    if(m_swap_lock && m_engaged)
      m_swaps_declined++;
    else {
      double cost_now = assignmentCost(m_assignment, slots);
      double cost_new = assignmentCost(candidate, slots);
      if(cost_now - cost_new > m_swap_margin_deg) {
	m_assignment = candidate;
	m_swaps++;
      }
    }
  }

  if(m_posted_angle.size() != n)
    m_posted_angle.assign(n, -999);
  if(m_posted_cant.size() != n)
    m_posted_cant.assign(n, -999);

  // Re-post when a station has actually moved, and on a slow heartbeat
  // so a vehicle that (re)joined late still picks up its assignment.
  bool refresh_due = ((MOOSTime() - m_last_post_utc) > m_repost_interval);

  bool posted_any = false;
  for(unsigned int slot=0; slot<n; slot++) {
    unsigned int vidx = m_assignment[slot];
    if(vidx >= n)
      continue;

    // WHICH FRAME THE STATION IS ORDERED IN.
    //
    // The arc itself is computed in absolute bearings, and ordering it
    // that way is exactly what it says: the trail point sits at a fixed
    // compass bearing from the intruder. The problem is that the arc is
    // not actually static -- bowGuard() and the stern fallback both
    // track the intruder's heading -- so under an absolute order every
    // degree of the intruder's turn has to travel back out as a fresh
    // TRAIL_UPDATE. Those arrive quantised (the 1-degree deadband
    // below), at this app's tick, and one pShare hop late, so on the
    // chart the two station points do not sweep with the turn: they sit
    // still and then jump. That is the stutter.
    //
    // Ordered RELATIVE to the intruder's heading, the heading term is
    // applied by BHV_Trail itself, locally, every helm iteration, off
    // its own extrapolated contact record -- so the point sweeps
    // smoothly through the turn and only the slow part of the geometry
    // (which way we are herding it) comes over the wire.
    double post_angle = slots[slot];
    if(m_station_relative) {
      double hdg = 0;
      if(!targetHeading(hdg))
	continue;
      post_angle = angle180(slots[slot] - hdg);
    }

    // The cant is signed off which side of the intruder's TRACK the
    // station is on, so under an absolute station frame it can flip
    // while the station bearing itself has not moved at all -- the
    // intruder turned instead. Testing it here as well as the bearing
    // is what makes sure that flip is actually sent.
    double post_cant = slotCant(slots[slot]);

    if(!refresh_due &&
       (fabs(angle180(post_angle - m_posted_angle[vidx])) < 1.0) &&
       (fabs(post_cant - m_posted_cant[vidx]) < 1.0))
      continue;

    string spec = "trail_angle=" + doubleToStringX(post_angle,1);
    spec += " # trail_angle_type=";
    spec += (m_station_relative ? "relative" : "absolute");
    spec += " # trail_range=" + doubleToStringX(trailRange(),1);

    // The cant angle rides along in the same update. Like the station
    // bearing it is expressed RELATIVE to the intruder's heading --
    // BHV_Trail adds the heading term itself, locally, every helm
    // iteration -- so a turning intruder does not have to be chased
    // with a stream of fresh absolute headings over the wire. Only the
    // side the USV is standing on, which changes slowly, travels here.
    spec += " # trail_cant=" + doubleToStringX(post_cant,1);

    Notify("TRAIL_UPDATE_" + toupper(m_vnames[vidx]), spec);
    m_posted_angle[vidx] = post_angle;
    m_posted_cant[vidx]  = post_cant;
    posted_any = true;
  }

  if(posted_any) {
    m_last_post_utc = MOOSTime();
    m_assignments_posted++;
  }

  if(m_draw_pincer)
    postPincerVisuals(slots, m_assignment);
}

//------------------------------------------------------------
// Procedure: postPincerVisuals()
//   Purpose: Draw the closing arms so the pincer is legible on the
//            chart: a line from each station through the target, plus
//            a marker on each station point.

void TargetCoordinator::postPincerVisuals(const vector<double>& slots,
					  const vector<unsigned int>& order)
{
  double tx, ty;
  if(!targetPosition(tx, ty))
    return;

  XYSegList arms;
  for(unsigned int slot=0; slot<slots.size(); slot++) {
    double sx, sy;
    projectPoint(slots[slot], trailRange(), tx, ty, sx, sy);

    if(slot > 0)
      arms.add_vertex(tx, ty);
    arms.add_vertex(sx, sy);

    if(order.size() > slot) {
      unsigned int vidx = order[slot];
      if(vidx < m_vnames.size()) {
	XYPoint station(sx, sy);
	station.set_label(m_vnames[vidx] + "_station");
	station.set_vertex_size(7);
	station.set_vertex_color("red");
	station.set_label_color("red");
	station.set_active(true);
	Notify("VIEW_POINT", station.get_spec());
      }
    }
  }

  if(arms.size() < 2)
    return;

  arms.set_label("pincer");
  arms.set_edge_color("red");
  arms.set_vertex_color("red");
  arms.set_edge_size(2);
  arms.set_vertex_size(3);
  arms.set_active(true);
  Notify("VIEW_SEGLIST", arms.get_spec());
}

//------------------------------------------------------------
// Procedure: clearPincerVisuals()

void TargetCoordinator::clearPincerVisuals()
{
  XYSegList arms;
  arms.set_label("pincer");
  arms.set_active(false);
  Notify("VIEW_SEGLIST", arms.get_spec());

  for(unsigned int i=0; i<m_vnames.size(); i++) {
    XYPoint station;
    station.set_label(m_vnames[i] + "_station");
    station.set_active(false);
    Notify("VIEW_POINT", station.get_spec());
  }
}

//------------------------------------------------------------
// Procedure: blockOffset()
//   Purpose: The CEILING on the lateral offset -- the widest, most
//            conservative station block_offset ever stands at, floored
//            by block_min_offset. A fresh engagement starts here.
//
//            This used to be the number stood at for the whole run,
//            picked to sit inside a KNOWN reaction band read off the
//            intruder's own avoidance config (min/max_util_cpa_dist in
//            its .bhv file). That is fine for this simulated target,
//            whose numbers we happen to have, but it is not a distance
//            derived from anything the fleet can actually observe: a
//            contact of unknown type -- different avoidance model,
//            human on the helm, none at all -- has no reason to share
//            those thresholds, and a hard-coded 18 m is then either too
//            far to register or already inside its true safe-pass
//            distance. See updateAdaptiveOffset() for the number
//            actually used, currentBlockOffset(), which starts here and
//            is felt inward by observed reaction rather than assumed.

double TargetCoordinator::blockOffset() const
{
  if(m_block_offset < m_block_min_offset)
    return(m_block_min_offset);
  return(m_block_offset);
}

//------------------------------------------------------------
// Procedure: currentBlockOffset()
//   Purpose: The offset actually in force right now -- the adaptive
//            value maintained by updateAdaptiveOffset(), clamped to
//            [block_min_offset, blockOffset()]. Falls back to the
//            ceiling before the first engagement has initialized it.

double TargetCoordinator::currentBlockOffset() const
{
  if(m_block_offset_cur < 0)
    return(blockOffset());
  if(m_block_offset_cur < m_block_min_offset)
    return(m_block_min_offset);
  if(m_block_offset_cur > blockOffset())
    return(blockOffset());
  return(m_block_offset_cur);
}

//------------------------------------------------------------
// Procedure: resetAdaptiveOffset()
//   Purpose: Start each engagement over at the ceiling, with no
//            assumption carried in from the last target or the last
//            run -- a real contact's reaction band is not known in
//            advance, so every engagement re-discovers it from scratch.

void TargetCoordinator::resetAdaptiveOffset()
{
  m_block_offset_cur   = -1;
  m_block_reaction_hdg = 0;
  m_block_reaction_utc = 0;
  m_block_reacted       = false;
}

//------------------------------------------------------------
// Procedure: updateAdaptiveOffset()
//   Purpose: Feel the offset inward from the ceiling until the
//            intruder is actually observed reacting, instead of
//            assuming its reaction band from a known avoidance config.
//
//            Method: hold the present offset for block_reaction_wait
//            seconds and watch the intruder's heading against the
//            baseline recorded when that offset was set. If it turns
//            away from the block side by block_reaction_deg or more,
//            the offset is working -- freeze it (m_block_reacted) for
//            the rest of the engagement, since a smaller number bought
//            nothing but risk once a reaction is confirmed. If the wait
//            elapses with no such turn, the offset was either too wide
//            to register or the intruder has not gotten there yet;
//            shave block_shrink_step off it, re-baseline, and try
//            again, no lower than block_min_offset.
//
//            Deliberately NOT tied to blockStillGood()/reissue: a
//            point can still be geometrically "good" (intruder hasn't
//            passed it) while sitting outside the target's true
//            reaction band, and that is exactly the case this is meant
//            to catch. Invalidating the point on a shrink forces
//            assignAndPostBlock() to redraw the formation at the new
//            offset on its next pass.

void TargetCoordinator::updateAdaptiveOffset()
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return;
  double heading = angle360(t->second.getHeading());

  if(m_block_offset_cur < 0) {
    m_block_offset_cur   = blockOffset();
    m_block_reaction_hdg = heading;
    m_block_reaction_utc = MOOSTime();
    return;
  }

  if(m_block_reacted)
    return;

  // Turn away from the block side, signed positive when it is the turn
  // we want: block_side=+1 means we are standing to starboard, so the
  // wanted turn is to port, i.e. heading decreasing.
  double turn = angle180(heading - m_block_reaction_hdg) * -m_block_side;

  if(turn >= m_block_reaction_deg) {
    m_block_reacted = true;
    reportEvent("Intruder reacting at offset " +
		doubleToStringX(m_block_offset_cur,1) + " m - holding it.");
    return;
  }

  if((MOOSTime() - m_block_reaction_utc) < m_block_reaction_wait)
    return;

  double next = m_block_offset_cur - m_block_shrink_step;
  if(next < m_block_min_offset)
    next = m_block_min_offset;

  if(next < m_block_offset_cur) {
    m_block_offset_cur   = next;
    m_block_reaction_hdg = heading;
    m_block_reaction_utc = MOOSTime();
    reportEvent("No reaction inside " + doubleToStringX(m_block_reaction_wait,0) +
		"s - narrowing block offset to " +
		doubleToStringX(m_block_offset_cur,1) + " m.");
    // The point standing now was drawn at the old, wider offset -- it
    // is no longer the geometry we are trying, so force a redraw.
    if(m_block_valid.size() > 0)
      m_block_valid[0] = false;
  }
  else
    m_block_reaction_utc = MOOSTime();  // at the floor -- just keep waiting
}

//------------------------------------------------------------
// Procedure: blockPoints()
//   Purpose: Where the two USVs should be standing, in absolute
//            coordinates, to make the intruder turn the way we want.
//
//            Two roles:
//
//            BLOCKER  lead metres ahead of the intruder and
//              block_offset off its track, on the side OPPOSITE the
//              turn we want -- a vessel turns away from the thing in
//              front of it, so to send it to starboard we stand on its
//              port bow. Its projected CPA on the intruder's present
//              course is that offset, which is what makes the
//              intruder's avoidance produce a gradient at all.
//
//            DENIER   further out on the same side and not as far
//              ahead, sealing the turn back the way we do not want it
//              to go. It does not threaten the present track and is not
//              meant to: it bites only if the intruder turns into it.
//
//            When the intruder is already heading where we want it, the
//            pair splits into a funnel instead -- one either side, both
//            wide enough to leave the lane ahead open -- because
//            blocking the bow of a vessel that is already leaving would
//            only turn it back in.

bool TargetCoordinator::blockPoints(double& b1x, double& b1y,
				    double& b2x, double& b2y) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(false);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(false);
  double hdg = angle360(t->second.getHeading());

  bool blocking = false;
  if(m_exit_valid)
    blocking = (fabs(angle180(m_exit_dir - hdg)) >= m_block_dead_deg);

  // Not block_lead: the largest lead the blocker can actually reach in
  // time. See feasibleLead().
  double lead1 = feasibleLead();
  double lead2 = lead1 * m_block_lead2;
  double fx, fy;

  if(blocking) {
    double side_ang = angle360(hdg + (90.0 * m_block_side));

    projectPoint(hdg, lead1, tx, ty, fx, fy);
    projectPoint(side_ang, currentBlockOffset(), fx, fy, b1x, b1y);

    projectPoint(hdg, lead2, tx, ty, fx, fy);
    projectPoint(side_ang, currentBlockOffset() + m_block_span, fx, fy, b2x, b2y);
    return(true);
  }

  double wide = currentBlockOffset() + (m_block_span / 2.0);
  projectPoint(hdg, lead1, tx, ty, fx, fy);
  projectPoint(angle360(hdg + 90), wide, fx, fy, b1x, b1y);
  projectPoint(angle360(hdg - 90), wide, fx, fy, b2x, b2y);
  return(true);
}

//------------------------------------------------------------
// Procedure: blockStillGood()
//   Purpose: Is this station still doing anything?
//
//            A blocking point is spent when the intruder has turned
//            enough that the point no longer lies close to the track
//            ahead of it: at that moment its projected CPA to the point
//            rises above the range its avoidance cares about, and the
//            point stops being a block at all. Then, and only then, a
//            new one is issued in front of where it is going now.
//
//            That "then, and only then" is the whole trick. A station
//            recomputed every iteration would translate with the
//            intruder, the USV would chase it at the intruder's own
//            speed, and the relative velocity -- which is what a CPA is
//            made of -- would be zero again. Which is exactly what
//            BHV_Trail does once it is on station, and exactly why the
//            trail arc could never evict anything.

bool TargetCoordinator::blockStillGood(double bx, double by) const
{
  return(!blockSpent(bx, by) && !blockOffTrack(bx, by));
}

//------------------------------------------------------------
// Procedure: blockSpent()
//   Purpose: The point is GENUINELY dead: the intruder is level with
//            it or has passed it. Nothing can rescue this one -- a
//            station abeam or astern blocks nothing -- so no amount of
//            transit credit should keep it alive.

bool TargetCoordinator::blockSpent(double bx, double by) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(true);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(true);
  double hdg  = angle360(t->second.getHeading());
  double rads = degToRadians(hdg);

  double dx = bx - tx;
  double dy = by - ty;
  double along = (dx * sin(rads)) + (dy * cos(rads));

  return(along < m_block_min_lead);
}

//------------------------------------------------------------
// Procedure: blockOffTrack()
//   Purpose: The intruder's present track no longer passes close to
//            the point, so the point has stopped biting. RECOVERABLE:
//            the intruder may yaw back, and in the meantime the USV
//            assigned to the point is usually still a long way from
//            it. See blockTransitHold().

bool TargetCoordinator::blockOffTrack(double bx, double by) const
{
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(true);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(true);
  double hdg  = angle360(t->second.getHeading());
  double rads = degToRadians(hdg);

  double dx = bx - tx;
  double dy = by - ty;
  double lateral = (dx * cos(rads)) - (dy * sin(rads));

  return(fabs(lateral) > m_block_slack);
}

//------------------------------------------------------------
// Procedure: usvSpeed()
//   Purpose: How fast the USV is actually going, floored so a vehicle
//            reported stopped cannot make a transit estimate infinite.

double TargetCoordinator::usvSpeed(unsigned int vidx) const
{
  if(vidx >= m_vnames.size())
    return(m_block_usv_speed);

  map<string, NodeRecord>::const_iterator v =
    m_records.find(tolower(m_vnames[vidx]));
  if(v == m_records.end())
    return(m_block_usv_speed);

  double spd = v->second.getSpeed();
  if(spd < 0.5)
    return(m_block_usv_speed);
  return(spd);
}

//------------------------------------------------------------
// Procedure: transitTime()
//   Purpose: Seconds for the given USV to cover the straight-line
//            distance to a point at its present speed. A floor, not a
//            plan: the real path is longer, so this UNDER-estimates,
//            which is the safe direction for a commit timer.

double TargetCoordinator::transitTime(unsigned int vidx,
				      double bx, double by) const
{
  if(vidx >= m_vnames.size())
    return(0);

  map<string, NodeRecord>::const_iterator v =
    m_records.find(tolower(m_vnames[vidx]));
  if(v == m_records.end())
    return(0);

  double vx, vy;
  if(!recordPosition(v->second, vx, vy))
    return(0);

  return(distPointToPoint(vx, vy, bx, by) / usvSpeed(vidx));
}

//------------------------------------------------------------
// Procedure: blockTransitHold()
//   Purpose: Is the USV assigned to this station still on its way
//            there, with a fair chance of arriving?
//
//   WHY THIS EXISTS. Measured over two untouched 900 s benchmark runs:
//   a blocking station survived a median of 7-14 s, while the USV
//   ordered to it needed a median of 23-39 s just to cover the straight
//   line. Only 30-37 percent of stations were ever reached at all; the
//   median closest approach to one was 17-35 m. The fleet was being
//   ordered to points it could not physically occupy, and every time
//   the intruder yawed enough to shift its track the order was torn up
//   and replaced -- discarding the ten or fifteen seconds of transit
//   already spent and starting a fresh thirty-second journey somewhere
//   else. From the chart that reads exactly as it was described: the
//   eviction keeps failing and restarting from another direction.
//
//   The re-anchor cadence was set by how fast the INTRUDER manoeuvres.
//   It has to be set by how fast OUR OWN vehicles move, because that is
//   what bounds the rate at which the fleet can change its mind about
//   anything. So an off-track point is not torn up while the vehicle
//   assigned to it is still closing on it and inside its transit
//   estimate. Capped by block_transit_cap so an unreachable point
//   cannot be held forever, and never applied to a point the intruder
//   has already drawn level with (blockSpent), which no amount of
//   transit credit can rescue.

bool TargetCoordinator::blockTransitHold(unsigned int slot) const
{
  if(!m_block_commit_transit)
    return(false);
  if(slot >= m_block_valid.size() || !m_block_valid[slot])
    return(false);

  unsigned int n = m_vnames.size();
  unsigned int vidx = (m_assignment.size() == n) ? m_assignment[slot] : slot;
  if(vidx >= n)
    return(false);

  double bx = m_block_x[slot];
  double by = m_block_y[slot];

  // Already there: nothing left to protect, judge it on geometry alone.
  map<string, NodeRecord>::const_iterator v =
    m_records.find(tolower(m_vnames[vidx]));
  if(v == m_records.end())
    return(false);
  double vx, vy;
  if(!recordPosition(v->second, vx, vy))
    return(false);
  if(distPointToPoint(vx, vy, bx, by) < m_block_min_offset)
    return(false);

  double budget = m_block_transit_frac * transitTime(vidx, bx, by);
  if(budget > m_block_transit_cap)
    budget = m_block_transit_cap;

  return((MOOSTime() - m_block_utc[slot]) < budget);
}

//------------------------------------------------------------
// Procedure: feasibleLead()
//   Purpose: How far ahead of the intruder to plant the blocker, given
//            that the blocker has to GET there.
//
//            block_lead is what we would like: far enough ahead that
//            the intruder has time to see the geometry and make one
//            substantial alteration (Rule 8b). But a point the USV
//            reaches after the intruder has already passed it is not a
//            block at all -- it is the tail chase this whole mode
//            exists to avoid, and it is what the benchmark measured.
//
//            So: walk the lead down from block_lead until the blocker
//            can reach the station before the intruder draws level with
//            it, and take the largest lead that passes. Floored at
//            block_min_lead -- below that the point is spent on
//            arrival, so if even that is not reachable we take it
//            anyway and let the normal staleness logic retire it.

double TargetCoordinator::feasibleLead() const
{
  if(!m_block_feasible)
    return(m_block_lead);

  unsigned int n = m_vnames.size();
  if(n == 0)
    return(m_block_lead);

  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(m_block_lead);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(m_block_lead);
  double hdg = angle360(t->second.getHeading());

  double tspd = t->second.getSpeed();
  if(tspd < 0.1)
    return(m_block_lead);

  unsigned int vidx = (m_assignment.size() == n) ? m_assignment[0] : 0;
  if(vidx >= n)
    return(m_block_lead);

  double side_ang = angle360(hdg + (90.0 * m_block_side));

  for(double lead=m_block_lead; lead>=m_block_min_lead; lead-=5.0) {
    double fx, fy, bx, by;
    projectPoint(hdg, lead, tx, ty, fx, fy);
    projectPoint(side_ang, currentBlockOffset(), fx, fy, bx, by);

    if(transitTime(vidx, bx, by) <= (lead / tspd))
      return(lead);
  }

  return(m_block_min_lead);
}

//------------------------------------------------------------
// Procedure: assignAndPostBlock()
//   Purpose: Put the USVs on ground-fixed blocking stations and post
//            them as BLOCK_UPDATE_<VNAME>.
//
//   WHY NOT THE TRAIL ARC. The intruder runs BHV_AvoidCollision, whose
//   objective function is metric(projected CPA): zero below 12 m, 100
//   above 25 m, linear between. BHV_Trail, once inside its capture
//   radius, steers the USV at the CONTACT'S own course and speed (see
//   the "Inside radius" branch of BHV_Trail.cpp). Two vessels moving at
//   the same velocity have a relative velocity of zero, so the
//   intruder's projected CPA to a trail station equals the present
//   range whatever course or speed it picks -- the objective function
//   is FLAT, and a flat function contributes nothing to the helm's
//   solution whatever weight it carries. Swept over the intruder's
//   whole manoeuvre space, the best escape it can find beats holding
//   course by 0.0 at every station range from 30 m down to 14 m. The
//   arc was an escort, not a wall.
//
//   A station that does not move with the intruder does have a relative
//   velocity, so it does have a CPA, so it does produce a gradient. At
//   50 m ahead and 18 m off the track, holding course scores 59.6
//   against 100 for turning away: it turns.

void TargetCoordinator::assignAndPostBlock()
{
  unsigned int n = m_vnames.size();
  if(n == 0)
    return;

  double tx, ty;
  if(!targetPosition(tx, ty))
    return;

  if(m_block_x.size() != n) {
    m_block_x.assign(n, 0);
    m_block_y.assign(n, 0);
    m_block_valid.assign(n, false);
    m_block_utc.assign(n, 0);
    m_block_giving_way.assign(n, false);
  }

  // Which side are we standing on? LATCHED the same way the exit
  // bearing is (see updateExitDirection): a flip has to be wanted
  // continuously for block_side_commit seconds before it is taken.
  //
  // The exit bearing itself is already latched, but that alone was not
  // enough -- this side flip was being decided fresh off the intruder's
  // INSTANTANEOUS heading every iteration, with no commit timer of its
  // own. A target weaving, or simply yawing under its own COLREGs
  // avoidance while the pincer works on it, crosses the exit bearing
  // back and forth every few seconds; each crossing flipped block_side
  // and threw away whatever the formation had built up on the old side,
  // so the run looked like repeated failed attempts from alternating
  // directions rather than one sustained push. Requiring the new side
  // to be wanted for a beat, not an instant, is what the exit latch
  // already does for the strategic direction -- this does the same for
  // the tactical one.
  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if((t != m_records.end()) && m_exit_valid) {
    double delta = angle180(m_exit_dir - angle360(t->second.getHeading()));
    int wanted = (delta >= 0) ? -1 : 1;
    if((fabs(delta) >= m_block_dead_deg) && (wanted != m_block_side)) {
      if(m_block_side_challenge_since < 0)
	m_block_side_challenge_since = MOOSTime();
      else if((MOOSTime() - m_block_side_challenge_since) >= m_block_side_commit) {
	m_block_side                 = wanted;
	m_block_side_challenge_since = -1;
      }
    }
    else
      m_block_side_challenge_since = -1;
  }

  // Side is settled for this pass -- now feel out how close the
  // offset needs to be to actually get a reaction from THIS target.
  // Not while the blocker is giving way: no station is actually
  // testing anything then, so any heading change is noise, not signal.
  bool blocker_giving_way = (m_block_giving_way.size() > 0) && m_block_giving_way[0];
  if(!blocker_giving_way)
    updateAdaptiveOffset();

  // SAFETY OVERRIDE, ahead of everything else this function does.
  //
  // The station is placed block_offset off the intruder's TRACK, and
  // that is the distance it passes at only while it keeps to that
  // track. An intruder that turns into the station instead -- because
  // its own avoidance resolved the other way, or its patrol leg pulled
  // it round -- closes on a vehicle that is, by design, sitting still.
  // Measured before this rule existed: 8.1 m at the worst, and 29
  // seconds of a 20-minute run inside 14 m. The station-keeping
  // behavior and avdtgt_ were sharing the helm's solution between them
  // and neither was winning outright.
  //
  // So the coordinator gives way itself, at a range it chooses, rather
  // than leaving it to a contest between two behaviors: inside
  // block_abort the station is replaced by a point directly away from
  // the intruder, which turns the USV out and opens the range. The
  // block is a position, not a refusal to keep clear. Blocking resumes
  // once the range is back past block_abort + block_rearm, with a fresh
  // point in front of wherever the intruder is going by then.
  for(unsigned int slot=0; slot<n; slot++) {
    unsigned int vidx = (m_assignment.size() == n) ? m_assignment[slot] : slot;
    if(vidx >= n)
      continue;

    map<string, NodeRecord>::const_iterator v =
      m_records.find(tolower(m_vnames[vidx]));
    if(v == m_records.end())
      continue;
    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      continue;

    double range = distPointToPoint(vx, vy, tx, ty);
    double trip  = m_block_giving_way[slot] ? (m_block_abort + m_block_rearm)
                                            : m_block_abort;
    bool   yield = (range < trip);

    if(!yield) {
      if(m_block_giving_way[slot]) {
	m_block_giving_way[slot] = false;
	Notify("BLOCK_GIVEWAY_" + toupper(m_vnames[vidx]), "false");
      }
      continue;
    }

    if(!m_block_giving_way[slot]) {
      m_block_giving_way[slot] = true;
      m_block_valid[slot]      = false;   // this point is finished
      m_giveways++;
      Notify("BLOCK_GIVEWAY_" + toupper(m_vnames[vidx]), "true");
      reportEvent("Giving way: " + m_vnames[vidx] + " at " +
		  doubleToStringX(range,1) + " m of the intruder - block released.");
    }

    // Retreat point: straight out from the intruder along the bearing
    // the USV is already on, so the turn away is the shortest one.
    double away = angle360(relAng(tx, ty, vx, vy));
    double rx, ry;
    projectPoint(away, m_block_abort + m_block_rearm, tx, ty, rx, ry);

    string vname = toupper(m_vnames[vidx]);
    Notify("BLOCK_UPDATE_" + vname, "station_pt=" +
	   doubleToStringX(rx,1) + "," + doubleToStringX(ry,1));
    Notify("BLOCK_ACTIVE_" + vname, "true");
  }

  // The blocker is slot 0. Its point going stale is what re-issues the
  // whole formation, so both stations always belong to the same
  // geometry rather than drifting apart.
  bool have    = m_block_valid[0];
  bool reissue = !have;
  if(have) {
    // Spent (intruder level with it or past it) retires the point
    // outright. Merely off-track does not, while the USV ordered to it
    // is still closing and inside its transit budget -- see
    // blockTransitHold() for the measurements that forced this.
    if(blockSpent(m_block_x[0], m_block_y[0]))
      reissue = true;
    else if(blockOffTrack(m_block_x[0], m_block_y[0]) && !blockTransitHold(0))
      reissue = true;
  }
  if(reissue && have && ((MOOSTime() - m_block_utc[0]) < m_block_interval))
    reissue = false;

  // While the blocker is giving way there is no formation to re-issue:
  // a new one now would only drag the second USV around behind a
  // manoeuvre that is about keeping clear, not about blocking.
  if(m_block_giving_way[0])
    reissue = false;

  if(reissue) {
    double b1x, b1y, b2x, b2y;
    if(!blockPoints(b1x, b1y, b2x, b2y))
      return;

    // Pair the USVs to the two stations by which is closer, then leave
    // it alone for the rest of the run (swap_lock): changing places
    // mid-engagement means both leave station at once.
    if((m_assignment.size() != n) || !m_swap_lock || !m_engaged) {
      unsigned int blocker = 0;
      unsigned int denier  = (n > 1) ? 1 : 0;
      if(n > 1) {
	double d[2][2];
	for(unsigned int i=0; i<2; i++) {
	  map<string, NodeRecord>::const_iterator v =
	    m_records.find(tolower(m_vnames[i]));
	  if(v == m_records.end())
	    return;
	  double vx, vy;
	  if(!recordPosition(v->second, vx, vy))
	    return;
	  d[i][0] = distPointToPoint(vx, vy, b1x, b1y);
	  d[i][1] = distPointToPoint(vx, vy, b2x, b2y);
	}
	if((d[1][0] + d[0][1]) < (d[0][0] + d[1][1])) {
	  blocker = 1;
	  denier  = 0;
	}
      }
      m_assignment.clear();
      m_assignment.push_back(blocker);
      if(n > 1)
	m_assignment.push_back(denier);
    }

    m_block_x[0] = b1x;
    m_block_y[0] = b1y;
    if(n > 1) {
      m_block_x[1] = b2x;
      m_block_y[1] = b2y;
    }
    for(unsigned int i=0; i<n; i++) {
      m_block_valid[i] = true;
      m_block_utc[i]   = MOOSTime();
    }
    m_block_reissues++;
  }

  // Post. BHV_StationKeep needs the point only once, but a vehicle that
  // joined late needs it too, so it goes out again on a slow heartbeat.
  bool refresh = (!m_block_posted ||
		  ((MOOSTime() - m_last_post_utc) > m_repost_interval));
  if(!reissue && !refresh)
    return;

  for(unsigned int slot=0; slot<n; slot++) {
    unsigned int vidx = slot;
    if(m_assignment.size() == n)
      vidx = m_assignment[slot];
    if(vidx >= n)
      continue;

    if(m_block_giving_way[slot])
      continue;

    string vname = toupper(m_vnames[vidx]);
    string spec  = "station_pt=" + doubleToStringX(m_block_x[slot],1) +
      "," + doubleToStringX(m_block_y[slot],1);
    Notify("BLOCK_UPDATE_" + vname, spec);
    Notify("BLOCK_ACTIVE_" + vname, "true");
  }

  m_block_posted  = true;
  m_last_post_utc = MOOSTime();
  m_assignments_posted++;

  if(m_draw_pincer)
    postBlockVisuals();
}

//------------------------------------------------------------
// Procedure: postBlockVisuals()

void TargetCoordinator::postBlockVisuals()
{
  unsigned int n = m_vnames.size();
  if(m_block_x.size() != n)
    return;

  XYSegList line;
  for(unsigned int slot=0; slot<n; slot++) {
    unsigned int vidx = slot;
    if(m_assignment.size() == n)
      vidx = m_assignment[slot];
    if(vidx >= n)
      continue;

    line.add_vertex(m_block_x[slot], m_block_y[slot]);

    XYPoint station(m_block_x[slot], m_block_y[slot]);
    station.set_label(m_vnames[vidx] + "_block");
    station.set_vertex_size(8);
    station.set_vertex_color((slot == 0) ? "red" : "orange");
    station.set_label_color((slot == 0) ? "red" : "orange");
    station.set_active(true);
    Notify("VIEW_POINT", station.get_spec());
  }

  if(line.size() < 2)
    return;
  line.set_label("blockline");
  line.set_edge_color("red");
  line.set_vertex_color("red");
  line.set_edge_size(2);
  line.set_vertex_size(4);
  line.set_active(true);
  Notify("VIEW_SEGLIST", line.get_spec());
}

//------------------------------------------------------------
// Procedure: clearBlockVisuals()

void TargetCoordinator::clearBlockVisuals()
{
  XYSegList line;
  line.set_label("blockline");
  line.set_active(false);
  Notify("VIEW_SEGLIST", line.get_spec());

  for(unsigned int i=0; i<m_vnames.size(); i++) {
    XYPoint station;
    station.set_label(m_vnames[i] + "_block");
    station.set_active(false);
    Notify("VIEW_POINT", station.get_spec());
  }
}

//------------------------------------------------------------
// Procedure: standDownBlock()
//   Purpose: Release the blocking stations. BLOCK_ACTIVE is what the
//            station-keeping behavior is conditioned on, so dropping it
//            hands the USV back to its search pattern.

void TargetCoordinator::standDownBlock()
{
  for(unsigned int i=0; i<m_vnames.size(); i++)
    Notify("BLOCK_ACTIVE_" + toupper(m_vnames[i]), "false");

  for(unsigned int i=0; i<m_block_valid.size(); i++)
    m_block_valid[i] = false;
  for(unsigned int i=0; i<m_block_giving_way.size(); i++)
    m_block_giving_way[i] = false;

  m_block_posted = false;
  m_block_side_challenge_since = -1;
  resetAdaptiveOffset();
  clearBlockVisuals();
}

//------------------------------------------------------------
// Procedure: handleMailRegionPoly()
//   Purpose: Take the search-area boundary from pRegionDivider. The
//            centroid alone is not enough to answer "has the intruder
//            actually been pushed out of the area" -- that needs the
//            real boundary.

void TargetCoordinator::handleMailRegionPoly(string str)
{
  if(tolower(stripBlankEnds(str)) == "none") {
    m_region_known = false;
    return;
  }

  XYPolygon poly = string2Poly(str);
  if(poly.size() < 3) {
    reportRunWarning("Unparsable REGION_POLY: " + str);
    return;
  }

  m_region       = poly;
  m_region_known = true;
}

//------------------------------------------------------------
// Procedure: handleMailSuspectReport()
//   Purpose: Take a detection transition from one USV's own contact
//            manager.
//
//   Format:  "vname=abe # contact=target # state=on|off"
//
//            This is the seam a real sensor plugs into. Detection used
//            to be a range comparison here on the mothership, against a
//            NODE_REPORT the intruder broadcasts about itself twice a
//            second -- which is ground truth handed over by the
//            simulator, not something any sensor provides. Now each USV
//            decides for itself whether the intruder is a contact
//            within range, and reports only the on/off transition. Swap
//            the simulated NODE_REPORT source for radar, AIS or EO
//            tracks and nothing above this line changes.

void TargetCoordinator::handleMailSuspectReport(string str)
{
  m_last_suspect_report = str;
  m_suspect_reports++;

  string vname, contact, state;

  // Accept ':' , '#' and ',' as field separators. The mission uses ':'
  // because pContactMgrV20's off_flag path splits on '#' while its
  // on_flag path does not, but a report arriving in any of the three
  // forms should still be understood rather than dropped -- dropping it
  // is what left the contact latch stuck on.
  string norm = str;
  for(unsigned int i=0; i<norm.length(); i++)
    if((norm[i] == '#') || (norm[i] == ','))
      norm[i] = ':';

  vector<string> svector = parseString(norm, ':');
  for(unsigned int i=0; i<svector.size(); i++) {
    string part  = stripBlankEnds(svector[i]);
    string left  = tolower(biteStringX(part, '='));
    string right = stripBlankEnds(part);
    if(left == "vname")        vname   = tolower(right);
    else if(left == "contact") contact = tolower(right);
    else if(left == "state")   state   = tolower(right);
  }

  if((vname == "") || (state == "")) {
    reportRunWarning("Unparsable SUSPECT_REPORT: " + str);
    return;
  }

  // Only the designated intruder counts. A USV picking up its partner
  // is a contact, not a suspect.
  if((contact != "") && (contact != m_target_name))
    return;

  m_contact_held[vname] = (state == "on");
}

//------------------------------------------------------------
// Procedure: contactHeld()
//   Purpose: True while any USV still holds the intruder. Losing it
//            from every USV is what "driven off" means when detection
//            is a sensor rather than a coordinate feed -- so this is
//            also half the release test.

bool TargetCoordinator::contactHeld() const
{
  map<string, bool>::const_iterator q;
  for(q=m_contact_held.begin(); q!=m_contact_held.end(); q++)
    if(q->second)
      return(true);
  return(false);
}

//------------------------------------------------------------
// Procedure: regionOffset()
//   Purpose: Signed distance from the search-area boundary: negative
//            inside, positive outside. One continuous number that says
//            how far through the eviction a point is.

double TargetCoordinator::regionOffset(double x, double y) const
{
  double dist = m_region.dist_to_poly(x, y);
  if(dist < 0)
    return(0);
  return(m_region.contains(x, y) ? -dist : dist);
}

//------------------------------------------------------------
// Procedure: exitDistance()
//   Purpose: How far the intruder must travel along one bearing before
//            it counts as evicted -- region_exit_buffer clear of the
//            boundary, which is the same test the engagement is
//            released on.
//
//            THIS REPLACES A RAY CAST TO THE POLYGON. XYPolygon's
//            dist_to_poly(x,y,angle) answers "how far along this
//            bearing until I HIT the polygon", and that is only the
//            same question as "how far until I am out" while the
//            intruder is still inside. The moment it crosses the
//            boundary the meaning inverts: the only bearings that hit
//            the polygon at all are the ones pointing back INTO the
//            area, every outward bearing scored -1 and was skipped, and
//            so the cheapest "exit" became a bearing aimed at the water
//            the intruder had just been pushed out of. The blocking arc
//            is anchored to the reciprocal of that, so the pair crossed
//            to the seaward side of the intruder and pushed it back in.
//            Measured on the default region, the arc centre swung from
//            0 to 250 degrees within a few metres of the crossing.
//
//            Marching the signed offset instead is the same number
//            while inside (edge distance plus the buffer) and stays
//            meaningful outside, falling to zero as the intruder
//            completes its own eviction. It never inverts, because
//            "further out" is further out on both sides of the line.
//
//   Returns: 0 if the intruder is already clear, -1 if this bearing
//            never gets it clear (running parallel to a boundary, say).

double TargetCoordinator::exitDistance(double bearing) const
{
  if(!m_region_known)
    return(-1);

  double tx, ty;
  if(!targetPosition(tx, ty))
    return(-1);

  double goal = m_region_exit_buffer;
  double here = regionOffset(tx, ty);
  if(here >= goal)
    return(0);

  const double step     = 5;
  const double max_dist = 400;

  double prev = here;
  for(double dist=step; dist<=max_dist; dist+=step) {
    double px, py;
    projectPoint(bearing, dist, tx, ty, px, py);
    double off = regionOffset(px, py);
    if(off >= goal) {
      // Interpolate inside the step that crossed, so the cost is a
      // smooth function of the intruder's position rather than a
      // staircase the hysteresis would have to be sized around.
      double span = off - prev;
      if(span <= 0)
	return(dist);
      return(dist - step + (step * ((goal - prev) / span)));
    }
    prev = off;
  }

  return(-1);
}

//------------------------------------------------------------
// Procedure: exitCost()
//   Purpose: What it costs to push the intruder out along one bearing.
//
//            Two terms. The first is how far it still has to travel
//            along that bearing to be clear of the area by the
//            operator's eviction distance. The second charges
//            heading_bias metres for every degree that bearing sits
//            off the course it is already steering.
//
//            The distance term alone picks the nearest way out, which
//            on a long thin region often means demanding the intruder
//            reverse course -- it resists, and the engagement drags.
//            The heading term buys that momentum back at a stated
//            price. It has to stay CHEAP relative to the field: at the
//            old 2 m/deg a 90-degree turn cost 180 m on a region only
//            120 m across, so the heading could never be outvoted and
//            an intruder 10 m inside one edge but pointing at the
//            opposite one was escorted the whole way across. At 0.35
//            the same turn costs 31 m and the geometry normally wins.
//
//            Note that near the boundary the distance term compresses
//            -- every outward bearing is short -- so this cost is
//            almost pure heading there, and would follow the intruder
//            round any turn it made. That is exactly why the choice is
//            latched for the run rather than re-taken every iteration.

double TargetCoordinator::exitCost(double bearing, double heading) const
{
  double dist = exitDistance(bearing);
  if(dist < 0)
    return(-1);   // this bearing never gets the intruder clear

  double turn = fabs(angle180(bearing - heading));
  return(dist + (m_heading_bias * turn));
}

//------------------------------------------------------------
// Procedure: updateExitDirection()
//   Purpose: Pick the bearing the intruder is to be pushed out along,
//            and -- once the engagement is running -- COMMIT to it.
//
//            The doctrine is "pick a lane and push it out that lane".
//            A wall that swings does not push, and near the boundary
//            the cost function is almost pure heading (see exitCost),
//            so an intruder that reaches the edge and turns hard along
//            it would drag the exit bearing, and the whole blocking
//            arc, round with it -- leaving the water it had just been
//            pushed out of unguarded, which is the way back in.
//
//            So while engaged the exit is latched, and a rival bearing
//            has to be better by exit_margin CONTINUOUSLY for
//            exit_commit seconds before it takes over. The margin
//            alone was not enough: 15 m of cost is 7.5 degrees of
//            heading at the default bias, which the intruder's own yaw
//            covers. The commit timer is what the intruder cannot
//            fake, because it would have to hold the new course.
//
//            The latch is dropped between engagements (postAlert), so
//            each run starts from the geometry as it actually is.

void TargetCoordinator::updateExitDirection()
{
  if(!m_region_known) {
    m_exit_valid = false;
    return;
  }

  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end()) {
    m_exit_valid = false;
    return;
  }
  double heading = angle360(t->second.getHeading());

  double tx, ty;
  bool   have_pos = recordPosition(t->second, tx, ty);

  double best_dir  = 0;
  double best_cost = -1;
  for(unsigned int i=0; i<36; i++) {
    double bearing = i * 10.0;
    double cost = exitCost(bearing, heading);
    if(cost < 0)
      continue;
    if((best_cost < 0) || (cost < best_cost)) {
      best_cost = cost;
      best_dir  = bearing;
    }
  }

  if(best_cost < 0) {
    // Nothing gets it clear from here. Hold whatever we had rather
    // than dropping to the centroid fallback mid-run.
    if(!m_engaged)
      m_exit_valid = false;
    return;
  }

  if(!m_exit_valid) {
    m_exit_dir             = best_dir;
    m_exit_cost            = best_cost;
    m_exit_valid           = true;
    m_exit_since           = MOOSTime();
    m_exit_challenge_since = -1;
    return;
  }

  // Re-price the exit we are already committed to before comparing.
  double current_cost = exitCost(m_exit_dir, heading);
  if(current_cost < 0) {
    m_exit_dir             = best_dir;
    m_exit_cost            = best_cost;
    m_exit_since           = MOOSTime();
    m_exit_challenge_since = -1;
    return;
  }
  m_exit_cost = current_cost;

  bool beaten = ((current_cost - best_cost) > m_exit_margin);

  if(!beaten) {
    m_exit_challenge_since = -1;
    return;
  }

  // Not engaged yet: no wall is standing on this bearing, so there is
  // nothing to protect and the freshest answer is the right one.
  if(!m_engaged) {
    m_exit_dir   = best_dir;
    m_exit_cost  = best_cost;
    m_exit_since = MOOSTime();
    return;
  }

  // Already evicted, but the engagement has not been released yet
  // (release_hold is still counting). Every bearing now costs the same
  // zero distance, so the comparison degenerates to "whichever way it
  // happens to be pointing" -- and if it has turned back toward the
  // area, re-anchoring on that would swing the pair to the seaward side
  // and push it back IN for the last few seconds of the run. Hold.
  if(have_pos && (regionOffset(tx, ty) >= m_region_exit_buffer)) {
    m_exit_challenge_since = -1;
    return;
  }

  // A re-anchoring must never ask the intruder to travel FURTHER to
  // get out than the bearing it is already being pushed along. Without
  // this the heading term can buy a bearing that leads back through the
  // area: an intruder 10 m outside the south edge that turns north is
  // charged nothing for its own heading, and "out through the far side"
  // (145 m away, straight back across the water it was just pushed out
  // of) then scores better than "another 5 m south". The pair would
  // cross to seaward of it and escort it back in.
  //
  // heading_bias still picks the lane at the start of the run, which is
  // what it is for. It just cannot un-pick it in the wrong direction.
  double cur_raw  = exitDistance(m_exit_dir);
  double best_raw = exitDistance(best_dir);
  if((cur_raw >= 0) && (best_raw > cur_raw)) {
    m_exit_challenge_since = -1;
    return;
  }

  if(m_exit_challenge_since < 0)
    m_exit_challenge_since = MOOSTime();

  if((MOOSTime() - m_exit_challenge_since) < m_exit_commit)
    return;

  m_exit_dir             = best_dir;
  m_exit_cost            = best_cost;
  m_exit_since           = MOOSTime();
  m_exit_challenge_since = -1;
}

//------------------------------------------------------------
// Procedure: detectRange()
//   Purpose: How close the intruder has to get before a USV notices
//            it. Zero configured means "whatever the operator set the
//            sensor range to", so the ring drawn on the chart, the
//            lane spacing the search plan uses, and the range the
//            fleet actually reacts at are all the same number.

double TargetCoordinator::detectRange() const
{
  if(m_detect_range > 0)
    return(m_detect_range);
  return(m_sensor_radius * m_detect_scale);
}

//------------------------------------------------------------
// Procedure: nearestRangeToTarget()
//   Purpose: Range from the closest USV to the intruder.

bool TargetCoordinator::nearestRangeToTarget(double& range, string& vname) const
{
  double tx, ty;
  if(!targetPosition(tx, ty))
    return(false);

  bool found = false;
  for(unsigned int i=0; i<m_vnames.size(); i++) {
    map<string, NodeRecord>::const_iterator v = m_records.find(tolower(m_vnames[i]));
    if(v == m_records.end())
      continue;

    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      continue;

    double dist = distPointToPoint(tx, ty, vx, vy);
    if(!found || (dist < range)) {
      range = dist;
      vname = m_vnames[i];
      found = true;
    }
  }

  return(found);
}

//------------------------------------------------------------
// Procedure: targetClearOfRegion()
//   Purpose: True when the intruder is outside the search area by at
//            least the given buffer. The buffer is what stops a target
//            skimming the boundary from toggling the alert on and off
//            as it crosses back and forth.

bool TargetCoordinator::targetClearOfRegion(double buffer) const
{
  if(!m_region_known)
    return(false);

  double tx, ty;
  if(!targetPosition(tx, ty))
    return(false);

  if(m_region.contains(tx, ty))
    return(false);

  return(m_region.dist_to_poly(tx, ty) > buffer);
}

//------------------------------------------------------------
// Procedure: updateEngagement()
//   Purpose: The engagement cycle the mission is actually about.
//
//            SEARCHING -> INTERCEPTING
//              an intruder inside the search area comes within
//              detection range of any USV.
//
//            INTERCEPTING -> SEARCHING  (a successful eviction)
//              the intruder has been pushed clear of the search AREA,
//              and has stayed clear for release_hold seconds.
//
//              Losing sensor contact on the intruder is NOT a release
//              condition by itself. Once engaged, the fleet stays
//              committed until the intruder is confirmed outside the
//              region -- a target that ducks out of a USV's detection
//              range for a beat (a wake, a COLREGs avoidance swerve, an
//              unlucky bearing) is still inside the area and still the
//              job. Releasing on contact loss let a target that had
//              only slipped out of sensor range wander back toward the
//              centre while the fleet had already stood down.
//
//              The hold is what makes the region-exit test a driven-off
//              judgement rather than a reading of one noisy instant:
//              the target weaving along the boundary while the pincer
//              works on it crosses out and back repeatedly before it
//              finally gives up and leaves.
//
//            After a success, detection is deaf for reengage_delay
//            seconds so the USVs get to turn back into their search
//            pattern instead of re-acquiring the same target while it
//            is still on their doorstep. Once that lapses the cycle is
//            armed again and the next intrusion is picked up on its
//            own.
//
//            An intercept started from the operator's INTERCEPT button
//            runs through exactly the same release test. It is a way to
//            trigger the cycle early, not a mode of its own.

void TargetCoordinator::updateEngagement()
{
  m_range_known = nearestRangeToTarget(m_last_range, m_closest_vname);

  if(!m_engaged) {
    if(!m_auto_engage || !m_deployed)
      return;
    if(MOOSTime() < m_rearm_utc)
      return;

    // Only intrusions count. A target loitering outside the area is
    // not something to go chasing after -- the fleet's job is the
    // water the operator drew.
    bool inside = !m_region_known || !targetClearOfRegion(0);
    if(inside && contactHeld()) {
      m_engaged       = true;
      m_engage_source = "auto";
      m_clear_since   = -1;
      m_had_contact   = false;
      m_detections++;
      m_assignment.clear();
      m_posted_angle.clear();
      m_posted_cant.clear();
      m_exit_valid           = false;
      m_exit_challenge_since = -1;
      m_block_side_challenge_since = -1;
      resetAdaptiveOffset();
      postAlert(true);
    }
    return;
  }

  // Engaged, from whichever source. The release test is the same
  // either way -- see the note on the trigger variable in OnNewMail.
  if((m_engage_source == "operator") && !m_operator_release) {
    m_clear_since = -1;
    return;
  }

  // Region exit is the ONLY release condition. Losing the contact
  // (target outside every USV's sensor range) must NOT release the
  // engagement by itself -- once acquired, the fleet stays committed
  // to driving the intruder out of the AREA, not merely out of sensor
  // range. Sensor range is transient: the intruder can dodge behind a
  // wake, momentarily overtake a USV's own COLREGs standoff, or simply
  // be at a bad geometry for a beat or two, and none of that means it
  // has actually left. Releasing on contact loss let a target that had
  // only slipped out of range wander back toward the centre while the
  // fleet had already stood down to search.
  //
  // This does mean the coordinator has to keep steering off a NODE
  // report the USVs can no longer sense while contact is lost -- see
  // targetPosition()/recordPosition(): m_records still holds the last
  // report for as long as it is not stale, which is what lets the
  // pincer keep tracking through a brief drop-out instead of freezing
  // the stations at the last-seen point.
  bool clear = targetClearOfRegion(m_region_exit_buffer);

  if(!clear) {
    // The intruder is being held. Remember it, because only an
    // engagement that actually contested the target can end in an
    // eviction -- pressing INTERCEPT while the target happens to be
    // sitting outside the area already would otherwise score a
    // success for driving off something that was never there.
    m_had_contact = true;
    m_clear_since = -1;
    return;
  }

  if(m_clear_since < 0)
    m_clear_since = MOOSTime();

  if((MOOSTime() - m_clear_since) < m_release_hold)
    return;

  m_engaged     = false;
  m_clear_since = -1;
  m_rearm_utc   = MOOSTime() + m_reengage_delay;
  if(m_had_contact)
    m_evictions++;
  m_assignment.clear();
  m_posted_angle.clear();
  m_posted_cant.clear();
  m_exit_challenge_since = -1;
  postAlert(false);
}

//------------------------------------------------------------
// Procedure: postAlert()
//   Purpose: Put the fleet into (or out of) intercept. Both halves of
//            the mode are posted, never just the one that changed: a
//            TARGET_ALERT left true under a SURVEY is exactly the
//            stale-flag problem the shoreside buttons were rewritten
//            to avoid.
//
//            These are deliberately NOT the operator's TARGET_ALERT_ALL
//            / SURVEY_ALL. Those arrive here from the shoreside, and
//            pShare refuses to inject an inbound variable into the
//            local DB once an outbound route of the same name exists --
//            publishing them from here would silently kill the
//            operator's own buttons. Separate names, routed straight
//            down to the vehicles, keep both paths alive.

void TargetCoordinator::postAlert(bool active)
{
  Notify(m_alert_var, active ? "true" : "false");
  Notify(m_survey_var, active ? "false" : "true");

  Notify("SHIELD_STATE", active ? "intercepting" : "searching");
  Notify("SHIELD_EVICTIONS", (double)m_evictions);
  Notify("SHIELD_DETECTIONS", (double)m_detections);

  if(!active && m_had_contact)
    reportEvent("Target driven off - eviction #" + uintToString(m_evictions) +
		" - fleet back to search.");
  else if(!active)
    reportEvent("Stood down - target was never inside the fleet's reach.");
  else
    reportEvent("Target detected by " + m_closest_vname + " at " +
		doubleToStringX(m_last_range,1) + "m - intercepting.");
}

//------------------------------------------------------------
// Procedure: postDetectRings()
//   Purpose: Draw each USV's detection range on the chart, so the
//            operator can see the thing the success criterion is
//            actually stated in terms of: the intruder is evicted
//            when it is outside these rings (and outside the area).
//
//            Drawn as a seglist rather than a circle because
//            pMarineViewer hides circles unless the operator has
//            turned them on, and this ring is not optional scenery --
//            it is the rule of the engagement.

void TargetCoordinator::postDetectRings()
{
  // The rings follow the vehicles, so they are re-posted on a timer
  // rather than every iteration: at AppTick they would be a steady
  // stream of full seglist specs across the pShare link to the
  // shoreside for no visible gain.
  if((MOOSTime() - m_last_ring_utc) < 1.0)
    return;
  m_last_ring_utc = MOOSTime();

  double radius = detectRange();
  string color  = m_intercept_active ? "orange" : "green";

  for(unsigned int i=0; i<m_vnames.size(); i++) {
    map<string, NodeRecord>::const_iterator v = m_records.find(tolower(m_vnames[i]));
    if(v == m_records.end())
      continue;

    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      continue;

    XYSegList ring;
    unsigned int pts = 24;
    for(unsigned int k=0; k<=pts; k++) {
      double ang = (2.0 * M_PI * k) / (double)pts;
      ring.add_vertex(vx + (radius * cos(ang)), vy + (radius * sin(ang)));
    }
    ring.set_label(m_vnames[i] + "_detect");
    ring.set_edge_color(color);
    ring.set_vertex_color("invisible");
    ring.set_edge_size(1);
    ring.set_vertex_size(0);
    ring.set_active(true);
    Notify("VIEW_SEGLIST", ring.get_spec());
  }

  m_rings_drawn = true;
}

//------------------------------------------------------------
// Procedure: eraseDetectRings()

void TargetCoordinator::eraseDetectRings()
{
  for(unsigned int i=0; i<m_vnames.size(); i++) {
    XYSegList ring;
    ring.set_label(m_vnames[i] + "_detect");
    ring.set_active(false);
    Notify("VIEW_SEGLIST", ring.get_spec());
  }
  m_rings_drawn = false;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool TargetCoordinator::buildReport()
{
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicles:    " << stringVectorToString(m_vnames, ':') << endl;
  m_msgs << "  Target Name: " << m_target_name << endl;
  m_msgs << "  Formation:   " << formationName()
	 << "   (switch with " << m_formation_var << ")" << endl;
  m_msgs << "  Spread Deg:  " << doubleToStringX(m_spread_deg) << endl;
  m_msgs << "  Trail Range: " << doubleToStringX(trailRange());
  if(trailRange() > m_trail_range)
    m_msgs << "  (raised from " << doubleToStringX(m_trail_range)
	   << " by min_standoff)";
  m_msgs << endl;
  m_msgs << "  Cant Angle:  " << doubleToStringX(cantAngle())
	 << " deg inward off the parallel";
  if(cantAngle() <= 0)
    m_msgs << "  (0 = bows parallel to the target)";
  m_msgs << endl;
  if(cantAngle() > 0) {
    m_msgs << "               ";
    for(unsigned int i=0; i<m_posted_cant.size(); i++) {
      if(i < m_vnames.size())
	m_msgs << m_vnames[i] << " "
	       << doubleToStringX(m_posted_cant[i],1) << "  ";
    }
    m_msgs << " (signed: + stbd of target course, - port)" << endl;
  }
  m_msgs << "  Swap Margin: " << doubleToStringX(m_swap_margin_deg) << endl;
  m_msgs << "  Stn Frame:   "
	 << (m_station_relative ? "relative (offset from target heading)"
	                        : "absolute (compass bearing)") << endl;
  m_msgs << "  Exit Bearing:";
  if(m_exit_valid) {
    m_msgs << " " << doubleToStringX(m_exit_dir,0)
	   << " deg  (cost " << doubleToStringX(m_exit_cost,0)
	   << " m = metres still to travel + heading_bias "
	   << doubleToStringX(m_heading_bias,1) << " m/deg)" << endl;
    m_msgs << "               ";
    if(m_engaged) {
      m_msgs << "LATCHED for " << doubleToStringX(MOOSTime()-m_exit_since,0)
	     << " s;  a rival must beat it by "
	     << doubleToStringX(m_exit_margin,0) << " m for "
	     << doubleToStringX(m_exit_commit,0) << " s";
      if(m_exit_challenge_since > 0)
	m_msgs << "  <-- challenged for "
	       << doubleToStringX(MOOSTime()-m_exit_challenge_since,1) << " s";
      m_msgs << endl;
    }
    else
      m_msgs << "free-running (latches when the engagement starts)" << endl;
  }
  else
    m_msgs << " none yet (no region polygon)" << endl;
  m_msgs << "  Bow Guard:   " << doubleToStringX(m_bow_guard_deg,0)
	 << " deg either side of the intruder's head kept clear of stations"
	 << endl;
  m_msgs << "               (the outward half-plane is open by"
	 << " construction - that is the sea room)" << endl;
  m_msgs << "  Trigger Var: " << m_trigger_var << endl;
  m_msgs << endl;

  m_msgs << "Engagement Cycle:" << endl;
  m_msgs << "  Auto Engage:   " << boolToString(m_auto_engage)
	 << "   (alert var: " << m_alert_var << ")" << endl;
  m_msgs << "  Sensor Range:  " << doubleToStringX(detectRange(),1)
	 << " m   (SENSOR_RADIUS - one number: search-plan lane spacing,"
	 << endl
	 << "                  the ring drawn round each USV, and the range"
	 << endl
	 << "                  each USV's contact manager alerts at)" << endl;
  m_msgs << "  Detection:     contact-driven (SUSPECT_REPORT from the USVs'"
	 << " own contact managers)" << endl;
  m_msgs << "  Last report:   " << m_last_suspect_report
	 << "   (" << m_suspect_reports << " received)" << endl;
  m_msgs << "  Holding now:   ";
  {
    string who;
    map<string,bool>::const_iterator q;
    for(q=m_contact_held.begin(); q!=m_contact_held.end(); q++)
      if(q->second)
	who += (who=="" ? "" : ",") + q->first;
    m_msgs << (who=="" ? "nobody" : who) << endl;
  }
  m_msgs << "  Release:       clear of the region only (contact loss does"
	 << " NOT release), held " << doubleToStringX(m_release_hold,1)
	 << " s" << endl;
  m_msgs << "  Exit Buffer:   " << doubleToStringX(m_region_exit_buffer,1)
	 << " m outside the region   (EVICT_BUFFER - Action/Eviction"
	 << " Distance)" << endl;

  string state = "SEARCHING (armed)";
  if(m_engaged)
    state = "INTERCEPTING (" + m_engage_source + ")";
  else if(MOOSTime() < m_rearm_utc)
    state = "SEARCHING (re-arming in " +
      doubleToStringX(m_rearm_utc - MOOSTime(),1) + "s)";
  else if(!m_deployed)
    state = "IDLE (not deployed)";

  m_msgs << "  State:         " << state << endl;
  if(m_range_known)
    m_msgs << "  Nearest USV:   " << m_closest_vname << " at "
	   << doubleToStringX(m_last_range,1) << " m" << endl;
  else
    m_msgs << "  Nearest USV:   unknown (no target report yet)" << endl;
  if(m_engaged)
    m_msgs << "  Clear test:    outside-region="
	   << boolToString(targetClearOfRegion(m_region_exit_buffer))
	   << "   (contact-held=" << boolToString(contactHeld())
	   << " is informational only -- see Release above)" << endl;
  if(m_engaged && (m_clear_since > 0))
    m_msgs << "  Clear for:     "
	   << doubleToStringX(MOOSTime() - m_clear_since,1) << " s of "
	   << doubleToStringX(m_release_hold,1) << endl;
  m_msgs << "  Detections:    " << m_detections << endl;
  m_msgs << "  Evictions:     " << m_evictions << "   <-- successes" << endl;
  m_msgs << endl;

  m_msgs << "State:" << endl;
  m_msgs << "  Intercept Active:   " << boolToString(m_intercept_active) << endl;
  m_msgs << "  Assignments Posted: " << m_assignments_posted << endl;
  if(m_block_mode) {
    m_msgs << "  Block Stations:     lead " << doubleToStringX(m_block_lead,0)
	   << " m, offset " << doubleToStringX(currentBlockOffset(),0)
	   << " m of " << doubleToStringX(blockOffset(),0) << " m ceiling"
	   << (m_block_reacted ? "  [reaction confirmed]" : "  [still feeling it out]")
	   << ", give way inside " << doubleToStringX(m_block_abort,0) << " m" << endl;
    m_msgs << "  Block Lead:         " << doubleToStringX(feasibleLead(),0)
	   << " m of " << doubleToStringX(m_block_lead,0)
	   << " m wanted   (walked down to what the blocker can reach in time)"
	   << endl;
    if(m_block_commit_transit && (m_block_valid.size() > 0) && m_block_valid[0]) {
      unsigned int vidx = (m_assignment.size() == m_vnames.size())
	? m_assignment[0] : 0;
      m_msgs << "  Blocker Transit:    needs "
	     << doubleToStringX(transitTime(vidx, m_block_x[0], m_block_y[0]),0)
	     << " s to reach station, held for "
	     << doubleToStringX(MOOSTime() - m_block_utc[0],0) << " s"
	     << (blockTransitHold(0) ? "   [committed]" : "") << endl;
    }
    m_msgs << "  Points Re-issued:   " << m_block_reissues << endl;
    m_msgs << "  Gave Way:           " << m_giveways << " times" << endl;
  }
  m_msgs << "  Station Swaps:      " << m_swaps
	 << "   (declined by swap_lock: " << m_swaps_declined << ")" << endl;
  m_msgs << "  Region Known:       " << boolToString(m_region_known) << endl;
  if(m_center_known)
    m_msgs << "  Region Center:      " << doubleToStringX(m_center_x,1)
	   << "," << doubleToStringX(m_center_y,1) << endl;
  else
    m_msgs << "  Region Center:      unknown (falling back to stern arc)" << endl;
  m_msgs << endl;

  map<string, NodeRecord>::iterator t = m_records.find(m_target_name);
  if(t == m_records.end()) {
    m_msgs << "No NODE_REPORT for target '" << m_target_name
	   << "' yet - holding." << endl;
    return(true);
  }

  double base_angle = 0;
  string basis;
  if(!pincerBaseAngle(base_angle, basis))
    return(true);

  m_msgs << "Target heading:    "
	 << doubleToStringX(t->second.getHeading(),1) << endl;
  m_msgs << "Arc centered on:   " << doubleToStringX(base_angle,1)
	 << "  [" << basis << "]" << endl;
  if(m_station_relative)
    m_msgs << "(station angles below are ABSOLUTE bearings from the target; "
	   << "they are ordered as offsets from its heading)" << endl << endl;
  else
    m_msgs << "(station angles below are ABSOLUTE bearings from the target)"
	   << endl << endl;

  vector<double> slots = computeSlots(base_angle);
  vector<unsigned int> order = m_assignment;
  if(order.size() != m_vnames.size())
    order = bestAssignment(slots);

  ACTable actab(4);
  actab << "Slot | Vehicle | Station Bearing (abs) | Position Known";
  actab.addHeaderLines();

  for(unsigned int slot=0; slot<order.size(); slot++) {
    unsigned int vidx = order[slot];
    if(vidx >= m_vnames.size())
      continue;
    string vname = m_vnames[vidx];
    string known = (m_records.count(tolower(vname)) > 0) ? "yes" : "no";
    actab << uintToString(slot) << vname
	  << doubleToStringX(slots[slot],1) << known;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
