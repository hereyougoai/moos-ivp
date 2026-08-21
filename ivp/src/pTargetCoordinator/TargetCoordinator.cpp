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
  m_spread_deg  = 120;
  m_trail_range = 25;
  m_trigger_var = "TARGET_ALERT_ALL";

  m_herd_mode       = true;
  m_swap_margin_deg = 30;
  m_center_deadzone = 20;
  m_repost_interval = 4;
  m_draw_pincer     = true;

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

  // Half-width of the sector dead ahead of the intruder that stations
  // are nudged out of. Small on purpose -- see bowGuard().
  m_bow_guard_deg      = 20;

  // Floor under trail_range. The interception station has to sit
  // outside the band where the USV's own COLREGs avoidance of the
  // intruder starts pulling, or the two fight and the pass ends up
  // closer than either intended. See trailRange().
  m_min_standoff       = 30;

  // Floor under the detection range. See detectRange().
  // Eviction direction. heading_bias is the cost, in metres, charged
  // per degree the exit bearing sits off the intruder's own head, so a
  // big value means "push it out the way it is already going even if
  // that is the long way round". See updateExitDirection().
  m_heading_bias       = 2.0;
  m_exit_margin        = 15;

  m_exit_dir           = 0;
  m_exit_valid         = false;
  m_exit_cost          = 0;

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
    if(!m_engaged)
      clearPincerVisuals();
    m_intercept_active = m_engaged;
  }

  if(m_intercept_active)
    assignAndPost();

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
      else if(param == "pincer_mode") {
	string mode = tolower(value);
	if(mode == "herd") {
	  m_herd_mode = true;
	  handled = true;
	}
	else if(mode == "stern") {
	  m_herd_mode = false;
	  handled = true;
	}
      }
      else if(param == "swap_margin_deg")
	handled = setNonNegDoubleOnString(m_swap_margin_deg, value);
      else if(param == "center_deadzone")
	handled = setNonNegDoubleOnString(m_center_deadzone, value);
      else if(param == "repost_interval")
	handled = setPosDoubleOnString(m_repost_interval, value);
      else if(param == "draw_pincer")
	handled = setBooleanOnString(m_draw_pincer, value);
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
  Register("SUSPECT_REPORT", 0);
  Register("DEPLOY_ALL", 0);
  Register(m_trigger_var, 0);
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
//            bearings from the target, so BHV_Trail must be running
//            with trail_angle_type=absolute.

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
    double cost_now = assignmentCost(m_assignment, slots);
    double cost_new = assignmentCost(candidate, slots);
    if(cost_now - cost_new > m_swap_margin_deg) {
      m_assignment = candidate;
      m_swaps++;
    }
  }

  if(m_posted_angle.size() != n)
    m_posted_angle.assign(n, -999);

  // Re-post when a station has actually moved, and on a slow heartbeat
  // so a vehicle that (re)joined late still picks up its assignment.
  bool refresh_due = ((MOOSTime() - m_last_post_utc) > m_repost_interval);

  bool posted_any = false;
  for(unsigned int slot=0; slot<n; slot++) {
    unsigned int vidx = m_assignment[slot];
    if(vidx >= n)
      continue;

    if(!refresh_due && (fabs(angle180(slots[slot] - m_posted_angle[vidx])) < 1.0))
      continue;

    string spec = "trail_angle=" + doubleToStringX(slots[slot],1);
    spec += " # trail_angle_type=absolute";
    spec += " # trail_range=" + doubleToStringX(trailRange(),1);

    Notify("TRAIL_UPDATE_" + toupper(m_vnames[vidx]), spec);
    m_posted_angle[vidx] = slots[slot];
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
// Procedure: exitCost()
//   Purpose: What it costs to push the intruder out along one bearing.
//
//            Two terms. The first is how far it has to travel along
//            that bearing to leave the region -- a ray cast from its
//            position to the boundary. The second charges
//            heading_bias metres for every degree that bearing sits
//            off the course it is already steering.
//
//            The distance term alone picks the nearest way out, which
//            on a long thin region often means demanding the intruder
//            reverse course. The heading term is what makes the fleet
//            work with the intruder's own momentum instead of against
//            it: at the default bias of 2 m/deg, a 90-degree turn is
//            charged 180 m, so an exit that is aligned with its head
//            wins unless it is very much longer.

double TargetCoordinator::exitCost(double bearing, double heading) const
{
  double tx, ty;
  if(!targetPosition(tx, ty))
    return(-1);

  double dist = m_region.dist_to_poly(tx, ty, bearing);
  if(dist < 0)
    return(-1);   // ray never leaves through the boundary

  double turn = fabs(angle180(bearing - heading));
  return(dist + (m_heading_bias * turn));
}

//------------------------------------------------------------
// Procedure: updateExitDirection()
//   Purpose: Pick the bearing the intruder should be pushed out along,
//            and hold it until another is clearly better.
//
//            Without the hysteresis the choice flips as the intruder
//            yaws, and the blocking arc -- which is anchored to the
//            reciprocal of this -- swings with it. A wall that swings
//            does not push.

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
    m_exit_valid = false;
    return;
  }

  if(!m_exit_valid) {
    m_exit_dir   = best_dir;
    m_exit_cost  = best_cost;
    m_exit_valid = true;
    return;
  }

  // Re-price the exit we are already committed to before comparing.
  double current_cost = exitCost(m_exit_dir, heading);
  if(current_cost < 0) {
    m_exit_dir  = best_dir;
    m_exit_cost = best_cost;
    return;
  }

  if((current_cost - best_cost) > m_exit_margin) {
    m_exit_dir  = best_dir;
    m_exit_cost = best_cost;
  }
  else
    m_exit_cost = current_cost;
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
  m_msgs << "  Pincer Mode: " << (m_herd_mode ? "herd" : "stern") << endl;
  m_msgs << "  Spread Deg:  " << doubleToStringX(m_spread_deg) << endl;
  m_msgs << "  Trail Range: " << doubleToStringX(trailRange());
  if(trailRange() > m_trail_range)
    m_msgs << "  (raised from " << doubleToStringX(m_trail_range)
	   << " by min_standoff)";
  m_msgs << endl;
  m_msgs << "  Swap Margin: " << doubleToStringX(m_swap_margin_deg) << endl;
  m_msgs << "  Exit Bearing:";
  if(m_exit_valid)
    m_msgs << " " << doubleToStringX(m_exit_dir,0)
	   << " deg  (cost " << doubleToStringX(m_exit_cost,0)
	   << " m, heading_bias " << doubleToStringX(m_heading_bias,1)
	   << " m/deg)" << endl;
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
	 << " m outside the region" << endl;

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
  m_msgs << "  Station Swaps:      " << m_swaps << endl;
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
