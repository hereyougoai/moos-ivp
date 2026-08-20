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

    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if(key == "REGION_CENTER")
      handleMailRegionCenter(sval);
    else if(key == m_trigger_var) {
      bool active = (tolower(sval) == "true");
      // On the rising edge, forget the previous pairing so the USVs are
      // matched to whichever flank they happen to be on right now.
      if(active && !m_intercept_active) {
	m_assignment.clear();
	m_posted_angle.clear();
      }
      if(!active && m_intercept_active)
	clearPincerVisuals();
      m_intercept_active = active;
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

  if(m_intercept_active)
    assignAndPost();

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
    slots.push_back(angle360(base_angle + offset));
  }
  return(slots);
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
    spec += " # trail_range=" + doubleToStringX(m_trail_range,1);

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
    projectPoint(slots[slot], m_trail_range, tx, ty, sx, sy);

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
// Procedure: buildReport()

bool TargetCoordinator::buildReport()
{
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicles:    " << stringVectorToString(m_vnames, ':') << endl;
  m_msgs << "  Target Name: " << m_target_name << endl;
  m_msgs << "  Pincer Mode: " << (m_herd_mode ? "herd" : "stern") << endl;
  m_msgs << "  Spread Deg:  " << doubleToStringX(m_spread_deg) << endl;
  m_msgs << "  Trail Range: " << doubleToStringX(m_trail_range) << endl;
  m_msgs << "  Swap Margin: " << doubleToStringX(m_swap_margin_deg) << endl;
  m_msgs << "  Trigger Var: " << m_trigger_var << endl;
  m_msgs << endl;

  m_msgs << "State:" << endl;
  m_msgs << "  Intercept Active:   " << boolToString(m_intercept_active) << endl;
  m_msgs << "  Assignments Posted: " << m_assignments_posted << endl;
  m_msgs << "  Station Swaps:      " << m_swaps << endl;
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
