/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetCoordinator.cpp                                */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include "TargetCoordinator.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "AngleUtils.h"
#include "NodeRecordUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

TargetCoordinator::TargetCoordinator()
{
  m_target_name = "target";
  m_spread_deg  = 90;
  m_trail_range = 30;
  m_trigger_var = "TARGET_ALERT_ALL";

  m_intercept_active   = false;
  m_assignments_posted = 0;
  m_geodesy_ok         = false;
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
    else if(key == m_trigger_var)
      m_intercept_active = (tolower(sval) == "true");
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

      if(!handled)
	reportUnhandledConfigWarning(orig);
    }
  }

  if(m_vnames.size() == 0)
    reportConfigWarning("No vnames configured - nothing to assign trail angles to.");

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
// Procedure: computeSlots()
//   Purpose: The fan of trail angles around the target's stern,
//            ordered from the target's port flank to its starboard
//            flank. Slot i sits at -spread/2 + i*(spread/(n-1))
//            away from dead astern.
//
//   NOTE: These are RELATIVE angles. BHV_Trail defaults to
//         trail_angle_type=relative, i.e. it steers to
//         (contact_heading + trail_angle), so 180 means dead
//         astern and we must not fold the target's heading in
//         ourselves. Relative also keeps the station correct
//         between our posts, since the helm re-derives it from
//         the freshest contact heading every iteration.

vector<double> TargetCoordinator::computeSlots(unsigned int n) const
{
  vector<double> slots;
  for(unsigned int i=0; i<n; i++) {
    double offset = 0;
    if(n > 1)
      offset = -(m_spread_deg / 2.0) + (i * (m_spread_deg / (double)(n-1)));
    slots.push_back(angle180(180 + offset));
  }
  return(slots);
}

//------------------------------------------------------------
// Procedure: assignSlots()
//   Purpose: Match each USV to a flank slot by where it actually
//            is right now, so nobody is told to swap sides across
//            the target's wake.
//
//            Both the slots and the USVs are ordered by the same
//            angular sweep (relative to the target's stern), then
//            paired off in order. Sorting both sides by the same
//            key means the pairing never crosses: the USV already
//            furthest to port takes the port-most slot, and so on.
//
//   Returns: assignment[i] = index into m_vnames for slot i.
//            Falls back to identity order for any USV whose
//            position we have not heard yet.

vector<unsigned int> TargetCoordinator::assignSlots(double base_angle) const
{
  unsigned int n = m_vnames.size();

  vector<unsigned int> ident;
  for(unsigned int i=0; i<n; i++)
    ident.push_back(i);

  map<string, NodeRecord>::const_iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return(ident);

  double tx, ty;
  if(!recordPosition(t->second, tx, ty))
    return(ident);

  // Sweep key: signed angular offset of the USV's current bearing
  // (as seen from the target) away from the stern bearing, in
  // [-180,180]. Port flank sorts before starboard flank.
  vector<double> keys;
  for(unsigned int i=0; i<n; i++) {
    map<string, NodeRecord>::const_iterator v = m_records.find(tolower(m_vnames[i]));
    if(v == m_records.end())
      return(ident);

    double vx, vy;
    if(!recordPosition(v->second, vx, vy))
      return(ident);

    double bearing = relAng(tx, ty, vx, vy);
    keys.push_back(angle180(bearing - base_angle));
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
//   Purpose: Space the USVs around the target's stern (heading +
//            180) over m_spread_deg, assigning each to the flank
//            it is already nearest to, and post a
//            TRAIL_UPDATE_<VNAME> for each.

void TargetCoordinator::assignAndPost()
{
  unsigned int n = m_vnames.size();
  if(n == 0)
    return;

  map<string, NodeRecord>::iterator t = m_records.find(m_target_name);
  if(t == m_records.end())
    return;

  // Absolute stern bearing, used only to order the vehicles by
  // which flank they are already on. The posted angles are relative.
  double base_angle = angle360(t->second.getHeading() + 180);

  vector<double>       slots = computeSlots(n);
  vector<unsigned int> order = assignSlots(base_angle);

  for(unsigned int slot=0; slot<n; slot++) {
    string vname       = m_vnames[order[slot]];
    string vname_upper = toupper(vname);

    string spec = "trail_angle=" + doubleToStringX(slots[slot],1);
    spec += " # trail_range=" + doubleToStringX(m_trail_range,1);

    Notify("TRAIL_UPDATE_" + vname_upper, spec);
  }

  m_assignments_posted++;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool TargetCoordinator::buildReport()
{
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicles:    " << stringVectorToString(m_vnames, ':') << endl;
  m_msgs << "  Target Name: " << m_target_name << endl;
  m_msgs << "  Spread Deg:  " << doubleToStringX(m_spread_deg) << endl;
  m_msgs << "  Trail Range: " << doubleToStringX(m_trail_range) << endl;
  m_msgs << "  Trigger Var: " << m_trigger_var << endl;
  m_msgs << endl;

  m_msgs << "State:" << endl;
  m_msgs << "  Intercept Active:   " << boolToString(m_intercept_active) << endl;
  m_msgs << "  Assignments Posted: " << m_assignments_posted << endl;
  m_msgs << endl;

  map<string, NodeRecord>::iterator t = m_records.find(m_target_name);
  bool have_target = (t != m_records.end());
  unsigned int n = m_vnames.size();

  if(!have_target) {
    m_msgs << "No NODE_REPORT for target '" << m_target_name
	   << "' yet - holding." << endl;
    return(true);
  }

  double base_angle = angle360(t->second.getHeading() + 180);
  m_msgs << "Target heading:       "
	 << doubleToStringX(t->second.getHeading(),1) << endl;
  m_msgs << "Target stern bearing: " << doubleToStringX(base_angle,1) << endl;
  m_msgs << "(trail angles below are relative to target heading)" << endl << endl;

  vector<double>       slots = computeSlots(n);
  vector<unsigned int> order = assignSlots(base_angle);

  ACTable actab(4);
  actab << "Slot | Vehicle | Trail Angle (rel) | Position Known";
  actab.addHeaderLines();

  for(unsigned int slot=0; slot<n; slot++) {
    string vname = m_vnames[order[slot]];
    string known = (m_records.count(tolower(vname)) > 0) ? "yes" : "no";
    actab << uintToString(slot) << vname
	  << doubleToStringX(slots[slot],1) << known;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
