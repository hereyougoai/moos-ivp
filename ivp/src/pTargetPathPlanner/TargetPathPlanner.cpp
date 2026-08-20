/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetPathPlanner.cpp                                */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cmath>
#include "TargetPathPlanner.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPoint.h"
#include "XYFormatUtilsSegl.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

TargetPathPlanner::TargetPathPlanner()
{
  m_update_var    = "TGT_WPT_UPDATE";
  m_route_label   = "target_route";
  m_default_route = "";
  m_speed         = 0;
  m_auto_apply    = true;
  m_close_loop    = true;

  m_route_applied    = false;
  m_route_is_default = false;
  m_routes_posted    = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TargetPathPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key  = msg.GetKey();
    string sval = msg.GetString();
    if(!msg.IsString())
      sval = doubleToStringX(msg.GetDouble(), 4);

    if(key == "TARGET_PATH_VERTEX") {
      if(!handleMailVertex(sval))
	reportRunWarning("Unparsable TARGET_PATH_VERTEX: " + sval);
    }
    else if(key == "TARGET_PATH_ROUTE") {
      if(!handleMailRoute(sval))
	reportRunWarning("Unparsable TARGET_PATH_ROUTE: " + sval);
    }
    else if(key == "TARGET_PATH_CLEAR")
      handleMailClear();
    else if(key == "TARGET_PATH_UNDO")
      handleMailUndo();
    else if(key == "TARGET_PATH_APPLY") {
      if(!applyRoute())
	reportRunWarning("Need at least two route points before APPLY.");
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TargetPathPlanner::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool TargetPathPlanner::Iterate()
{
  AppCastingMOOSApp::Iterate();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool TargetPathPlanner::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

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
      if(param == "update_var")
	handled = setNonWhiteVarOnString(m_update_var, value);
      else if(param == "route_label")
	handled = setNonWhiteVarOnString(m_route_label, value);
      else if(param == "default_route") {
	m_default_route = value;
	handled = true;
      }
      else if(param == "speed")
	handled = setDoubleOnString(m_speed, value);
      else if(param == "auto_apply")
	handled = setBooleanOnString(m_auto_apply, value);
      else if(param == "close_loop")
	handled = setBooleanOnString(m_close_loop, value);

      if(!handled)
	reportUnhandledConfigWarning(orig);
    }
  }

  // The launch script's built-in patrol is the starting route, so the
  // mission still runs untouched if the operator never draws one. It is
  // flagged as the default, because the operator's first click means
  // "start a new route here", not "add a fifth corner to the route I
  // never drew".
  if(m_default_route != "") {
    if(!handleMailRoute(m_default_route))
      reportConfigWarning("Unparsable default_route: " + m_default_route);
    else {
      m_route_is_default = true;
      drawRoute();
    }
  }

  registerVariables();
  return(true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void TargetPathPlanner::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("TARGET_PATH_VERTEX", 0);
  Register("TARGET_PATH_ROUTE", 0);
  Register("TARGET_PATH_CLEAR", 0);
  Register("TARGET_PATH_UNDO", 0);
  Register("TARGET_PATH_APPLY", 0);
}

//------------------------------------------------------------
// Procedure: handleMailVertex()
//   Purpose: Append one operator-clicked waypoint. Click order is
//            the route order -- no hull, no sorting.
//
//   Format: "x=val,y=val"  or  "val,val"

bool TargetPathPlanner::handleMailVertex(string str)
{
  double vx = 0, vy = 0;
  bool x_set = false, y_set = false;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string part  = stripBlankEnds(svector[i]);
    string left  = tolower(biteStringX(part, '='));
    string right = part;

    if((left == "x") && isNumber(right)) {
      vx = atof(right.c_str());
      x_set = true;
    }
    else if((left == "y") && isNumber(right)) {
      vy = atof(right.c_str());
      y_set = true;
    }
    else if((right == "") && isNumber(left)) {
      if(!x_set) {
	vx = atof(left.c_str());
	x_set = true;
      }
      else if(!y_set) {
	vy = atof(left.c_str());
	y_set = true;
      }
    }
  }

  if(!x_set || !y_set)
    return(false);

  // First click on an untouched default route starts a fresh one.
  if(m_route_is_default) {
    m_vx.clear();
    m_vy.clear();
    m_route_is_default = false;
  }

  m_vx.push_back(vx);
  m_vy.push_back(vy);

  drawRoute();
  if(m_auto_apply)
    applyRoute();

  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailRoute()
//   Purpose: Take a whole route at once, e.g. "30,-120:150,-120:...".
//            Lets the route be scripted or restored, not only clicked.

bool TargetPathPlanner::handleMailRoute(string str)
{
  XYSegList segl = string2SegList(stripBlankEnds(str));
  if(segl.size() < 2)
    return(false);

  m_vx.clear();
  m_vy.clear();
  for(unsigned int i=0; i<segl.size(); i++) {
    m_vx.push_back(segl.get_vx(i));
    m_vy.push_back(segl.get_vy(i));
  }

  m_route_is_default = false;

  drawRoute();
  if(m_auto_apply)
    applyRoute();
  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailClear()

void TargetPathPlanner::handleMailClear()
{
  m_vx.clear();
  m_vy.clear();
  m_route_applied    = false;
  m_route_is_default = false;
  eraseRoute();
}

//------------------------------------------------------------
// Procedure: handleMailUndo()
//   Purpose: Drop the most recent click. Misplacing one corner of a
//            route should not mean re-drawing the whole thing.

void TargetPathPlanner::handleMailUndo()
{
  if(m_vx.size() == 0)
    return;

  m_vx.pop_back();
  m_vy.pop_back();

  if(m_vx.size() == 0) {
    eraseRoute();
    return;
  }

  drawRoute();
  if(m_auto_apply)
    applyRoute();
}

//------------------------------------------------------------
// Procedure: currentRoute()

XYSegList TargetPathPlanner::currentRoute() const
{
  XYSegList segl;
  for(unsigned int i=0; i<m_vx.size(); i++)
    segl.add_vertex(m_vx[i], m_vy[i]);
  return(segl);
}

//------------------------------------------------------------
// Procedure: applyRoute()
//   Purpose: Hand the route to the target's patrol behavior.
//
//            The behavior is configured repeat=999, so it cycles back
//            to the first point on its own; the route is therefore a
//            loop without us duplicating the first point here.

bool TargetPathPlanner::applyRoute()
{
  if(m_vx.size() < 2)
    return(false);

  XYSegList segl = currentRoute();
  segl.set_label(m_route_label);

  string spec = "points = " + segl.get_spec_pts();
  if(m_speed > 0)
    spec += " # speed = " + doubleToStringX(m_speed, 2);

  Notify(m_update_var, spec);

  m_last_posted   = spec;
  m_route_applied = true;
  m_routes_posted++;
  return(true);
}

//------------------------------------------------------------
// Procedure: drawRoute()

void TargetPathPlanner::drawRoute()
{
  if(m_vx.size() == 0)
    return;

  XYSegList segl = currentRoute();

  // Draw the closing leg too, so what the operator sees on the chart
  // matches the loop the vessel will actually fly.
  if(m_close_loop && (m_vx.size() > 2))
    segl.add_vertex(m_vx[0], m_vy[0]);

  segl.set_label(m_route_label);
  segl.set_edge_color("orange");
  segl.set_vertex_color("gold");
  segl.set_vertex_size(3);
  segl.set_edge_size(1);
  segl.set_active(true);
  Notify("VIEW_SEGLIST", segl.get_spec());

  // A distinct marker on the first waypoint: with a loop drawn there is
  // otherwise no way to tell where the vessel starts its circuit.
  XYPoint pt(m_vx[0], m_vy[0]);
  pt.set_label(m_route_label + "_start");
  pt.set_color("vertex", "orange");
  pt.set_vertex_size(6);
  pt.set_active(true);
  Notify("VIEW_POINT", pt.get_spec());
}

//------------------------------------------------------------
// Procedure: eraseRoute()

void TargetPathPlanner::eraseRoute()
{
  XYSegList segl;
  segl.set_label(m_route_label);
  segl.set_active(false);
  Notify("VIEW_SEGLIST", segl.get_spec());

  XYPoint pt(0, 0);
  pt.set_label(m_route_label + "_start");
  pt.set_active(false);
  Notify("VIEW_POINT", pt.get_spec());
}

//------------------------------------------------------------
// Procedure: buildReport()

bool TargetPathPlanner::buildReport()
{
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Update Var:   " << m_update_var << endl;
  m_msgs << "  Auto Apply:   " << boolToString(m_auto_apply) << endl;
  m_msgs << "  Close Loop:   " << boolToString(m_close_loop) << endl;
  m_msgs << endl;

  m_msgs << "State:" << endl;
  m_msgs << "  Route Points: " << m_vx.size()
	 << (m_route_is_default ? "  (default, first click replaces it)" : "") << endl;
  m_msgs << "  Applied:      " << boolToString(m_route_applied) << endl;
  m_msgs << "  Routes Sent:  " << m_routes_posted << endl;
  m_msgs << endl;

  if(m_vx.size() == 0) {
    m_msgs << "No route. Pick \"target_path\" in the pMarineViewer mouse" << endl;
    m_msgs << "context menu and left-click the waypoints in order." << endl;
    return(true);
  }

  ACTable actab(3);
  actab << "Idx | X | Y";
  actab.addHeaderLines();
  double length = 0;
  for(unsigned int i=0; i<m_vx.size(); i++) {
    actab << uintToString(i+1)
	  << doubleToStringX(m_vx[i],1)
	  << doubleToStringX(m_vy[i],1);
    if(i > 0)
      length += hypot(m_vx[i]-m_vx[i-1], m_vy[i]-m_vy[i-1]);
  }
  m_msgs << actab.getFormattedString() << endl;
  m_msgs << endl;
  m_msgs << "Route length: " << doubleToStringX(length,1) << " m" << endl;

  return(true);
}
