/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: RegionDivider.cpp                                    */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cmath>
#include "RegionDivider.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "XYFormatUtilsPoly.h"
#include "ConvexHullGenerator.h"
#include "XYGridUpdate.h"
#include "XYFormatUtilsSegl.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

RegionDivider::RegionDivider()
{
  m_lane_width         = 20;
  m_rows               = "east-west";
  m_default_region_str = "";
  m_coverage_enabled   = true;
  m_cell_size          = 10;
  m_sensor_radius      = 15;
  m_grid_label         = "coverage";

  m_region_set        = false;
  m_deployed          = false;
  m_divisions_posted  = 0;

  // A vehicle's survey behavior starts with an invisible placeholder
  // pattern and only turns real (and visible) once our WPT_UPDATE
  // lands. That post races the DEPLOY that switches the helm into
  // SURVEYING, so repeat it a few times over the first seconds.
  m_repost_interval    = 1.5;
  m_reposts_per_deploy = 3;
  m_reposts_left       = 0;
  m_next_repost_utc    = 0;

  m_grid_ready        = false;
  m_cells_swept       = 0;
  m_geodesy_ok        = false;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RegionDivider::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key  = msg.GetKey();
    string sval = msg.GetString();

    if(key == "MISSION_POLY") {
      bool ok = setRegion(sval);
      if(!ok)
	reportRunWarning("Unable to parse MISSION_POLY: " + sval);
      else {
	// An explicit whole-polygon overrides any click-built region
	m_vertices_x.clear();
	m_vertices_y.clear();
	initCoverageGrid();
	divideAndPost();
      }
    }
    else if(key == "REGION_VERTEX")
      handleMailRegionVertex(sval);
    else if(key == "REGION_CLEAR")
      clearRegion();
    else if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if(key == "DEPLOY_ALL") {
      bool newly_deployed = (tolower(sval) == "true");
      if(newly_deployed && !m_deployed) {
	// First deploy edge: if no region has been handed to us by the
	// operator yet, fall back to the configured default so the
	// fleet always has something to search rather than sit idle.
	if(!m_region_set && (m_default_region_str != "")) {
	  if(setRegion(m_default_region_str))
	    initCoverageGrid();
	}
	divideAndPost();
	m_reposts_left    = m_reposts_per_deploy;
	m_next_repost_utc = MOOSTime() + m_repost_interval;
      }
      m_deployed = newly_deployed;
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RegionDivider::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool RegionDivider::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_coverage_enabled)
    postCoverageUpdates();

  if((m_reposts_left > 0) && (MOOSTime() >= m_next_repost_utc)) {
    divideAndPost();
    m_reposts_left--;
    m_next_repost_utc = MOOSTime() + m_repost_interval;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool RegionDivider::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // NODE_REPORTs from vehicles running pNodeReporter with
  // coord_policy_global carry LAT/LON but no X/Y, so we need the
  // datum to convert them into the local grid the region lives in.
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
      else if(param == "lane_width")
	handled = setPosDoubleOnString(m_lane_width, value);
      else if(param == "rows") {
	m_rows = value;
	handled = true;
      }
      else if(param == "default_region") {
	m_default_region_str = value;
	handled = true;
      }
      else if(param == "coverage_shading")
	handled = setBooleanOnString(m_coverage_enabled, value);
      else if(param == "cell_size")
	handled = setPosDoubleOnString(m_cell_size, value);
      else if(param == "sensor_radius")
	handled = setPosDoubleOnString(m_sensor_radius, value);
      else if(param == "grid_label")
	handled = setNonWhiteVarOnString(m_grid_label, value);
      else if(param == "repost_interval")
	handled = setPosDoubleOnString(m_repost_interval, value);
      else if(param == "reposts_per_deploy")
	handled = setUIntOnString(m_reposts_per_deploy, value);

      if(!handled)
	reportUnhandledConfigWarning(orig);
    }
  }

  if(m_vnames.size() == 0)
    reportConfigWarning("No vnames configured - nothing to divide region for.");

  registerVariables();
  return(true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void RegionDivider::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("MISSION_POLY", 0);
  Register("REGION_VERTEX", 0);
  Register("REGION_CLEAR", 0);
  Register("DEPLOY_ALL", 0);
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
}

//------------------------------------------------------------
// Procedure: setRegion()
//   Purpose: Parse an incoming polygon spec string (e.g. from
//            MISSION_POLY) into m_region. Returns false if the
//            string does not describe a valid polygon.

bool RegionDivider::setRegion(string poly_str)
{
  XYPolygon new_region = string2Poly(poly_str);
  if((new_region.size() < 3) || !new_region.is_convex())
    return(false);

  m_region = new_region;
  m_region_set = true;
  postRegionPoly();
  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailRegionVertex()
//   Purpose: Accumulate an operator mouse-click vertex. The region
//            is the convex hull of all clicks so far, so the
//            operator can click in any order. Needs three clicks
//            before a region exists.
//
//   Format: "x=val,y=val"  or  "val,val"

void RegionDivider::handleMailRegionVertex(string str)
{
  double vx = 0;
  double vy = 0;
  bool   x_set = false;
  bool   y_set = false;

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
      // Bare "x,y" form: first number is x, second is y
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

  if(!x_set || !y_set) {
    reportRunWarning("Unparsable REGION_VERTEX: " + str);
    return;
  }

  m_vertices_x.push_back(vx);
  m_vertices_y.push_back(vy);

  if(m_vertices_x.size() < 3)
    return;

  ConvexHullGenerator generator;
  for(unsigned int i=0; i<m_vertices_x.size(); i++)
    generator.addPoint(m_vertices_x[i], m_vertices_y[i]);

  XYPolygon hull = generator.generateConvexHull();
  if((hull.size() < 3) || !hull.is_convex()) {
    reportRunWarning("Clicked points do not yet form a valid region.");
    return;
  }

  m_region     = hull;
  m_region_set = true;

  postRegionPoly();
  initCoverageGrid();
  divideAndPost();
}

//------------------------------------------------------------
// Procedure: clearRegion()
//   Purpose: Drop the operator's clicked vertices and the derived
//            region/grid, so a fresh region can be drawn. The
//            region polygon and grid are erased from the GUI too.

void RegionDivider::clearRegion()
{
  m_vertices_x.clear();
  m_vertices_y.clear();

  XYPolygon dead_poly = m_region;
  dead_poly.set_label("mission_region");
  dead_poly.set_active(false);
  Notify("VIEW_POLYGON", dead_poly.get_spec());

  if(m_grid_ready) {
    XYConvexGrid dead_grid = m_grid;
    dead_grid.set_label(m_grid_label);
    dead_grid.set_active(false);
    Notify("VIEW_GRID", dead_grid.get_spec());
  }

  eraseSurveyPaths();

  m_region_set   = false;
  m_reposts_left = 0;
  m_grid_ready  = false;
  m_cells_swept = 0;
  m_grid_deltas.clear();
}

//------------------------------------------------------------
// Procedure: postRegionPoly()

void RegionDivider::postRegionPoly()
{
  if(!m_region_set)
    return;

  XYPolygon poly = m_region;
  poly.set_label("mission_region");
  poly.set_edge_color("white");
  poly.set_vertex_color("dodger_blue");
  poly.set_active(true);
  Notify("VIEW_POLYGON", poly.get_spec());

  postRegionCenter();
}

//------------------------------------------------------------
// Procedure: postRegionCenter()
//   Purpose: Publish the middle of the search region. pTargetCoordinator
//            uses it as the "inboard" direction: the USVs take station
//            on the arc between the target and this point, so the pair
//            is always between the intruder and the water it wants to
//            stay in, and the pincer pushes it outward.

void RegionDivider::postRegionCenter()
{
  if(!m_region_set)
    return;

  string spec = "x=" + doubleToStringX(m_region.get_centroid_x(),2);
  spec += ",y=" + doubleToStringX(m_region.get_centroid_y(),2);
  Notify("REGION_CENTER", spec);
}

//------------------------------------------------------------
// Procedure: initCoverageGrid()
//   Purpose: (Re)build the coverage grid over the current region.
//            Cell var "swept" goes 0 -> 1 as vehicles pass over,
//            which pMarineViewer renders as a shaded cell.

void RegionDivider::initCoverageGrid()
{
  if(!m_coverage_enabled || !m_region_set)
    return;

  vector<string> cell_vars;
  vector<double> cell_init;
  cell_vars.push_back("swept");
  cell_init.push_back(0);

  XYConvexGrid new_grid;
  bool ok = new_grid.initialize(m_region, m_cell_size, cell_vars, cell_init);
  if(!ok || (new_grid.size() == 0)) {
    reportRunWarning("Unable to build coverage grid over region.");
    m_grid_ready = false;
    return;
  }

  new_grid.setMinLimit(0, 0);
  new_grid.setMaxLimit(1, 0);
  new_grid.set_label(m_grid_label);

  m_grid = new_grid;
  m_grid_ready  = true;
  m_cells_swept = 0;
  m_grid_deltas.clear();

  postCoverageGrid();
}

//------------------------------------------------------------
// Procedure: recordPosition()
//   Purpose: Local-grid position of a node report. Prefers the X/Y
//            fields, and falls back to converting LAT/LON when the
//            reporter is configured for global coordinates only.

bool RegionDivider::recordPosition(const NodeRecord& record,
				   double& x, double& y)
{
  if(record.isSetX() && record.isSetY()) {
    x = record.getX();
    y = record.getY();
    return(true);
  }

  if(!m_geodesy_ok || !record.isSetLatitude() || !record.isSetLongitude())
    return(false);

  double nav_x, nav_y;
  m_geodesy.LatLong2LocalGrid(record.getLat(), record.getLon(), nav_y, nav_x);
  x = nav_x;
  y = nav_y;
  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Purpose: Mark every grid cell within m_sensor_radius of the
//            reporting vehicle as swept. The target vessel is a
//            contact, not a searcher, so only vehicles named in
//            m_vnames contribute coverage.

void RegionDivider::handleMailNodeReport(string str)
{
  if(!m_coverage_enabled || !m_grid_ready)
    return;

  NodeRecord record = string2NodeRecord(str);
  if(!record.valid())
    return;

  string vname = tolower(record.getName());
  bool is_searcher = false;
  for(unsigned int i=0; i<m_vnames.size(); i++) {
    if(tolower(m_vnames[i]) == vname)
      is_searcher = true;
  }
  if(!is_searcher)
    return;

  double posx, posy;
  if(!recordPosition(record, posx, posy))
    return;

  for(unsigned int ix=0; ix<m_grid.size(); ix++) {
    if(m_grid.getVal(ix, 0) > 0)
      continue;

    XYSquare cell = m_grid.getElement(ix);
    double dx = cell.getCenterX() - posx;
    double dy = cell.getCenterY() - posy;
    if(hypot(dx, dy) > m_sensor_radius)
      continue;

    m_grid.setVal(ix, 1, 0);
    m_grid_deltas[ix] = 1;
    m_cells_swept++;
  }
}

//------------------------------------------------------------
// Procedure: postCoverageGrid()

void RegionDivider::postCoverageGrid()
{
  if(!m_grid_ready)
    return;
  Notify("VIEW_GRID", m_grid.get_spec());
}

//------------------------------------------------------------
// Procedure: postCoverageUpdates()

void RegionDivider::postCoverageUpdates()
{
  if(!m_grid_ready || (m_grid_deltas.size() == 0))
    return;

  XYGridUpdate update(m_grid_label);

  map<unsigned int, double>::iterator p;
  for(p=m_grid_deltas.begin(); p!=m_grid_deltas.end(); p++)
    update.addUpdate(p->first, "swept", p->second);

  m_grid_deltas.clear();
  Notify("VIEW_GRID_DELTA", update.get_spec());
}

//------------------------------------------------------------
// Procedure: divideAndPost()
//   Purpose: Slice m_region's bounding box into m_vnames.size()
//            vertical (west->east) strips and post a lawnmower
//            WPT_UPDATE_<VNAME> for each one.

void RegionDivider::divideAndPost()
{
  unsigned int n = m_vnames.size();
  if(!m_region_set || (n == 0))
    return;

  double xmin = m_region.get_min_x();
  double xmax = m_region.get_max_x();
  double ymin = m_region.get_min_y();
  double ymax = m_region.get_max_y();

  double total_width = xmax - xmin;
  double height       = ymax - ymin;
  if((total_width <= 0) || (height <= 0))
    return;

  double strip_width = total_width / (double)(n);

  for(unsigned int i=0; i<n; i++) {
    string vname       = m_vnames[i];
    string vname_upper = toupper(vname);

    double startx  = xmin + (i * strip_width);
    double centerx = startx + (strip_width / 2.0);
    double centery = ymin + (height / 2.0);

    string spec = "polygon = format=lawnmower";
    spec += ", label="      + vname + "_survey";
    spec += ", x="          + doubleToStringX(centerx,2);
    spec += ", y="          + doubleToStringX(centery,2);
    spec += ", height="     + doubleToStringX(height,2);
    spec += ", width="      + doubleToStringX(strip_width,2);
    spec += ", lane_width=" + doubleToStringX(m_lane_width,2);
    spec += ", rows="       + m_rows;
    spec += ", startx="     + doubleToStringX(startx,2);
    spec += ", starty="     + doubleToStringX(ymin,2);

    Notify("WPT_UPDATE_" + vname_upper, spec);
  }

  m_divisions_posted++;

  // Draw the lawnmower pattern ourselves, from the same spec each
  // vehicle was just handed, rather than relying on BHV_Waypoint's own
  // drawing. BHV_Waypoint erases its segment list the instant its
  // condition goes false (e.g. the moment INTERCEPT engages), so the
  // planned sweep path would disappear from the chart every time the
  // mode changed. This copy is independent of vehicle/behavior state
  // and stays up until the operator clears or redraws the region.
  postSurveyPaths();
}

//------------------------------------------------------------
// Procedure: postSurveyPaths()
//   Purpose: Re-derive each vehicle's lawnmower pattern and post it
//            as its own persistent VIEW_SEGLIST, independent of the
//            vehicle's own helm/behavior state.

void RegionDivider::postSurveyPaths()
{
  unsigned int n = m_vnames.size();
  if(!m_region_set || (n == 0))
    return;

  double xmin = m_region.get_min_x();
  double xmax = m_region.get_max_x();
  double ymin = m_region.get_min_y();
  double ymax = m_region.get_max_y();

  double total_width = xmax - xmin;
  double height       = ymax - ymin;
  if((total_width <= 0) || (height <= 0))
    return;

  double strip_width = total_width / (double)(n);

  for(unsigned int i=0; i<n; i++) {
    string vname = m_vnames[i];

    double startx  = xmin + (i * strip_width);
    double centerx = startx + (strip_width / 2.0);
    double centery = ymin + (height / 2.0);

    string spec = "format=lawnmower";
    spec += ", x="          + doubleToStringX(centerx,2);
    spec += ", y="          + doubleToStringX(centery,2);
    spec += ", height="     + doubleToStringX(height,2);
    spec += ", width="      + doubleToStringX(strip_width,2);
    spec += ", lane_width=" + doubleToStringX(m_lane_width,2);
    spec += ", rows="       + m_rows;
    spec += ", startx="     + doubleToStringX(startx,2);
    spec += ", starty="     + doubleToStringX(ymin,2);

    XYSegList segl = string2SegList(spec);
    if(segl.size() == 0)
      continue;

    segl.set_label(vname + "_survey_path");
    segl.set_edge_color("white");
    segl.set_vertex_color("dodger_blue");
    segl.set_vertex_size(2);
    segl.set_edge_size(1);
    segl.set_active(true);
    Notify("VIEW_SEGLIST", segl.get_spec());
  }
}

//------------------------------------------------------------
// Procedure: eraseSurveyPaths()

void RegionDivider::eraseSurveyPaths()
{
  for(unsigned int i=0; i<m_vnames.size(); i++) {
    XYSegList segl;
    segl.set_label(m_vnames[i] + "_survey_path");
    segl.set_active(false);
    Notify("VIEW_SEGLIST", segl.get_spec());
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool RegionDivider::buildReport()
{
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicles:    " << stringVectorToString(m_vnames, ':') << endl;
  m_msgs << "  Lane Width:  " << doubleToStringX(m_lane_width) << endl;
  m_msgs << "  Rows:        " << m_rows << endl;
  m_msgs << "  Coverage:    " << boolToString(m_coverage_enabled)
	 << "  (cell=" << doubleToStringX(m_cell_size)
	 << ", sensor_radius=" << doubleToStringX(m_sensor_radius) << ")" << endl;
  m_msgs << endl;

  m_msgs << "State:" << endl;
  m_msgs << "  Region Set:       " << boolToString(m_region_set) << endl;
  m_msgs << "  Clicked Vertices: " << m_vertices_x.size() << endl;
  m_msgs << "  Deployed:         " << boolToString(m_deployed) << endl;
  m_msgs << "  Divisions Posted: " << m_divisions_posted << endl;
  m_msgs << "  Reposts Pending:  " << m_reposts_left << endl;
  if(m_grid_ready) {
    double pct = 0;
    if(m_grid.size() > 0)
      pct = (100.0 * (double)m_cells_swept) / (double)m_grid.size();
    m_msgs << "  Coverage:         " << m_cells_swept << "/" << m_grid.size()
	   << " cells (" << doubleToStringX(pct,1) << "%)" << endl;
  }
  m_msgs << endl;

  if(m_region_set) {
    ACTable actab(5);
    actab << "Vehicle | StripX-Min | StripX-Max | Y-Min | Y-Max";
    actab.addHeaderLines();

    double xmin = m_region.get_min_x();
    double xmax = m_region.get_max_x();
    double ymin = m_region.get_min_y();
    double ymax = m_region.get_max_y();
    unsigned int n = m_vnames.size();
    double strip_width = (n > 0) ? (xmax - xmin) / (double)(n) : 0;

    for(unsigned int i=0; i<n; i++) {
      double sxmin = xmin + (i * strip_width);
      double sxmax = sxmin + strip_width;
      actab << m_vnames[i] << doubleToStringX(sxmin,1) << doubleToStringX(sxmax,1)
	    << doubleToStringX(ymin,1) << doubleToStringX(ymax,1);
    }
    m_msgs << actab.getFormattedString();
  }
  else
    m_msgs << "No region yet. Click 3+ points (REGION_VERTEX) or send MISSION_POLY." << endl;

  return(true);
}
