/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: RegionDivider.cpp                                    */
/*    DATE: Aug 2026                                              */
/*****************************************************************/

#include <cmath>
#include <algorithm>
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
// Local geometry helpers
//
// Everything about the sweep is easier in a frame whose u axis runs
// along the lanes. worldToSweep() rotates a world point into that
// frame; sweepToWorld() brings it back.

namespace {

  void worldToSweep(double x, double y, double ang, double& u, double& v)
  {
    double c = cos(ang), s = sin(ang);
    u =  (x * c) + (y * s);
    v = -(x * s) + (y * c);
  }

  void sweepToWorld(double u, double v, double ang, double& x, double& y)
  {
    double c = cos(ang), s = sin(ang);
    x = (u * c) - (v * s);
    y = (u * s) + (v * c);
  }

  // Clip the segment (x1,y1)-(x2,y2) against a convex polygon given as
  // parallel vertex arrays, using the parametric (Cyrus-Beck) form. The
  // inside half-space of each edge is the one holding the centroid, so
  // the vertex winding direction does not matter.
  bool clipSegToConvex(const vector<double>& px, const vector<double>& py,
		       double x1, double y1, double x2, double y2,
		       double& ox1, double& oy1, double& ox2, double& oy2)
  {
    unsigned int n = px.size();
    if(n < 3)
      return(false);

    double cx = 0, cy = 0;
    for(unsigned int i=0; i<n; i++) {
      cx += px[i];
      cy += py[i];
    }
    cx /= (double)n;
    cy /= (double)n;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double t0 = 0, t1 = 1;

    for(unsigned int i=0; i<n; i++) {
      unsigned int j = (i + 1) % n;
      double ax = px[i], ay = py[i];
      double ex = px[j] - ax, ey = py[j] - ay;

      // Edge normal, flipped so it points away from the interior.
      double nx = ey, ny = -ex;
      if((((cx - ax) * nx) + ((cy - ay) * ny)) > 0) {
	nx = -nx;
	ny = -ny;
      }

      // Inside means n.(P(t) - a) <= 0, i.e. A + t*B <= 0
      double A = ((x1 - ax) * nx) + ((y1 - ay) * ny);
      double B = (dx * nx) + (dy * ny);

      if(fabs(B) < 1e-12) {
	if(A > 1e-9)
	  return(false);
	continue;
      }

      double t = -A / B;
      if(B > 0) {
	if(t < t1)
	  t1 = t;
      }
      else {
	if(t > t0)
	  t0 = t;
      }
      if(t0 > t1)
	return(false);
    }

    ox1 = x1 + (t0 * dx);
    oy1 = y1 + (t0 * dy);
    ox2 = x1 + (t1 * dx);
    oy2 = y1 + (t1 * dy);
    return(true);
  }

} // namespace

//---------------------------------------------------------
// Constructor()

RegionDivider::RegionDivider()
{
  m_lane_width         = 20;
  m_auto_lane_width    = true;
  m_lane_overlap       = 0.10;
  m_endpoint_inset     = 0.70;
  m_pattern            = "lawnmower";
  m_sweep_align        = "auto";
  m_default_region_str = "";
  m_coverage_enabled   = true;
  m_cell_size          = 10;
  m_sensor_radius      = 15;
  m_grid_label         = "coverage";

  m_region_set        = false;
  m_deployed          = false;
  m_divisions_posted  = 0;
  m_plan_sweep_ang    = 0;

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
    if(!msg.IsString())
      sval = doubleToStringX(msg.GetDouble(), 4);

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
    else if(key == "SEARCH_PATTERN") {
      if(!setPattern(sval))
	reportRunWarning("Unknown SEARCH_PATTERN: " + sval);
      else
	divideAndPost();
    }
    else if(key == "SENSOR_RADIUS") {
      if(!setSensorRadius(sval))
	reportRunWarning("Bad SENSOR_RADIUS: " + sval);
      else {
	// The sensor range sets the lane spacing, so a new range means
	// a new plan -- and the coverage shading uses the same number.
	divideAndPost();
      }
    }
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
      else if(param == "lane_width") {
	handled = setPosDoubleOnString(m_lane_width, value);
	// An explicit lane width is an explicit override.
	if(handled)
	  m_auto_lane_width = false;
      }
      else if(param == "auto_lane_width")
	handled = setBooleanOnString(m_auto_lane_width, value);
      else if(param == "lane_overlap") {
	handled = setDoubleOnString(m_lane_overlap, value);
	if(handled && ((m_lane_overlap < 0) || (m_lane_overlap > 0.9)))
	  handled = false;
      }
      else if(param == "endpoint_inset") {
	handled = setDoubleOnString(m_endpoint_inset, value);
	if(handled && ((m_endpoint_inset < 0) || (m_endpoint_inset > 1)))
	  handled = false;
      }
      else if((param == "pattern") || (param == "search_pattern"))
	handled = setPattern(value);
      else if((param == "sweep_align") || (param == "rows")) {
	// "rows" is kept for backward compatibility with the old
	// lawnmower-spec config; east-west/north-south now name the
	// lane direction rather than a lawnmower keyword.
	string val = tolower(stripBlankEnds(value));
	if((val == "auto") || (val == "east-west") || (val == "north-south")) {
	  m_sweep_align = val;
	  handled = true;
	}
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
  Register("SEARCH_PATTERN", 0);
  Register("SENSOR_RADIUS", 0);
  Register("DEPLOY_ALL", 0);
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
}

//------------------------------------------------------------
// Procedure: setPattern()
//   Purpose: Accept an operator-chosen search pattern. Kept in one
//            place so the config parameter and the run-time
//            SEARCH_PATTERN variable can never drift apart.

bool RegionDivider::setPattern(string pattern)
{
  string val = tolower(stripBlankEnds(pattern));

  // A few friendly aliases for the same four patterns.
  if((val == "lawn") || (val == "boustrophedon"))
    val = "lawnmower";
  else if((val == "skip-lane") || (val == "wide-turn"))
    val = "skip";
  else if(val == "inward-spiral")
    val = "spiral";
  else if((val == "boundary") || (val == "perim"))
    val = "perimeter";

  if((val != "lawnmower") && (val != "skip") &&
     (val != "spiral") && (val != "perimeter"))
    return(false);

  m_pattern = val;
  return(true);
}

//------------------------------------------------------------
// Procedure: setSensorRadius()

bool RegionDivider::setSensorRadius(string val)
{
  double newval;
  if(!setPosDoubleOnString(newval, stripBlankEnds(val)))
    return(false);
  if(newval < 1)
    return(false);

  m_sensor_radius = newval;
  return(true);
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
  m_plans.clear();
  m_plan_length.clear();
  m_plan_turns.clear();
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
// Procedure: laneWidth()
//   Purpose: Spacing between adjacent sweep lanes. With auto sizing
//            on this comes from the sensor's own range: a sensor of
//            radius R sees a swath 2R wide, so lanes can sit that far
//            apart, less an overlap fraction to keep a seam between
//            neighbouring lanes. Driving 20m lanes with a 15m sensor
//            -- the old fixed setting -- re-covered a third of every
//            lane and cost a third more distance for nothing.

double RegionDivider::laneWidth() const
{
  if(!m_auto_lane_width)
    return(m_lane_width);

  double width = 2.0 * m_sensor_radius * (1.0 - m_lane_overlap);
  if(width < 1)
    width = 1;
  return(width);
}

//------------------------------------------------------------
// Procedure: sweepAngle()
//   Purpose: Direction (radians, math convention) the lanes should
//            run in.
//
//            On "auto" this is the long axis of the region's
//            minimum-area bounding rectangle, found by testing the
//            frame aligned with each hull edge -- a standard rotating
//            calipers result for a convex polygon. Running the lanes
//            along the region's longest dimension gives the fewest
//            possible lanes, and therefore the fewest 180-degree turns:
//            for a long diagonal region the old fixed east-west sweep
//            produced many short lanes and a turn every few seconds.

double RegionDivider::sweepAngle() const
{
  if(m_sweep_align == "east-west")
    return(0);
  if(m_sweep_align == "north-south")
    return(M_PI / 2.0);

  unsigned int n = m_region.size();
  if(n < 3)
    return(0);

  double best_area = -1;
  double best_ang  = 0;

  for(unsigned int i=0; i<n; i++) {
    unsigned int j = (i + 1) % n;
    double ex = m_region.get_vx(j) - m_region.get_vx(i);
    double ey = m_region.get_vy(j) - m_region.get_vy(i);
    if(hypot(ex, ey) < 1e-9)
      continue;

    double ang = atan2(ey, ex);
    double umin=0, umax=0, vmin=0, vmax=0;
    for(unsigned int k=0; k<n; k++) {
      double u, v;
      worldToSweep(m_region.get_vx(k), m_region.get_vy(k), ang, u, v);
      if((k == 0) || (u < umin)) umin = u;
      if((k == 0) || (u > umax)) umax = u;
      if((k == 0) || (v < vmin)) vmin = v;
      if((k == 0) || (v > vmax)) vmax = v;
    }

    double area = (umax - umin) * (vmax - vmin);
    if((best_area < 0) || (area < best_area)) {
      best_area = area;
      // Lanes run along whichever side of the rectangle is longer.
      if((umax - umin) >= (vmax - vmin))
	best_ang = ang;
      else
	best_ang = ang + (M_PI / 2.0);
    }
  }

  return(best_ang);
}

//------------------------------------------------------------
// Procedure: laneExtent()
//   Purpose: Where a lane at cross-track offset v enters and leaves
//            the region, in sweep-frame u. Clipping against the real
//            polygon rather than its bounding box is what keeps the
//            vehicle off water that is not part of the mission.
//
//            The ends are then pulled in by the sensor range: standing
//            at the end of a lane the sensor already sees that much
//            further ahead, so driving all the way to the boundary
//            covers nothing new.

bool RegionDivider::laneExtent(double sweep_ang, double v,
			       double& u_a, double& u_b) const
{
  unsigned int n = m_region.size();
  if(n < 3)
    return(false);

  vector<double> px, py;
  double umin = 0, umax = 0;
  for(unsigned int k=0; k<n; k++) {
    double u, vv;
    worldToSweep(m_region.get_vx(k), m_region.get_vy(k), sweep_ang, u, vv);
    px.push_back(u);
    py.push_back(vv);
    if((k == 0) || (u < umin)) umin = u;
    if((k == 0) || (u > umax)) umax = u;
  }

  double pad = 10 + (umax - umin);
  double x1, y1, x2, y2;
  bool ok = clipSegToConvex(px, py, umin - pad, v, umax + pad, v, x1, y1, x2, y2);
  if(!ok)
    return(false);

  u_a = (x1 < x2) ? x1 : x2;
  u_b = (x1 < x2) ? x2 : x1;

  double inset = m_endpoint_inset * m_sensor_radius;
  double length = u_b - u_a;
  if(length <= 0)
    return(false);

  if(length > (2.5 * inset)) {
    u_a += inset;
    u_b -= inset;
  }
  else {
    // Too short to trim: shrink toward the middle but keep a segment
    // the helm can actually steer along.
    double mid = (u_a + u_b) / 2.0;
    double half = length / 4.0;
    if(half < 1)
      half = (length / 2.0);
    u_a = mid - half;
    u_b = mid + half;
  }

  return((u_b - u_a) > 0.5);
}

//------------------------------------------------------------
// Procedure: buildBoustrophedon()
//   Purpose: Classic back-and-forth sweep of one band.
//
//            skip=false: lanes in adjacent order. Shortest total
//              distance, but every turn is a 180 across one lane
//              width -- tighter than most hulls can hold, so the
//              vehicle overshoots and saws the rudder.
//            skip=true: every other lane on the way out, the ones
//              missed on the way back. The turn is then across two
//              lane widths, i.e. twice the turning circle, which is
//              what "fewer hard turns" actually means at the helm.

XYSegList RegionDivider::buildBoustrophedon(double sweep_ang,
					    double v_lo, double v_hi,
					    bool skip) const
{
  XYSegList segl;

  double lw = laneWidth();
  double band = v_hi - v_lo;
  if((band <= 0) || (lw <= 0))
    return(segl);

  // Round the lane count UP. Rounding down leaves a sliver of the band
  // narrower than one lane completely unswept along the region edge and
  // along the seam with the neighbouring vehicle's band -- visible on
  // the coverage grid as two unshaded strips that never fill in.
  // Rounding up costs at most one extra lane and, since the sensor sees
  // half a lane width to either side, guarantees the band is covered.
  unsigned int lanes = (unsigned int)(ceil((band / lw) - 1e-9));
  if(lanes < 1)
    lanes = 1;

  // Center the lane stack in the band so the outer margins are even
  // rather than all the slack piling up against one edge.
  double used = lanes * lw;
  double base = v_lo + ((band - used) / 2.0) + (lw / 2.0);

  vector<unsigned int> order;
  if(!skip) {
    for(unsigned int i=0; i<lanes; i++)
      order.push_back(i);
  }
  else {
    for(unsigned int i=0; i<lanes; i+=2)
      order.push_back(i);
    // Come back down the lanes that were skipped.
    unsigned int start = (lanes >= 2) ? (((lanes - 1) % 2) ? (lanes - 1) : (lanes - 2)) : 0;
    for(int i=(int)start; i>=1; i-=2)
      order.push_back((unsigned int)i);
  }

  bool forward = true;
  for(unsigned int i=0; i<order.size(); i++) {
    double v = base + (order[i] * lw);
    double u_a, u_b;
    if(!laneExtent(sweep_ang, v, u_a, u_b))
      continue;

    double first = forward ? u_a : u_b;
    double last  = forward ? u_b : u_a;

    double x, y;
    sweepToWorld(first, v, sweep_ang, x, y);
    segl.add_vertex(x, y);
    sweepToWorld(last, v, sweep_ang, x, y);
    segl.add_vertex(x, y);

    forward = !forward;
  }

  return(segl);
}

//------------------------------------------------------------
// Procedure: buildSpiral()
//   Purpose: Inward rectangular spiral over the band (single_loop
//            true gives just the outer ring, i.e. a perimeter patrol).
//
//            Corners that fall outside the region -- the band is a
//            slab of the bounding rectangle, the region may be
//            narrower there -- are pulled back onto the boundary
//            rather than dropped, so the loop stays closed.

XYSegList RegionDivider::buildSpiral(double sweep_ang,
				     double v_lo, double v_hi,
				     bool single_loop) const
{
  XYSegList segl;

  double lw = laneWidth();
  if(lw <= 0)
    return(segl);

  // Outer extent of the band along the lane axis, taken at the widest
  // lane in the band so the ring encloses as much of it as possible.
  double u_lo = 0, u_hi = 0;
  bool have_extent = false;
  for(double v=v_lo; v<=v_hi+1e-9; v+=(v_hi-v_lo)/8.0) {
    double a, b;
    if(!laneExtent(sweep_ang, v, a, b))
      continue;
    if(!have_extent || (a < u_lo)) u_lo = a;
    if(!have_extent || (b > u_hi)) u_hi = b;
    have_extent = true;
    if((v_hi - v_lo) <= 0)
      break;
  }
  if(!have_extent)
    return(segl);

  double a0 = u_lo, a1 = u_hi;
  double b0 = v_lo + (lw / 2.0), b1 = v_hi - (lw / 2.0);
  if(b0 > b1) {
    double mid = (v_lo + v_hi) / 2.0;
    b0 = b1 = mid;
  }

  unsigned int guard = 0;
  while((a0 <= a1) && (b0 <= b1) && (guard < 200)) {
    guard++;

    double corners[4][2] = {{a0,b0},{a1,b0},{a1,b1},{a0,b1}};
    for(unsigned int c=0; c<4; c++) {
      double x, y;
      sweepToWorld(corners[c][0], corners[c][1], sweep_ang, x, y);
      if(!m_region.contains(x, y)) {
	double rx, ry;
	if(m_region.closest_point_on_poly(x, y, rx, ry)) {
	  x = rx;
	  y = ry;
	}
      }
      segl.add_vertex(x, y);
    }

    if(single_loop)
      break;

    a0 += lw;
    a1 -= lw;
    b0 += lw;
    b1 -= lw;

    // Re-enter the next ring from the corner we just left.
    if((a0 <= a1) && (b0 <= b1)) {
      double x, y;
      sweepToWorld(a0, b0, sweep_ang, x, y);
      if(!m_region.contains(x, y)) {
	double rx, ry;
	if(m_region.closest_point_on_poly(x, y, rx, ry)) {
	  x = rx;
	  y = ry;
	}
      }
      segl.add_vertex(x, y);
    }
  }

  return(segl);
}

//------------------------------------------------------------
// Procedure: buildBandPath()

XYSegList RegionDivider::buildBandPath(double sweep_ang,
				       double v_lo, double v_hi) const
{
  if(m_pattern == "skip")
    return(buildBoustrophedon(sweep_ang, v_lo, v_hi, true));
  if(m_pattern == "spiral")
    return(buildSpiral(sweep_ang, v_lo, v_hi, false));
  if(m_pattern == "perimeter")
    return(buildSpiral(sweep_ang, v_lo, v_hi, true));

  return(buildBoustrophedon(sweep_ang, v_lo, v_hi, false));
}

//------------------------------------------------------------
// Procedure: buildPlans()
//   Purpose: Cut the region into one band per vehicle and lay the
//            chosen pattern into each.
//
//            The cut runs ACROSS the sweep axis, not along it. Split
//            the other way and every vehicle's lanes get chopped in
//            half, doubling the number of end-of-lane turns for the
//            same water covered. Cutting across leaves every lane
//            full length, so the fleet as a whole makes exactly the
//            same number of turns a single vehicle would.

bool RegionDivider::buildPlans()
{
  m_plans.clear();
  m_plan_length.clear();
  m_plan_turns.clear();

  unsigned int n = m_vnames.size();
  if(!m_region_set || (n == 0) || (m_region.size() < 3))
    return(false);

  double sweep_ang = sweepAngle();
  m_plan_sweep_ang = sweep_ang;

  double vmin = 0, vmax = 0;
  for(unsigned int k=0; k<m_region.size(); k++) {
    double u, v;
    worldToSweep(m_region.get_vx(k), m_region.get_vy(k), sweep_ang, u, v);
    if((k == 0) || (v < vmin)) vmin = v;
    if((k == 0) || (v > vmax)) vmax = v;
  }
  if((vmax - vmin) <= 0)
    return(false);

  double band = (vmax - vmin) / (double)n;

  for(unsigned int i=0; i<n; i++) {
    double v_lo = vmin + (i * band);
    double v_hi = v_lo + band;

    XYSegList segl = buildBandPath(sweep_ang, v_lo, v_hi);

    double length = 0;
    unsigned int turns = 0;
    for(unsigned int k=1; k<segl.size(); k++) {
      length += hypot(segl.get_vx(k) - segl.get_vx(k-1),
		      segl.get_vy(k) - segl.get_vy(k-1));
      if(k >= 2)
	turns++;
    }

    m_plans.push_back(segl);
    m_plan_length.push_back(length);
    m_plan_turns.push_back(turns);
  }

  return(true);
}

//------------------------------------------------------------
// Procedure: divideAndPost()
//   Purpose: Re-plan and hand each vehicle its own path.

void RegionDivider::divideAndPost()
{
  if(!buildPlans())
    return;

  for(unsigned int i=0; i<m_vnames.size(); i++) {
    if(m_plans[i].size() < 2)
      continue;

    string vname_upper = toupper(m_vnames[i]);

    XYSegList segl = m_plans[i];
    segl.set_label(m_vnames[i] + "_survey");

    // An explicit point list rather than a lawnmower spec: the pattern
    // is ours to shape (skip order, spiral, clipped lane ends), none of
    // which the built-in lawnmower generator can express.
    string spec = "points = " + segl.get_spec_pts_label();

    Notify("WPT_UPDATE_" + vname_upper, spec);
  }

  m_divisions_posted++;

  // Draw the pattern ourselves, from the same points each vehicle was
  // just handed, rather than relying on BHV_Waypoint's own drawing.
  // BHV_Waypoint erases its segment list the instant its condition goes
  // false (e.g. the moment INTERCEPT engages), so the planned sweep
  // path would disappear from the chart every time the mode changed.
  // This copy is independent of vehicle/behavior state and stays up
  // until the operator clears or redraws the region.
  postSurveyPaths();
}

//------------------------------------------------------------
// Procedure: postSurveyPaths()
//   Purpose: Post each vehicle's planned path as its own persistent
//            VIEW_SEGLIST, independent of the vehicle's own
//            helm/behavior state.

void RegionDivider::postSurveyPaths()
{
  for(unsigned int i=0; i<m_plans.size(); i++) {
    if(i >= m_vnames.size())
      break;
    if(m_plans[i].size() == 0)
      continue;

    XYSegList segl = m_plans[i];
    segl.set_label(m_vnames[i] + "_survey_path");
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
  double lw = laneWidth();

  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicles:      " << stringVectorToString(m_vnames, ':') << endl;
  m_msgs << "  Pattern:       " << m_pattern
	 << "   (SEARCH_PATTERN: lawnmower|skip|spiral|perimeter)" << endl;
  m_msgs << "  Sensor Range:  " << doubleToStringX(m_sensor_radius,1)
	 << " m   (SENSOR_RADIUS)" << endl;
  m_msgs << "  Lane Width:    " << doubleToStringX(lw,1) << " m"
	 << (m_auto_lane_width ? "  (auto: 2*range*(1-overlap))" : "  (fixed)")
	 << endl;
  m_msgs << "  Lane Overlap:  " << doubleToStringX(m_lane_overlap*100,0) << "%" << endl;
  m_msgs << "  End Inset:     " << doubleToStringX(m_endpoint_inset*m_sensor_radius,1)
	 << " m per lane end" << endl;
  m_msgs << "  Sweep Align:   " << m_sweep_align;
  if(m_region_set)
    m_msgs << "  (lanes bearing " << doubleToStringX(fmod(90 - (m_plan_sweep_ang*180.0/M_PI) + 360, 180),1) << " deg)";
  m_msgs << endl;
  m_msgs << "  Coverage:      " << boolToString(m_coverage_enabled)
	 << "  (cell=" << doubleToStringX(m_cell_size) << ")" << endl;
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

  if(m_region_set && (m_plans.size() > 0)) {
    ACTable actab(4);
    actab << "Vehicle | Waypoints | Path Length (m) | Turns";
    actab.addHeaderLines();
    for(unsigned int i=0; i<m_plans.size(); i++) {
      actab << m_vnames[i]
	    << uintToString(m_plans[i].size())
	    << doubleToStringX(m_plan_length[i],1)
	    << uintToString(m_plan_turns[i]);
    }
    m_msgs << actab.getFormattedString();
  }
  else
    m_msgs << "No region yet. Click 3+ points (REGION_VERTEX) or send MISSION_POLY." << endl;

  return(true);
}
