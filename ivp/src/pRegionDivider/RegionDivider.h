/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: RegionDivider.h                                      */
/*    DATE: Aug 2026                                              */
/*                                                                 */
/*    pRegionDivider owns the mission search region:              */
/*      - accepts it either as a whole polygon (MISSION_POLY) or  */
/*        as operator mouse-clicks (REGION_VERTEX), in which case */
/*        the convex hull of the clicks is used;                  */
/*      - picks the sweep axis from the region's own shape (the   */
/*        long axis of its minimum-area bounding rectangle), so   */
/*        lanes run as long as the water allows and the vehicles  */
/*        make as few 180-degree turns as possible;               */
/*      - slices the region into N bands ACROSS that sweep axis,  */
/*        one per vehicle, so each vehicle keeps full-length      */
/*        lanes instead of a chopped-up set of short ones;        */
/*      - lays a search pattern into each band -- the operator    */
/*        chooses lawnmower / skip / spiral / perimeter at run    */
/*        time via SEARCH_PATTERN -- and posts it as a            */
/*        WPT_UPDATE_<VNAME> point list;                          */
/*      - sizes the lane spacing from the sensor range (visual    */
/*        range) rather than a fixed number, and pulls the lane   */
/*        ends in by the same range, so no distance is spent      */
/*        driving over water the sensor already sees;             */
/*      - tracks which parts of the region have been swept and    */
/*        shades them via a VIEW_GRID coverage grid.              */
/*****************************************************************/

#ifndef REGION_DIVIDER_HEADER
#define REGION_DIVIDER_HEADER

#include <string>
#include <vector>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "XYPolygon.h"
#include "XYSegList.h"
#include "XYConvexGrid.h"

class NodeRecord;

class RegionDivider : public AppCastingMOOSApp
{
 public:
  RegionDivider();
  virtual ~RegionDivider() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:
  bool buildReport();
  void registerVariables();

  bool  setRegion(std::string poly_str);
  bool  setPattern(std::string pattern);
  bool  setSensorRadius(std::string val);
  bool  recordPosition(const NodeRecord&, double& x, double& y);
  void  handleMailRegionVertex(std::string);
  void  handleMailNodeReport(std::string);
  void  clearRegion();
  void  divideAndPost();
  void  postRegionCenter();
  void  postSurveyPaths();
  void  eraseSurveyPaths();

  void  initCoverageGrid();
  void  postCoverageGrid();
  void  postCoverageUpdates();
  void  postRegionPoly();

 protected: // Planning
  double    laneWidth() const;
  double    sweepAngle() const;
  bool      buildPlans();
  XYSegList buildBandPath(double sweep_ang, double v_lo, double v_hi) const;
  XYSegList buildBoustrophedon(double sweep_ang, double v_lo, double v_hi,
			       bool skip) const;
  XYSegList buildSpiral(double sweep_ang, double v_lo, double v_hi,
			bool single_loop) const;
  XYSegList buildSector(double sweep_ang, double v_lo, double v_hi) const;
  XYSegList buildBarrier(double sweep_ang, double v_lo, double v_hi) const;
  XYSegList buildFigureEight(double sweep_ang, double v_lo, double v_hi) const;
  bool      laneExtent(double sweep_ang, double v, double& u_a, double& u_b,
		       bool trim_ends = true) const;
  bool      bandExtent(double sweep_ang, double v_lo, double v_hi,
		       double& u_lo, double& u_hi, bool trim_ends = true) const;
  bool      crossExtent(double sweep_ang, double u, double v_lo, double v_hi,
			double& v_a, double& v_b) const;
  bool      rayReach(double sweep_ang, double u0, double v0, double ray_ang,
		     double v_lo, double v_hi, double& reach) const;
  void      addSweepPoint(XYSegList&, double u, double v, double sweep_ang) const;
  bool      patternCoversBand() const;

 protected: // Geodesy
  CMOOSGeodesy m_geodesy;
  bool         m_geodesy_ok;

 protected: // Configuration variables
  std::vector<std::string> m_vnames;
  double                   m_lane_width;      // used when auto sizing is off
  bool                     m_auto_lane_width; // derive spacing from sensor range
  double                   m_lane_overlap;    // fraction of swath re-covered
  double                   m_endpoint_inset;  // fraction of sensor range
  std::string              m_pattern;         // see setPattern() for the list
  std::string              m_sweep_align;     // auto|north-south|east-west
  std::string              m_default_region_str;
  bool                     m_coverage_enabled;
  double                   m_cell_size;
  double                   m_sensor_radius;
  std::string              m_grid_label;
  unsigned int             m_repeat_count;   // laps of the posted path

 protected: // State variables
  XYPolygon    m_region;
  bool         m_region_set;
  bool         m_deployed;
  unsigned int m_divisions_posted;

  std::vector<double> m_vertices_x;
  std::vector<double> m_vertices_y;

  // One planned path per vehicle, index-aligned with m_vnames.
  std::vector<XYSegList> m_plans;
  std::vector<double>    m_plan_length;
  std::vector<unsigned int> m_plan_turns;
  double                 m_plan_sweep_ang;

  double       m_repost_interval;
  unsigned int m_reposts_per_deploy;
  unsigned int m_reposts_left;
  double       m_next_repost_utc;

  XYConvexGrid m_grid;
  bool         m_grid_ready;
  std::map<unsigned int, double> m_grid_deltas;
  unsigned int m_cells_swept;
};

#endif
