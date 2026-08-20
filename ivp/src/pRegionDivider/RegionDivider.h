/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: RegionDivider.h                                      */
/*    DATE: Aug 2026                                              */
/*                                                                 */
/*    pRegionDivider owns the mission search region:              */
/*      - accepts it either as a whole polygon (MISSION_POLY) or  */
/*        as operator mouse-clicks (REGION_VERTEX), in which case */
/*        the convex hull of the clicks is used;                  */
/*      - slices it into N vertical (west->east) strips, one per  */
/*        vehicle in the configured vnames list, and posts a      */
/*        WPT_UPDATE_<VNAME> lawnmower survey pattern for each;   */
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

 protected: // Geodesy
  CMOOSGeodesy m_geodesy;
  bool         m_geodesy_ok;

 protected: // Configuration variables
  std::vector<std::string> m_vnames;
  double                   m_lane_width;
  std::string              m_rows;
  std::string              m_default_region_str;
  bool                     m_coverage_enabled;
  double                   m_cell_size;
  double                   m_sensor_radius;
  std::string              m_grid_label;

 protected: // State variables
  XYPolygon    m_region;
  bool         m_region_set;
  bool         m_deployed;
  unsigned int m_divisions_posted;

  std::vector<double> m_vertices_x;
  std::vector<double> m_vertices_y;

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
