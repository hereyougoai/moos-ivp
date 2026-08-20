/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetCoordinator.h                                  */
/*    DATE: Aug 2026                                              */
/*                                                                 */
/*    pTargetCoordinator tracks the intruder's NODE_REPORT and,   */
/*    while intercept is active, places the configured USVs on a  */
/*    blocking arc between the intruder and the middle of the     */
/*    mission region, so the pair pinches it from both sides and  */
/*    herds it out of the area rather than queuing up in its      */
/*    wake. Stations are re-posted as TRAIL_UPDATE_<VNAME>.       */
/*****************************************************************/

#ifndef TARGET_COORDINATOR_HEADER
#define TARGET_COORDINATOR_HEADER

#include <string>
#include <vector>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "NodeRecord.h"

class TargetCoordinator : public AppCastingMOOSApp
{
 public:
  TargetCoordinator();
  virtual ~TargetCoordinator() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:
  bool buildReport();
  void registerVariables();

  void handleMailNodeReport(std::string);
  void handleMailRegionCenter(std::string);
  bool recordPosition(const NodeRecord&, double& x, double& y) const;
  void assignAndPost();
  void clearPincerVisuals();

  bool   targetPosition(double& x, double& y) const;
  bool   pincerBaseAngle(double& base_angle, std::string& basis) const;
  std::vector<double>       computeSlots(double base_angle) const;
  std::vector<unsigned int> bestAssignment(const std::vector<double>& slots) const;
  double assignmentCost(const std::vector<unsigned int>&,
			const std::vector<double>& slots) const;
  void   postPincerVisuals(const std::vector<double>& slots,
			   const std::vector<unsigned int>& order);

 protected: // Geodesy
  CMOOSGeodesy m_geodesy;
  bool         m_geodesy_ok;

 protected: // Configuration variables
  std::vector<std::string> m_vnames;
  std::string              m_target_name;
  double                   m_spread_deg;
  double                   m_trail_range;
  std::string              m_trigger_var;
  bool                     m_herd_mode;
  double                   m_swap_margin_deg;
  double                   m_center_deadzone;
  double                   m_repost_interval;
  bool                     m_draw_pincer;

 protected: // State variables
  std::map<std::string, NodeRecord> m_records;
  bool         m_intercept_active;
  unsigned int m_assignments_posted;

  double       m_center_x;
  double       m_center_y;
  bool         m_center_known;

  std::vector<unsigned int> m_assignment;
  std::vector<double>       m_posted_angle;
  double                    m_last_post_utc;
  std::string               m_basis;
  unsigned int              m_swaps;
};

#endif
