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
/*                                                                 */
/*    It also runs the engagement cycle itself: an intruder that   */
/*    comes inside the USVs' detection range while they are        */
/*    sweeping is picked up automatically, and once it has been    */
/*    pushed clear of the search area (or clear of every USV's     */
/*    detection range) for a sustained period the eviction counts  */
/*    as a success -- the alert drops, the USVs go back to their   */
/*    search pattern, and detection re-arms for the next run.      */
/*****************************************************************/

#ifndef TARGET_COORDINATOR_HEADER
#define TARGET_COORDINATOR_HEADER

#include <string>
#include <vector>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "NodeRecord.h"
#include "XYPolygon.h"

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
  double                    bowGuard(double slot_angle) const;
  double                    trailRange() const;
  std::vector<unsigned int> bestAssignment(const std::vector<double>& slots) const;
  double assignmentCost(const std::vector<unsigned int>&,
			const std::vector<double>& slots) const;
  void   postPincerVisuals(const std::vector<double>& slots,
			   const std::vector<unsigned int>& order);

 protected: // Automatic engagement cycle
  void   handleMailRegionPoly(std::string);
  void   handleMailSuspectReport(std::string);
  bool   contactHeld() const;
  void   updateExitDirection();
  double exitCost(double bearing, double heading) const;
  void   updateEngagement();
  bool   nearestRangeToTarget(double& range, std::string& vname) const;
  bool   targetClearOfRegion(double buffer) const;
  double detectRange() const;
  void   postAlert(bool active);
  void   postDetectRings();
  void   eraseDetectRings();

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

  bool                     m_auto_engage;
  double                   m_detect_range;     // 0 => from sensor radius
  double                   m_detect_scale;     // multiplier on sensor radius
  double                   m_sensor_radius;
  double                   m_release_hold;     // secs clear before success
  double                   m_region_exit_buffer;
  double                   m_reengage_delay;
  std::string              m_alert_var;
  std::string              m_survey_var;
  bool                     m_draw_detect_rings;
  bool                     m_operator_release;
  double                   m_bow_guard_deg;     // 0 disables
  double                   m_min_standoff;      // floor under trail_range
  double                   m_heading_bias;   // cost metres per degree off head
  double                   m_exit_margin;    // cost a new exit must beat by

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

  bool         m_engaged;       // intercept is running
  std::string  m_engage_source;  // "auto" | "operator"
  bool         m_had_contact;    // target was ever contested this run
  bool         m_deployed;
  double       m_clear_since;    // -1 when the target is not clear
  double       m_rearm_utc;      // detection is deaf until this time
  unsigned int m_detections;
  unsigned int m_evictions;
  double       m_last_range;
  std::string  m_closest_vname;
  bool         m_range_known;
  XYPolygon    m_region;
  bool         m_region_known;
  bool         m_rings_drawn;
  double       m_last_ring_utc;

  // Chosen eviction direction, with hysteresis.
  double       m_exit_dir;
  bool         m_exit_valid;
  double       m_exit_cost;

  // Contact-driven detection: which USVs currently hold the intruder,
  // latched by the on/off transitions their contact managers report.
  std::map<std::string, bool> m_contact_held;
  std::string  m_last_suspect_report;
  unsigned int m_suspect_reports;
};

#endif
