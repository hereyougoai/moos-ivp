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
  void handleMailFormation(std::string);
  bool setFormation(bool block_mode, bool herd_mode, std::string why);
  std::string formationName() const;
  void handleMailRegionCenter(std::string);
  bool recordPosition(const NodeRecord&, double& x, double& y) const;
  void assignAndPost();
  void clearPincerVisuals();

  void   assignAndPostBlock();
  bool   blockPoints(double& b1x, double& b1y,
		     double& b2x, double& b2y) const;
  bool   blockStillGood(double bx, double by) const;
  bool   blockSpent(double bx, double by) const;
  bool   blockOffTrack(double bx, double by) const;
  double usvSpeed(unsigned int vidx) const;
  double transitTime(unsigned int vidx, double bx, double by) const;
  bool   blockTransitHold(unsigned int slot) const;
  double feasibleLead() const;
  double blockOffset() const;
  double currentBlockOffset() const;
  void   updateAdaptiveOffset();
  void   resetAdaptiveOffset();
  void   postBlockVisuals();
  void   clearBlockVisuals();
  void   standDownBlock();

  bool   targetPosition(double& x, double& y) const;
  bool   targetHeading(double& heading) const;
  bool   pincerBaseAngle(double& base_angle, std::string& basis) const;
  std::vector<double>       computeSlots(double base_angle) const;
  double                    bowGuard(double slot_angle) const;
  double                    trailRange() const;
  double                    cantAngle() const;
  double                    slotCant(double slot_angle) const;
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
  double regionOffset(double x, double y) const;
  double exitDistance(double bearing) const;
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
  std::string              m_formation_var;
  bool                     m_herd_mode;
  double                   m_swap_margin_deg;

  // CANT ANGLE. How far each USV's BOW is turned IN off the parallel,
  // in degrees. See cantAngle() / slotCant().
  double                   m_cant_deg;
  double                   m_cant_dead_deg;
  double                   m_center_deadzone;
  double                   m_repost_interval;
  bool                     m_draw_pincer;

  // Frame the station bearing is ORDERED in. See assignAndPost().
  bool                     m_station_relative;

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
  double                   m_exit_commit;    // secs a challenger must hold
  bool                     m_swap_lock;      // freeze the pairing when engaged

  // Blocking stations. See assignAndPostBlock().
  bool                     m_block_mode;
  double                   m_block_lead;      // metres ahead of the intruder
  double                   m_block_offset;    // lateral offset = imposed CPA
  double                   m_block_span;      // extra offset for the denier
  double                   m_block_lead2;     // denier lead, as a fraction
  double                   m_block_slack;     // track error that voids a point
  double                   m_block_min_lead;  // point is spent once this close
  double                   m_block_interval;  // min secs between re-issues
  double                   m_block_min_offset;// hard floor under block_offset
  double                   m_block_dead_deg;  // |turn wanted| below this = none
  double                   m_block_abort;     // range at which we give way
  double                   m_block_rearm;     // extra range before blocking again

  // Adaptive offset: block_offset above is now a CEILING, not the
  // number used. See updateAdaptiveOffset().
  double                   m_block_reaction_wait; // secs to wait for a reaction
  double                   m_block_reaction_deg;  // heading change = "it reacted"
  double                   m_block_shrink_step;   // metres shaved per failed wait

  // Hysteresis on which side we stand. See assignAndPostBlock().
  double                   m_block_side_commit;   // secs a flip must hold

  // Transit feasibility. Measured: stations were being replaced 2.5-4x
  // faster than the USV could reach them. See blockTransitHold().
  bool                     m_block_commit_transit;
  double                   m_block_transit_frac;  // hold this x transit time
  double                   m_block_transit_cap;   // but never longer than this
  bool                     m_block_feasible;      // shorten lead to reachable
  double                   m_block_usv_speed;     // fallback if report has none

 protected: // State variables
  std::map<std::string, NodeRecord> m_records;
  bool         m_intercept_active;
  unsigned int m_assignments_posted;

  double       m_center_x;
  double       m_center_y;
  bool         m_center_known;

  std::vector<unsigned int> m_assignment;
  std::vector<double>       m_posted_angle;
  std::vector<double>       m_posted_cant;
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

  // Chosen eviction direction, latched for the run. See
  // updateExitDirection().
  double       m_exit_dir;
  bool         m_exit_valid;
  double       m_exit_cost;
  double       m_exit_since;            // when the current exit was set
  double       m_exit_challenge_since;  // -1 when nothing is challenging
  unsigned int m_swaps_declined;

  // Blocking-station state, one ground-fixed point per USV.
  std::vector<double>       m_block_x;
  std::vector<double>       m_block_y;
  std::vector<bool>         m_block_valid;
  std::vector<double>       m_block_utc;
  std::vector<bool>         m_block_giving_way;
  bool                      m_block_posted;
  int                       m_block_side;      // +1 starboard, -1 port
  double                    m_block_side_challenge_since; // -1 = none pending
  unsigned int              m_block_reissues;
  unsigned int              m_giveways;

  // Adaptive-offset state. See updateAdaptiveOffset().
  double                    m_block_offset_cur;   // -1 = not yet initialized
  double                    m_block_reaction_hdg;  // baseline for this offset
  double                    m_block_reaction_utc;  // when that baseline was set
  bool                      m_block_reacted;        // seen a reaction this engagement

  // Contact-driven detection: which USVs currently hold the intruder,
  // latched by the on/off transitions their contact managers report.
  std::map<std::string, bool> m_contact_held;
  std::string  m_last_suspect_report;
  unsigned int m_suspect_reports;
};

#endif
