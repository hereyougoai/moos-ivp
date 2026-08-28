/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: LandGuard.h                                          */
/*    DATE: Aug 2026                                             */
/*                                                               */
/*  Keeps a vehicle off the charted shoreline, and gets it back  */
/*  off if it ever ends up on it.                                */
/*                                                               */
/*  BHV_AvoidObstacleV24 handles the ordinary case well, but it  */
/*  has nothing to say once the vehicle is actually inside an    */
/*  obstacle: the objective function answers "which way is least */
/*  likely to hit this", which is not a question with an answer  */
/*  after the fact. The behavior falls silent, and whatever      */
/*  waypoint behavior is running takes over -- possibly steering */
/*  further inland, since its next waypoint knows nothing about  */
/*  land. Left alone the vehicle either halts where it is and    */
/*  never recovers (the halt condition cannot clear, because     */
/*  clearing it requires moving), or drives on into the beach.   */
/*                                                               */
/*  This app supplies what is missing, in three parts:           */
/*                                                               */
/*    LAND_CLEARANCE  distance to the nearest shoreline, always. */
/*    LAND_SLOW       set while that distance is small, so a     */
/*                    speed behavior can throttle back. Slower   */
/*                    is not just more warning time: running out */
/*                    of turning room IS the "unavoidable" case, */
/*                    and turning room is bought with speed.     */
/*    LAND_BREACH     set once the vehicle is on land or nearly  */
/*                    so, together with LAND_ESCAPE_PT, a point  */
/*                    a waypoint behavior can be sent to.        */
/*                                                               */
/*  The escape point is the last position at which the vehicle   */
/*  was in genuinely open water -- not the nearest water, which  */
/*  can lie across a spit and be reachable only by crossing more */
/*  land. The vehicle arrived from the breadcrumb, so it can get */
/*  back to it. For a recovery mechanism, reachability matters   */
/*  more than distance.                                          */
/*****************************************************************/

#ifndef LAND_GUARD_HEADER
#define LAND_GUARD_HEADER

#include <map>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "LandModel.h"

class LandGuard : public AppCastingMOOSApp
{
 public:
  LandGuard();
  virtual ~LandGuard() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:
  bool buildReport();
  void registerVariables();

  void updateStuck();
  void updateBreadcrumb();
  void updateSlow();
  void updateBreach();

  void postEscapePoint();
  void postVisuals();
  void eraseVisuals();

 private: // Configuration
  std::string  m_land_file;
  double       m_slow_dist;      // clearance below which LAND_SLOW is set
  double       m_slow_hyst;      // extra clearance needed to clear it again
  double       m_breach_dist;    // clearance below which LAND_BREACH is set
  double       m_crumb_dist;     // clearance a position needs to be a crumb
  double       m_crumb_spacing;  // min travel between crumbs, metres
  double       m_arrive_dist;    // how close to the escape point ends a breach

  // Deadlock breaker. Two hulls whose collision avoidance both solve to
  // "hold still" are a stable fixed point: neither moves, so the geometry
  // never changes, so neither ever moves again. Observed with three vessels
  // frozen 26 m apart for the remaining 8000 seconds of a run.
  double       m_stuck_speed;    // speed below which a vehicle counts as still
  double       m_stuck_secs;     // how long it must stay there
  double       m_unstick_secs;   // how long to hold the flag once raised
  std::string  m_stuck_var;
  std::vector<std::string> m_hold_vars;  // flags meaning "stopped on purpose"

  bool         m_post_visuals;
  std::string  m_slow_var;
  std::string  m_breach_var;

 private: // State
  LandModel    m_land;
  bool         m_land_ok;
  std::string  m_land_errmsg;

  bool         m_pos_known;
  double       m_osx;
  double       m_osy;
  double       m_clearance;

  bool         m_have_crumb;
  double       m_crumb_x;
  double       m_crumb_y;

  bool         m_slow;
  bool         m_breach;
  double       m_escape_x;
  double       m_escape_y;

  bool         m_pos_moving;
  double       m_nav_speed;
  bool         m_deployed;
  std::map<std::string, bool> m_hold_state;
  double       m_still_since;
  bool         m_stuck;
  double       m_stuck_since;
  unsigned int m_stucks;

  unsigned int m_breaches;
  double       m_breach_began;
  double       m_worst_clearance;
};

#endif
