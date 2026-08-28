/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: LandGuard.cpp                                        */
/*****************************************************************/

#include <cmath>
#include <iterator>
#include "LandGuard.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPoint.h"
#include "XYSegList.h"

using namespace std;

//---------------------------------------------------------
// Constructor

LandGuard::LandGuard()
{
  m_land_file     = "land.txt";

  // Slow down inside 25 m. That is comfortably outside the avoidance
  // behavior's pwt_outer_dist (30 m) minus its reaction, so the throttle
  // comes off at roughly the moment the turn starts rather than after it.
  m_slow_dist     = 25;

  // Hysteresis, or the vehicle chatters between speeds while running a
  // lane parallel to the shore at exactly the trigger distance.
  m_slow_hyst     = 8;

  // A breach is declared before contact, not after. By the time the hull is
  // inside a tile the avoidance behavior has already gone quiet, so waiting
  // for that gives recovery nothing to work with. 3 m is inside the 4 m the
  // shoreline was grown seaward when the tiles were generated, so this fires
  // while the vehicle is still, strictly, in water.
  m_breach_dist   = 3;

  // A breadcrumb has to be somewhere worth returning to: far enough off the
  // beach that arriving there actually ends the breach.
  m_crumb_dist    = 20;
  m_crumb_spacing = 10;
  m_arrive_dist   = 8;

  // Deadlock breaker defaults. 20 s is long enough that ordinary manoeuvring
  // -- a waypoint turn, a give-way slow-down -- is never mistaken for a
  // deadlock, and short enough that an operator does not sit watching a
  // frozen screen wondering what broke.
  m_stuck_speed   = 0.05;
  m_stuck_secs    = 20;
  m_unstick_secs  = 12;
  m_stuck_var     = "STUCK";
  m_hold_vars.push_back("STATION_KEEP");
  m_hold_vars.push_back("TGT_HOLD");

  m_post_visuals  = true;
  m_slow_var      = "LAND_SLOW";
  m_breach_var    = "LAND_BREACH";

  m_land_ok       = false;
  m_pos_known     = false;
  m_osx           = 0;
  m_osy           = 0;
  m_clearance     = -1;

  m_have_crumb    = false;
  m_crumb_x       = 0;
  m_crumb_y       = 0;

  m_slow          = false;
  m_breach        = false;
  m_escape_x      = 0;
  m_escape_y      = 0;

  m_nav_speed     = 0;
  m_deployed      = false;
  m_still_since   = -1;
  m_stuck         = false;
  m_stuck_since   = 0;
  m_stucks        = 0;

  m_breaches      = 0;
  m_breach_began  = 0;
  m_worst_clearance = -1;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool LandGuard::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X") {
      m_osx = msg.GetDouble();
      m_pos_known = true;
    }
    else if(key == "NAV_Y") {
      m_osy = msg.GetDouble();
      m_pos_known = true;
    }
    else if(key == "NAV_SPEED")
      m_nav_speed = msg.GetDouble();
    else if(key == "DEPLOY")
      m_deployed = (tolower(msg.GetString()) == "true");
    else if(m_hold_state.count(key))
      m_hold_state[key] = (tolower(msg.GetString()) == "true");
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool LandGuard::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool LandGuard::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Deliberately ahead of the land check, and not conditioned on it: a
  // mutual-avoidance deadlock has nothing to do with the shoreline and must
  // still be broken on a mission with no land loaded at all.
  updateStuck();

  if(!m_land.active() || !m_pos_known) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  m_clearance = m_land.distToLand(m_osx, m_osy);
  Notify("LAND_CLEARANCE", m_clearance);

  if((m_worst_clearance < 0) || (m_clearance < m_worst_clearance))
    m_worst_clearance = m_clearance;

  updateBreadcrumb();
  updateSlow();
  updateBreach();

  if(m_post_visuals)
    postVisuals();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: updateStuck()
//   Purpose: Notice a vehicle that has stopped and is not going to restart.
//
//            Two hulls whose collision avoidance both solve to "hold still"
//            form a stable fixed point: neither moves, so the geometry never
//            changes, so neither ever moves again. It is not an error and
//            nothing reports it -- the helm is solving happily, and the
//            answer it keeps arriving at is zero. Observed with three
//            vessels frozen at 26 and 36 m for the remaining 8000 seconds of
//            a run, after one closed to 4.8 m of the intruder while
//            trailing it.
//
//            Breaking it takes almost nothing: any motion at all changes the
//            geometry and the fixed point is gone. So this only raises a
//            flag, and a modest constant-speed behavior in the .bhv file
//            supplies a floor under the solved speed. Heading still comes
//            from the avoidance behaviors, so the vehicle leaves along the
//            least-bad course rather than being steered blind.
//
//            Vehicles that are stopped on purpose are exempt: not deployed,
//            or holding station. Those are answers, not deadlocks.

void LandGuard::updateStuck()
{
  bool holding = false;
  std::map<std::string, bool>::const_iterator h;
  for(h=m_hold_state.begin(); h!=m_hold_state.end(); h++)
    holding = holding || h->second;

  bool exempt = (!m_deployed || holding);
  bool still  = (m_nav_speed < m_stuck_speed);

  if(exempt || !still) {
    m_still_since = -1;
    if(m_stuck && !still) {
      // Moving again. Drop the flag as soon as that is true, rather than
      // holding it for the full unstick window: the floor has done its job
      // and leaving it in place would keep overriding a solution that is
      // now perfectly sound.
      m_stuck = false;
      Notify(m_stuck_var, "false");
      reportEvent("UNSTUCK: making way again at " +
                  doubleToStringX(m_nav_speed,2) + " m/s");
    }
    else if(m_stuck && exempt) {
      m_stuck = false;
      Notify(m_stuck_var, "false");
    }
    return;
  }

  if(m_still_since < 0)
    m_still_since = MOOSTime();

  if(!m_stuck) {
    if((MOOSTime() - m_still_since) < m_stuck_secs)
      return;
    m_stuck = true;
    m_stucks++;
    m_stuck_since = MOOSTime();
    Notify(m_stuck_var, "true");
    reportEvent("STUCK: no way on for " +
                doubleToStringX(m_stuck_secs,0) + "s - forcing a speed floor");
    return;
  }

  // Already flagged and still not moving. Give up on this attempt and let it
  // re-arm, so a vehicle that is genuinely pinned keeps being nudged rather
  // than holding one flag forever.
  if((MOOSTime() - m_stuck_since) > m_unstick_secs) {
    m_stuck       = false;
    m_still_since = -1;
    Notify(m_stuck_var, "false");
    reportEvent("unstick attempt expired - re-arming");
  }
}

//---------------------------------------------------------
// Procedure: updateBreadcrumb()
//   Purpose: Remember the last position that was safely in open water.
//
//            Only positions with real clearance qualify. A crumb dropped
//            just off the beach would be a destination that does not end
//            the breach on arrival, which would leave recovery cycling.

void LandGuard::updateBreadcrumb()
{
  if(m_clearance < m_crumb_dist)
    return;

  // While a breach is in progress the trail is frozen. The vehicle is
  // being driven back along it; overwriting the destination with wherever
  // it currently is would make it chase itself.
  if(m_breach)
    return;

  if(m_have_crumb) {
    double moved = hypot(m_osx - m_crumb_x, m_osy - m_crumb_y);
    if(moved < m_crumb_spacing)
      return;
  }

  m_crumb_x    = m_osx;
  m_crumb_y    = m_osy;
  m_have_crumb = true;

  // Keep the escape waypoint loaded at all times, not only once a breach is
  // declared. The recovery behavior consumes this through UPDATES, which it
  // does whether running or idle, so publishing it continuously means the
  // destination is already in place the instant the condition flips. Posting
  // it only at breach time would leave one helm iteration in which the
  // behavior is active with no waypoint -- pointed at whatever stale value
  // it held, which is the one moment it must not be.
  m_escape_x = m_crumb_x;
  m_escape_y = m_crumb_y;
  postEscapePoint();
}

//---------------------------------------------------------
// Procedure: updateSlow()

void LandGuard::updateSlow()
{
  bool want_slow = m_slow;

  if(!m_slow && (m_clearance < m_slow_dist))
    want_slow = true;
  else if(m_slow && (m_clearance > (m_slow_dist + m_slow_hyst)))
    want_slow = false;

  if(want_slow == m_slow)
    return;

  m_slow = want_slow;
  Notify(m_slow_var, boolToString(m_slow));
  reportEvent(m_slow_var + "=" + boolToString(m_slow) +
	      " (clearance " + doubleToStringX(m_clearance,1) + "m)");
}

//---------------------------------------------------------
// Procedure: updateBreach()
//   Purpose: Declare and clear the recovery state.
//
//            Clearing is deliberately not the inverse of declaring. The
//            breach ends when the vehicle has reached the escape point, or
//            has otherwise recovered real clearance -- not the moment it
//            creeps back over the breach distance, which on a shoreline it
//            can do while still pointed at the beach.

void LandGuard::updateBreach()
{
  if(!m_breach) {
    if(m_clearance >= m_breach_dist)
      return;
    if(!m_have_crumb) {
      // Nothing to steer back to. Say so rather than posting a breach that
      // no behavior can act on.
      reportRunWarning("Breached with no open-water position recorded - "
		       "no escape point can be offered.");
      return;
    }

    m_breach   = true;
    m_breaches++;
    m_breach_began = MOOSTime();

    // m_escape_x/y already hold the last crumb, published as it was laid.
    // updateBreadcrumb() stops updating them for the duration of the breach,
    // so the destination stays put while the vehicle drives to it.
    Notify(m_breach_var, "true");
    reportEvent("BREACH: clearance " + doubleToStringX(m_clearance,1) +
		"m - escaping to " + doubleToStringX(m_escape_x,1) + "," +
		doubleToStringX(m_escape_y,1));
    return;
  }

  double to_escape = hypot(m_osx - m_escape_x, m_osy - m_escape_y);
  bool   arrived   = (to_escape < m_arrive_dist);
  bool   recovered = (m_clearance >= m_crumb_dist);

  if(!arrived && !recovered) {
    // Re-post periodically: the waypoint behavior spawns on the flag, and a
    // single post can land before it is listening.
    postEscapePoint();
    return;
  }

  m_breach = false;
  Notify(m_breach_var, "false");
  reportEvent("RECOVERED after " +
	      doubleToStringX(MOOSTime() - m_breach_began, 1) + "s, clearance " +
	      doubleToStringX(m_clearance,1) + "m");
}

//---------------------------------------------------------
// Procedure: postEscapePoint()

void LandGuard::postEscapePoint()
{
  Notify("LAND_ESCAPE_PT", "x=" + doubleToStringX(m_escape_x,1) +
	 ",y=" + doubleToStringX(m_escape_y,1));

  // The waypoint behavior is templated on this update, in the form its
  // "points" parameter expects.
  Notify("LAND_ESCAPE_UPDATE", "points = " +
	 doubleToStringX(m_escape_x,1) + "," +
	 doubleToStringX(m_escape_y,1));
}

//---------------------------------------------------------
// Procedure: postVisuals()

void LandGuard::postVisuals()
{
  if(!m_have_crumb)
    return;

  XYPoint pt(m_crumb_x, m_crumb_y);
  pt.set_label("landguard_" + m_host_community);
  pt.set_label_color("invisible");
  pt.set_vertex_size(m_breach ? 10 : 4);
  pt.set_color("vertex", m_breach ? "red" : "green");
  pt.set_duration(10);
  Notify("VIEW_POINT", pt.get_spec());

  if(!m_breach)
    return;

  XYSegList segl;
  segl.add_vertex(m_osx, m_osy);
  segl.add_vertex(m_escape_x, m_escape_y);
  segl.set_label("landesc_" + m_host_community);
  segl.set_edge_color("red");
  segl.set_vertex_color("invisible");
  segl.set_duration(10);
  Notify("VIEW_SEGLIST", segl.get_spec());
}

//---------------------------------------------------------
// Procedure: eraseVisuals()

void LandGuard::eraseVisuals()
{
  XYPoint pt(0, 0);
  pt.set_label("landguard_" + m_host_community);
  Notify("VIEW_POINT", pt.get_spec_inactive());
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool LandGuard::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "land_file") {
      m_land_file = value;
      handled = true;
    }
    else if(param == "slow_dist")
      handled = setNonNegDoubleOnString(m_slow_dist, value);
    else if(param == "slow_hysteresis")
      handled = setNonNegDoubleOnString(m_slow_hyst, value);
    else if(param == "breach_dist")
      handled = setNonNegDoubleOnString(m_breach_dist, value);
    else if(param == "crumb_dist")
      handled = setNonNegDoubleOnString(m_crumb_dist, value);
    else if(param == "crumb_spacing")
      handled = setPosDoubleOnString(m_crumb_spacing, value);
    else if(param == "arrive_dist")
      handled = setPosDoubleOnString(m_arrive_dist, value);
    else if(param == "stuck_speed")
      handled = setNonNegDoubleOnString(m_stuck_speed, value);
    else if(param == "stuck_secs")
      handled = setPosDoubleOnString(m_stuck_secs, value);
    else if(param == "unstick_secs")
      handled = setPosDoubleOnString(m_unstick_secs, value);
    else if(param == "stuck_var")
      handled = setNonWhiteVarOnString(m_stuck_var, value);
    else if(param == "hold_var") {
      m_hold_vars.push_back(stripBlankEnds(value));
      handled = true;
    }
    else if(param == "post_visuals")
      handled = setBooleanOnString(m_post_visuals, value);
    else if(param == "slow_var")
      handled = setNonWhiteVarOnString(m_slow_var, value);
    else if(param == "breach_var")
      handled = setNonWhiteVarOnString(m_breach_var, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // A crumb has to be a place where arriving actually ends the breach.
  if(m_crumb_dist <= m_breach_dist) {
    reportConfigWarning("crumb_dist must exceed breach_dist; raising it.");
    m_crumb_dist = m_breach_dist + 10;
  }

  m_land_ok = m_land.loadFile(m_land_file, m_land_errmsg);
  if(!m_land_ok)
    reportConfigWarning("Land: " + m_land_errmsg + " - guard is inert.");
  else if(!m_land.active())
    reportConfigWarning("Land: no land masses in " + m_land_file +
			" - guard is inert.");

  // Publish the initial states, so a behavior conditioned on either flag has
  // a defined value to test rather than waiting for the first transition.
  for(unsigned int i=0; i<m_hold_vars.size(); i++)
    m_hold_state[m_hold_vars[i]] = false;

  Notify(m_slow_var, "false");
  Notify(m_breach_var, "false");
  Notify(m_stuck_var, "false");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void LandGuard::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_SPEED", 0);
  Register("DEPLOY", 0);
  for(unsigned int i=0; i<m_hold_vars.size(); i++)
    Register(m_hold_vars[i], 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool LandGuard::buildReport()
{
  m_msgs << "Stuck guard: speed " << doubleToStringX(m_nav_speed,2)
	 << " m/s, deployed " << boolToString(m_deployed)
	 << ", flag " << boolToString(m_stuck)
	 << ", fired " << m_stucks << " time(s)" << endl;
  m_msgs << "             (< " << doubleToStringX(m_stuck_speed,2)
	 << " m/s for " << doubleToStringX(m_stuck_secs,0) << "s trips it)"
	 << endl;
  m_msgs << endl;

  if(!m_land.active()) {
    m_msgs << "Land INERT: " << (m_land_ok ? "no land masses loaded"
			    : m_land_errmsg) << endl;
    return(true);
  }

  m_msgs << "Land:       " << m_land.getSummary() << endl;
  m_msgs << "Position:   ";
  if(!m_pos_known)
    m_msgs << "unknown (no NAV_X/NAV_Y yet)" << endl;
  else
    m_msgs << doubleToStringX(m_osx,1) << "," << doubleToStringX(m_osy,1)
	   << endl;
  m_msgs << endl;

  m_msgs << "Clearance:  " << doubleToStringX(m_clearance,1) << " m"
	 << "   (worst this run: " << doubleToStringX(m_worst_clearance,1)
	 << " m)" << endl;
  m_msgs << endl;

  ACTable actab(4);
  actab << "Layer | Trigger | State | Detail";
  actab.addHeaderLines();

  actab << "slow"
	<< "< " + doubleToStringX(m_slow_dist,1) + " m"
	<< boolToString(m_slow)
	<< "clears at " + doubleToStringX(m_slow_dist + m_slow_hyst,1) + " m";

  string escape = "-";
  if(m_breach)
    escape = doubleToStringX(m_escape_x,1) + "," + doubleToStringX(m_escape_y,1);
  actab << "breach"
	<< "< " + doubleToStringX(m_breach_dist,1) + " m"
	<< boolToString(m_breach)
	<< escape;

  string crumb = "none yet";
  if(m_have_crumb)
    crumb = doubleToStringX(m_crumb_x,1) + "," + doubleToStringX(m_crumb_y,1);
  actab << "crumb"
	<< ">= " + doubleToStringX(m_crumb_dist,1) + " m"
	<< boolToString(m_have_crumb)
	<< crumb;

  m_msgs << actab.getFormattedString() << endl;
  m_msgs << endl;
  m_msgs << "Breaches:   " << m_breaches << endl;

  // No breadcrumb means no recovery is possible, which is worth saying
  // loudly rather than leaving as a "false" in a table.
  if(!m_have_crumb)
    m_msgs << "  !! No open-water position recorded yet - a breach now "
	   << "could not be recovered." << endl;

  return(true);
}
