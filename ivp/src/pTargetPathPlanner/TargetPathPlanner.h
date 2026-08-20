/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetPathPlanner.h                                  */
/*    DATE: Aug 2026                                              */
/*                                                                 */
/*    pTargetPathPlanner lets the operator draw the intruder's    */
/*    route instead of it being frozen into the launch script.    */
/*                                                                 */
/*    It runs in the target vessel's own community, collects the  */
/*    operator's mouse clicks (TARGET_PATH_VERTEX, forwarded from */
/*    the shoreside) in the order they were made, and hands the   */
/*    resulting point list to the target's patrol behavior as a   */
/*    TGT_WPT_UPDATE. Unlike the search region, order matters and */
/*    the shape need not be convex -- this is a route, not an     */
/*    area -- so no convex hull is taken.                         */
/*                                                                 */
/*    Until the operator draws anything the configured            */
/*    default_route is in force, and the first click replaces it  */
/*    rather than extending it.                                   */
/*****************************************************************/

#ifndef TARGET_PATH_PLANNER_HEADER
#define TARGET_PATH_PLANNER_HEADER

#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYSegList.h"

class TargetPathPlanner : public AppCastingMOOSApp
{
 public:
  TargetPathPlanner();
  virtual ~TargetPathPlanner() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:
  bool buildReport();
  void registerVariables();

  bool handleMailVertex(std::string);
  bool handleMailRoute(std::string);
  void handleMailClear();
  void handleMailUndo();
  bool applyRoute();
  void drawRoute();
  void eraseRoute();

  XYSegList currentRoute() const;

 protected: // Configuration variables
  std::string m_update_var;    // behavior update variable to post to
  std::string m_route_label;   // label of the drawn route
  std::string m_default_route; // route used until the operator draws one
  double      m_speed;         // 0 = leave the behavior's own speed
  bool        m_auto_apply;    // apply on every click, not just on APPLY
  bool        m_close_loop;    // repeat back to the first point

 protected: // State variables
  std::vector<double> m_vx;
  std::vector<double> m_vy;
  bool         m_route_applied;
  bool         m_route_is_default;
  unsigned int m_routes_posted;
  std::string  m_last_posted;
};

#endif
