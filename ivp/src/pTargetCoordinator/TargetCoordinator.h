/*****************************************************************/
/*    NAME: m_shield_demo                                        */
/*    FILE: TargetCoordinator.h                                  */
/*    DATE: Aug 2026                                              */
/*                                                                 */
/*    pTargetCoordinator tracks the intruder's NODE_REPORT and,   */
/*    while intercept is active, spaces the configured USVs into  */
/*    flanking "pincer" trail angles around the target's stern,   */
/*    evenly spread and continuously re-posted as the target's    */
/*    heading changes.                                            */
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
  bool recordPosition(const NodeRecord&, double& x, double& y) const;
  void assignAndPost();

  std::vector<double>       computeSlots(unsigned int n) const;
  std::vector<unsigned int> assignSlots(double base_angle) const;

 protected: // Geodesy
  CMOOSGeodesy m_geodesy;
  bool         m_geodesy_ok;

 protected: // Configuration variables
  std::vector<std::string> m_vnames;
  std::string              m_target_name;
  double                   m_spread_deg;
  double                   m_trail_range;
  std::string              m_trigger_var;

 protected: // State variables
  std::map<std::string, NodeRecord> m_records;
  bool         m_intercept_active;
  unsigned int m_assignments_posted;
};

#endif
