/*****************************************************************/
/*    FILE: LandModel.h                                          */
/*    ORGN: m_shield_demo                                        */
/*                                                               */
/*  Charted land, for the planning side of the mission.          */
/*                                                               */
/*  The helm avoids land reactively, through pObstacleMgr and a  */
/*  templated BHV_AvoidObstacleV24 fed the convex tile           */
/*  decomposition. That is enough to keep a hull off the beach   */
/*  but not enough to plan well: a survey lane or an intercept   */
/*  station laid across a headland is still laid across a        */
/*  headland, and the vehicle just spends the leg being shoved   */
/*  sideways. This class gives the planners -- pRegionDivider    */
/*  and pTargetCoordinator -- the same shoreline the helm sees,  */
/*  so they can avoid proposing those legs in the first place.   */
/*                                                               */
/*  It reads the true, non-convex outlines from land.txt, the    */
/*  "display" rows written by gen_land.py. XYPolygon is not used */
/*  to hold them: XYPolygon::contains() returns false outright   */
/*  for any polygon whose convex_state is false, so every land   */
/*  query against a real shoreline would silently answer "not    */
/*  land". Point containment here is an even-odd ray crossing    */
/*  that does not care about convexity.                          */
/*****************************************************************/

#ifndef LAND_MODEL_HEADER
#define LAND_MODEL_HEADER

#include <string>
#include <vector>

class LandModel
{
 public:
  LandModel();
  ~LandModel() {}

  // Reads the "display" rows of a gen_land.py land.txt. Returns false and
  // sets errmsg on a missing or unusable file; an empty file is not an
  // error, it just leaves the model inactive.
  bool loadFile(std::string filename, std::string& errmsg);

  // False when no land was loaded. Every query below then answers as if the
  // whole world were water, so callers can run unguarded and a mission
  // without a land.txt behaves exactly as it did before.
  bool active() const {return(m_rings.size() > 0);}

  unsigned int size() const {return(m_rings.size());}
  std::string  getFile() const {return(m_filename);}
  std::string  getSummary() const;

  bool contains(double x, double y) const;

  // True if the open segment touches land anywhere, endpoints included.
  bool segCrossesLand(double x1, double y1, double x2, double y2) const;

  // Distance from (x,y) to the nearest shoreline, or a negative value if no
  // land is loaded. Zero when the point is on land.
  double distToLand(double x, double y) const;

  // Walks from (x1,y1) toward (x2,y2) and returns the last point still in
  // open water, backed off by `standoff`. Returns false if the start itself
  // is on land, in which case nothing can be salvaged from this leg.
  bool waterPortion(double x1, double y1, double x2, double y2,
                    double standoff, double& ox, double& oy) const;

  // Fraction of the given convex polygon's area that is land, sampled on a
  // grid of the given cell size. Used to tell the operator how much of the
  // region they just drew is not navigable.
  double landFraction(const std::vector<double>& poly_x,
                      const std::vector<double>& poly_y,
                      double cell_size) const;

  const std::vector<std::vector<double> >& ringsX() const {return(m_rings_x);}
  const std::vector<std::vector<double> >& ringsY() const {return(m_rings_y);}

 protected:
  bool   ringContains(unsigned int ix, double x, double y) const;
  double ringDist(unsigned int ix, double x, double y) const;

 private:
  std::string m_filename;

  // Parallel arrays rather than a vector of points: every hot loop here is a
  // scan over one ring's coordinates, and the labels are only ever used for
  // reporting.
  std::vector<std::string>          m_rings;
  std::vector<std::vector<double> > m_rings_x;
  std::vector<std::vector<double> > m_rings_y;

  // Per-ring bounding boxes. The shoreline runs to several hundred vertices
  // and the planners query it per grid cell and per lane sample, so the
  // reject-early test matters.
  std::vector<double> m_bbox_xmin;
  std::vector<double> m_bbox_xmax;
  std::vector<double> m_bbox_ymin;
  std::vector<double> m_bbox_ymax;
};

#endif
