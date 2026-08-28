/*****************************************************************/
/*    FILE: LandModel.cpp                                        */
/*    ORGN: m_shield_demo                                        */
/*****************************************************************/

#include "LandModel.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "MBUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

LandModel::LandModel()
{
}

//---------------------------------------------------------
// Procedure: loadFile()

bool LandModel::loadFile(string filename, string& errmsg)
{
  m_rings.clear();
  m_rings_x.clear();
  m_rings_y.clear();
  m_bbox_xmin.clear();
  m_bbox_xmax.clear();
  m_bbox_ymin.clear();
  m_bbox_ymax.clear();
  m_filename = filename;

  ifstream file(filename.c_str());
  if(!file.is_open()) {
    errmsg = "could not open " + filename;
    return(false);
  }

  string line;
  unsigned int lineno = 0;
  while(getline(file, line)) {
    lineno++;
    line = stripBlankEnds(stripComment(line, "//"));
    if(line == "")
      continue;

    string key = stripBlankEnds(biteString(line, '='));
    string val = stripBlankEnds(line);

    // Only the display rows. The "tile" rows are the convex decomposition
    // handed to pObstacleMgr; using them here would double-count the
    // buffer that was baked into them and would report the tiles' inland
    // extent, which runs off the chart, as land area.
    if(key != "display")
      continue;

    // Pull the vertex list out by brace, not by splitting on commas: the
    // list is itself comma-separated, so a naive split would have to be
    // stitched back together.
    string::size_type p1 = val.find("pts={");
    if(p1 == string::npos)
      continue;
    string::size_type p2 = val.find('}', p1);
    if(p2 == string::npos) {
      stringstream ss;
      ss << filename << ":" << lineno << " unterminated pts={";
      errmsg = ss.str();
      return(false);
    }
    string pts = val.substr(p1+5, p2-(p1+5));

    vector<string> verts = parseString(pts, ':');
    vector<double> vx, vy;
    for(unsigned int i=0; i<verts.size(); i++) {
      string v = stripBlankEnds(verts[i]);
      string xs = stripBlankEnds(biteString(v, ','));
      string ys = stripBlankEnds(v);
      if(!isNumber(xs) || !isNumber(ys)) {
        stringstream ss;
        ss << filename << ":" << lineno << " bad vertex [" << verts[i] << "]";
        errmsg = ss.str();
        return(false);
      }
      vx.push_back(atof(xs.c_str()));
      vy.push_back(atof(ys.c_str()));
    }
    if(vx.size() < 3)
      continue;

    double xmin = vx[0], xmax = vx[0], ymin = vy[0], ymax = vy[0];
    for(unsigned int i=1; i<vx.size(); i++) {
      xmin = min(xmin, vx[i]);
      xmax = max(xmax, vx[i]);
      ymin = min(ymin, vy[i]);
      ymax = max(ymax, vy[i]);
    }

    stringstream lbl;
    lbl << "land_" << m_rings.size();
    m_rings.push_back(lbl.str());
    m_rings_x.push_back(vx);
    m_rings_y.push_back(vy);
    m_bbox_xmin.push_back(xmin);
    m_bbox_xmax.push_back(xmax);
    m_bbox_ymin.push_back(ymin);
    m_bbox_ymax.push_back(ymax);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: getSummary()

string LandModel::getSummary() const
{
  if(m_rings.size() == 0)
    return("no land loaded");

  unsigned int verts = 0;
  for(unsigned int i=0; i<m_rings_x.size(); i++)
    verts += m_rings_x[i].size();

  stringstream ss;
  ss << m_rings.size() << " land mass(es), " << verts << " vertices, from "
     << m_filename;
  return(ss.str());
}

//---------------------------------------------------------
// Procedure: ringContains()
//   Purpose: Even-odd ray crossing. Convexity-agnostic, which is the whole
//            reason this is not an XYPolygon.

bool LandModel::ringContains(unsigned int ix, double x, double y) const
{
  if(ix >= m_rings_x.size())
    return(false);
  if((x < m_bbox_xmin[ix]) || (x > m_bbox_xmax[ix]) ||
     (y < m_bbox_ymin[ix]) || (y > m_bbox_ymax[ix]))
    return(false);

  const vector<double>& vx = m_rings_x[ix];
  const vector<double>& vy = m_rings_y[ix];
  unsigned int n = vx.size();

  bool inside = false;
  for(unsigned int i=0, j=n-1; i<n; j=i++) {
    // Half-open comparison on y so a vertex exactly at the ray height is
    // counted by one edge and not the other; without it a horizontal ray
    // through a vertex flips parity twice and reports water inside land.
    if(((vy[i] > y) != (vy[j] > y)) &&
       (x < (vx[j] - vx[i]) * (y - vy[i]) / (vy[j] - vy[i]) + vx[i]))
      inside = !inside;
  }
  return(inside);
}

//---------------------------------------------------------
// Procedure: contains()

bool LandModel::contains(double x, double y) const
{
  for(unsigned int i=0; i<m_rings_x.size(); i++)
    if(ringContains(i, x, y))
      return(true);
  return(false);
}

//---------------------------------------------------------
// Procedure: segCrossesLand()

bool LandModel::segCrossesLand(double x1, double y1,
                               double x2, double y2) const
{
  if(m_rings_x.size() == 0)
    return(false);

  // Endpoints first: a segment wholly inside a land mass crosses no edge.
  if(contains(x1, y1) || contains(x2, y2))
    return(true);

  double sxmin = min(x1, x2), sxmax = max(x1, x2);
  double symin = min(y1, y2), symax = max(y1, y2);

  for(unsigned int r=0; r<m_rings_x.size(); r++) {
    if((sxmax < m_bbox_xmin[r]) || (sxmin > m_bbox_xmax[r]) ||
       (symax < m_bbox_ymin[r]) || (symin > m_bbox_ymax[r]))
      continue;

    const vector<double>& vx = m_rings_x[r];
    const vector<double>& vy = m_rings_y[r];
    unsigned int n = vx.size();
    for(unsigned int i=0, j=n-1; i<n; j=i++) {
      double x3 = vx[j], y3 = vy[j], x4 = vx[i], y4 = vy[i];
      double d1 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
      double d2 = (x2-x1)*(y4-y1) - (y2-y1)*(x4-x1);
      double d3 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
      double d4 = (x4-x3)*(y2-y3) - (y4-y3)*(x2-x3);
      if((((d1>0)&&(d2<0)) || ((d1<0)&&(d2>0))) &&
         (((d3>0)&&(d4<0)) || ((d3<0)&&(d4>0))))
        return(true);
    }
  }
  return(false);
}

//---------------------------------------------------------
// Procedure: ringDist()

double LandModel::ringDist(unsigned int ix, double x, double y) const
{
  const vector<double>& vx = m_rings_x[ix];
  const vector<double>& vy = m_rings_y[ix];
  unsigned int n = vx.size();

  double best = -1;
  for(unsigned int i=0, j=n-1; i<n; j=i++) {
    double ax = vx[j], ay = vy[j], bx = vx[i], by = vy[i];
    double dx = bx-ax, dy = by-ay;
    double len2 = dx*dx + dy*dy;
    double t = 0;
    if(len2 > 0) {
      t = ((x-ax)*dx + (y-ay)*dy) / len2;
      if(t < 0) t = 0;
      if(t > 1) t = 1;
    }
    double px = ax + t*dx, py = ay + t*dy;
    double d = hypot(x-px, y-py);
    if((best < 0) || (d < best))
      best = d;
  }
  return(best);
}

//---------------------------------------------------------
// Procedure: distToLand()

double LandModel::distToLand(double x, double y) const
{
  if(m_rings_x.size() == 0)
    return(-1);
  if(contains(x, y))
    return(0);

  double best = -1;
  for(unsigned int i=0; i<m_rings_x.size(); i++) {
    double d = ringDist(i, x, y);
    if((d >= 0) && ((best < 0) || (d < best)))
      best = d;
  }
  return(best);
}

//---------------------------------------------------------
// Procedure: waterPortion()

bool LandModel::waterPortion(double x1, double y1, double x2, double y2,
                             double standoff, double& ox, double& oy) const
{
  ox = x1;
  oy = y1;
  if(m_rings_x.size() == 0) {
    ox = x2;
    oy = y2;
    return(true);
  }
  if(contains(x1, y1))
    return(false);
  if(!segCrossesLand(x1, y1, x2, y2)) {
    ox = x2;
    oy = y2;
    return(true);
  }

  double len = hypot(x2-x1, y2-y1);
  if(len < 1e-6)
    return(true);

  // March in one-meter steps. A bisection would be tidier for a single
  // convex obstacle, but a leg can enter and leave several land masses and
  // only the first entry matters here.
  unsigned int steps = (unsigned int)(len) + 1;
  double px = x1, py = y1;
  for(unsigned int i=1; i<=steps; i++) {
    double t = (double)(i) / (double)(steps);
    double cx = x1 + (x2-x1)*t;
    double cy = y1 + (y2-y1)*t;
    if(contains(cx, cy))
      break;
    px = cx;
    py = cy;
  }

  // Back off along the leg so the trimmed endpoint is not sitting on the
  // waterline itself.
  double dx = (x2-x1)/len, dy = (y2-y1)/len;
  ox = px - dx*standoff;
  oy = py - dy*standoff;
  if(contains(ox, oy))
    return(false);
  return(true);
}

//---------------------------------------------------------
// Procedure: landFraction()

double LandModel::landFraction(const vector<double>& poly_x,
                               const vector<double>& poly_y,
                               double cell_size) const
{
  if((m_rings_x.size() == 0) || (poly_x.size() < 3) || (cell_size <= 0))
    return(0);

  double xmin = poly_x[0], xmax = poly_x[0];
  double ymin = poly_y[0], ymax = poly_y[0];
  for(unsigned int i=1; i<poly_x.size(); i++) {
    xmin = min(xmin, poly_x[i]);
    xmax = max(xmax, poly_x[i]);
    ymin = min(ymin, poly_y[i]);
    ymax = max(ymax, poly_y[i]);
  }

  unsigned int total = 0, land = 0;
  for(double y=ymin+cell_size/2; y<=ymax; y+=cell_size) {
    for(double x=xmin+cell_size/2; x<=xmax; x+=cell_size) {
      // Same even-odd test, applied to the region outline. The region is
      // convex in practice, but reusing one code path avoids a second
      // containment convention to keep straight.
      bool in_region = false;
      unsigned int n = poly_x.size();
      for(unsigned int i=0, j=n-1; i<n; j=i++) {
        if(((poly_y[i] > y) != (poly_y[j] > y)) &&
           (x < (poly_x[j]-poly_x[i]) * (y-poly_y[i]) /
                (poly_y[j]-poly_y[i]) + poly_x[i]))
          in_region = !in_region;
      }
      if(!in_region)
        continue;
      total++;
      if(contains(x, y))
        land++;
    }
  }
  if(total == 0)
    return(0);
  return((double)(land) / (double)(total));
}
