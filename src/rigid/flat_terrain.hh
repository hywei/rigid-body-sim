#ifndef RIGIDSIM_RIGID_FLATTERRAIN_H_
#define RIGIDSIM_RIGID_FLATTERRAIN_H_

#include "../common/types.hh"
#include "terrain.hh"

class FlatTerrain : public Terrain {
 public:
  FlatTerrain(uint nx, uint ny, double dx, double dy, double h=0.0, double ela=0.0, double st_fric=0.0, double dy_fric=0.0);

  uint getXNum() const { return nx_; }
  uint getYNum() const { return ny_; }
  double getXInterval() const { return dx_; }
  double getYInterval() const { return dy_; }
  double getXOrigin() const { return -0.5*(nx_-1)*dx_; }
  double getYOrigin() const { return -0.5*(ny_-1)*dy_; }

  double getHeight(double x, double y) const;
  vec3d getNormal(double x, double y) const;
  bool getSegmentCollision(const Segment& seg, double& fric, coord& coll_pos, vec3d& coll_norm) const;

  void draw() const;
 private:
  uint nx_, ny_; // number of interval 
  double dx_, dy_; // the interval
  double height_;
};

#endif
