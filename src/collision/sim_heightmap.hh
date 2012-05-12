#ifndef SIM_HEIGHTMAP_HPP
#define SIM_HEIGHTMAP_HPP

#include "segment.hh"

class SimHeightMap{
public:
  SimHeightMap() { elasticity_ = 0.0f; static_friction_ = 0.0f; dynamic_friction_ = 0.0f;}
  virtual ~SimHeightMap() {}

  double& elasticity() {return elasticity_;}
  const double& elasticity() const {return elasticity_;}
  double& static_friction() {return static_friction_;}
  const double& static_friction() const {return static_friction_;}

  double& dynamic_friction() {return dynamic_friction_;}
  const double& dynamic_friction() const {return dynamic_friction_;}

  virtual double get_z(int i, int j) const = 0;
  virtual void get_z_and_normal(double x, double y, double & z, vec3d & normal) const = 0;

  // position and normal are returned if there is a collision
  virtual bool get_segment_collision(const Segment & segment, 
    double & frac, 
    coord & position, 
    vec3d & normal) const = 0;
private:
  double elasticity_;
  double static_friction_;
  double dynamic_friction_;
};

#endif
