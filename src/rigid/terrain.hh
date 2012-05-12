#ifndef RIGIDSIM_RIGID_TERRAIN_H_
#define RIGIDSIM_RIGID_TERRAIN_H_

class Segment;

class Terrain{
 public:
  Terrain(double ela=0,  double st_fric=0, double dy_fric=0):
      elasticity_(ela), static_friction_(st_fric), dynamic_friction_(dy_fric) {}
  virtual ~Terrain(){}

  void setElasticity(double ela) { elasticity_ = ela;}
  double getElasticity() const { return elasticity_; }
  void setStaticFriction(double st_fric) { static_friction_ = st_fric; }
  double getStaticFriction() const { return static_friction_; }
  void setDynamicFriction(double dy_fric) { dynamic_friction_ = dy_fric; }
  double getDynamicFriction() { return dynamic_friction_; }

  virtual double getHeight(int x, int y) const = 0;
  virtual vec3d getNormal(int x, int y) const = 0;
  virtual bool getSegmentCollision(const Segment& seg, coord& coll_pos, vec3d& coll_norm) const = 0;
  
 private:
  double elasticity_, static_friction_, dynamic_friction_;
};

#endif
