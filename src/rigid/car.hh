#ifndef RAY_CAR_HPP
#define RAY_CAR_HPP

#include "object.hh"
#include "../render/render_object.hh"
#include "../common/types.hh"
#include "../simulation/physics_controller.hh"
#include "../collision/collision_body.hh"

class Wheel;
class Physics;

class Car : public Object, public Physics_controller{
public:
  Car(Physics* physics, bool fw_drive, bool rw_drive);
  ~Car();

  // control - values -1/0 to 1
  void set_accelerate(double val);
  void set_steer(double val);
  void set_hbrake(double val);

  // resets the wheels etc
  void reset();

  int get_num_wheels_on_floor() const;

  // RigidBody fns
  CollBody * collision_body() { return &m_collision_body; }

  void add_external_forces(double dt);
  void update(double dt);

  // from RenderObject
  void display_object();

  void getBoundingSphere(coord & pos, double & radius) {
    m_collision_body.getBoundingSphere(pos, radius);}

private:
  void draw_triangles() const;
  CollBody m_collision_body;

  std::vector<CollTriangle> m_triangles;

  bool m_fw_drive, m_rw_drive;

  enum Wheel_id {WHEEL_BL, WHEEL_BR, WHEEL_FL, WHEEL_FR};
  Wheel * m_wheels[4];

  // control stuff
  double m_dest_steering; // +1 for left, -1 for right
  double m_dest_accelerate; // +1 for acc, -1 for brake

  double m_steering;
  double m_accelerate;
  double m_hbrake;

  vec3d m_colour;  

  // used for cloning the collision mesh
  static Car * m_original_car;
};

#endif
