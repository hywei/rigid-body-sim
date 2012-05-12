#ifndef RAY_WHEEL_HPP
#define RAY_WHEEL_HPP

#include "types.hh"
#include "render_object.hh"
#include "glut_utils.hh"

class Physics;
class Object;

class Wheel{
public:
  Wheel(Object* parent, 
            Physics * physics,
            const coord & pos, 
            const vec3d & axis_up,
            double spring,  // force per suspension offset
            double travel,  // suspension travel upwards
            double inertia,
            double radius,
            double friction,
            double damping);
  
  bool get_on_floor() const {return m_last_on_floor;}

  // Adds the forces die to this wheel to the parent. Return value indicates if it's
  // on the ground.
  bool add_forces(double dt);

  void update(double dt);

  void reset();

  double get_steer() const {return m_steer_angle;}
  void set_steer(double steer) {m_steer_angle = steer;}
  void set_lock(bool lock) {m_locked = lock;}

  void add_torque(double torque) {m_drive_torque += torque;}

  void display_object();

  const coord & get_pos() const {return m_pos;}

private:
  void draw_wheel();
  
  Object * m_parent;
  Physics * m_physics;

  const coord m_pos;
  const vec3d m_axis_up;
  const double m_spring;
  const double m_travel;
  const double m_inertia;
  const double m_radius;
  const double m_friction;
  const double m_damping;
  // things that change 
  double m_ang_vel;
  double m_steer_angle;
  double m_torque;
  double m_drive_torque;
  double m_axis_angle;
  double m_displacement; // = m_travel when fully compressed
  double m_up_speed; // speed relative to the car
  bool   m_locked;
  // last frame stuff
  double m_last_displacement;
  bool   m_last_on_floor;

  GLuint m_display_list_num;
};

#endif
