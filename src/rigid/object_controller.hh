#ifndef OBJECT_CONTROLLER_HPP
#define OBJECT_CONTROLLER_HPP

#include "../simulation/physics_controller.hh"
#include "../common/types.hh"

class Object;
class Physics;

/// constantly tries to set the velocity of an object
class ObjectController : public PhysicsController{
public:
  // needs to be told the physics system
  ObjectController(Physics * physics);

  /// pass in 0 if you want to disable controlling any object
  void set_controller_object(Object * object);

  /// make this controller control
  void set_control_enabled(bool control);

  /// set the force to use per m/s difference between current vel and desired vel
  /// when controlling
  void set_force(double force_per_vel_per_mass) {m_force_per_vel_per_mass = force_per_vel_per_mass;}

  /// specifies the velocity that the controlled object should have
  void set_controlled_velocity(const vec3d & vel);

  /// do the control (if enabled)
  void update(double dt);

private:
  Physics* m_physics;
  Object* m_object;
  vec3d m_velocity;
  double m_force_per_vel_per_mass;
  bool m_control;
};

#endif
