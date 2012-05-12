#include "object_controller.hh"
#include "object.hh"
#include "../simulation/physics.hh"


ObjectController::ObjectController(Physics * physics)
:
m_physics(physics),
m_object(0),
m_velocity(0.0f),
m_force_per_vel_per_mass(15.0f),
m_control(false){}

void ObjectController::set_controller_object(Object * object){
  m_object = object;
}

void ObjectController::set_control_enabled(bool control){
  m_control = control;
}

void ObjectController::set_controlled_velocity(const vec3d & vel){
  m_velocity = vel;
}

void ObjectController::update(double dt){
  if (m_control && m_object){
    m_physics->activate_object(m_object);
    m_object->add_world_force((m_velocity - m_object->get_velocity() ) * 
                              m_force_per_vel_per_mass * m_object->get_mass());
  }
}
