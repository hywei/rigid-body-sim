#include "rigid_body.hh"

bool RigidBody::m_indicate_frozen_objects = true;

inline double safe_inv_num(double num){
  if (num > 0.0f) return (1.0f / num);    
  else return (1.0e-30);    
}

void RigidBody::set_position(const coord & pos){
  m_position = pos;
}

RigidBody::RigidBody(Physics* physics)
:m_physics(physics){
  m_position.set_to(0.0f);
  set_orientation(matrix3_identity());
  m_rotation.set_to(0.0f);
  m_velocity.set_to(0.0f);

  set_inv_mass(0.0f);
  set_body_inv_inertia(0.0f, 0.0f, 0.0f);

  m_immovable = false;

  m_elasticity = 0.0f;
  m_static_friction = 0.0f;
  m_dynamic_friction = 0.0f;
  m_force.set_to(0.0f);
  m_torque.set_to(0.0f);

  m_activity = ACTIVE;
  m_inactive_time = 0.0f;
  m_deactivation_time = 1.0f;
  set_activity_threshold(0.3f, 20.0f);
  m_allow_freezing = true;
  m_sqr_activation_factor = 1.0f;
}


// set_mass
void RigidBody::set_mass(double mass){
  m_mass = mass;
  m_inv_mass = safe_inv_num(m_mass);
}
// set_inv_mass
void RigidBody::set_inv_mass(double inv_mass){
  m_inv_mass = inv_mass;
  m_mass = safe_inv_num(m_inv_mass);
}

// set_body_inertia
void RigidBody::set_body_inertia(double Ixx, double Iyy, double Izz){
  m_body_inertia.set_to(0.0f);
  m_body_inertia(0, 0) = Ixx;
  m_body_inertia(1, 1) = Iyy;
  m_body_inertia(2, 2) = Izz;

  m_body_inv_inertia.set_to(0.0);
  m_body_inv_inertia(0, 0) = safe_inv_num(Ixx);
  m_body_inv_inertia(1, 1) = safe_inv_num(Iyy);
  m_body_inv_inertia(2, 2) = safe_inv_num(Izz);
}


// set_body_inv_inertia
void RigidBody::set_body_inv_inertia(double inv_Ixx, double inv_Iyy, double inv_Izz){
  m_body_inv_inertia.set_to(0.0f);
  m_body_inv_inertia(0, 0) = inv_Ixx;
  m_body_inv_inertia(1, 1) = inv_Iyy;
  m_body_inv_inertia(2, 2) = inv_Izz;

  m_body_inertia.set_to(0.0);
  m_body_inertia(0, 0) = safe_inv_num(inv_Ixx);
  m_body_inertia(1, 1) = safe_inv_num(inv_Iyy);
  m_body_inertia(2, 2) = safe_inv_num(inv_Izz);
}


// add_world_force
void RigidBody::add_world_force(const vec3d& force, const coord& pos){
  m_force += force ;
  m_torque += cross(pos - m_position, force);
}


// add_world_torque
void RigidBody::add_world_torque(const vec3d & torque, const coord & pos){
  m_torque += torque;
  m_force += cross(pos - m_position, torque);
}

void RigidBody::clear_forces(){
  m_force.set_to(0.0f);
  m_torque.set_to(0.0f);
}

void RigidBody::update_velocity(double dt){
  if ((m_immovable) || (m_activity == FROZEN))
    return;

  vec3d orig_velocity = m_velocity;
  vec3d orig_rotation = m_rotation;

  m_velocity += (dt * m_inv_mass) * m_force ;
  m_rotation += m_world_inv_inertia * (dt * m_torque);

  // check the result, and roll-back if needed
  if (!m_velocity.sensible()){
    TRACE("vec3d is not sensible: this = %p\n", this);
    orig_velocity.show("orig vel");
    m_force.show("force");
    m_velocity.show("velocity");
    while (1) {}
    m_velocity = orig_velocity;
  }

  if (!m_rotation.sensible()){
    TRACE("rotation is not sensible: this = %p\n-", this);
    m_rotation.show("rotation");
    orig_rotation.show("orig");
    m_torque.show("torque");
    m_world_inv_inertia.show("inv world inertia");
    while (1) {}
    m_rotation = orig_rotation;
  }
}

void RigidBody::update_position(double dt){
  if ((m_immovable) || (m_activity == FROZEN)){
    CollBody * cb = collision_body();
    if ( cb ){
      cb->setPosition(m_position);
      cb->setOrientation(m_orientation);
    }
    return;
  }

  // in case something goes wrong...
  coord orig_position = m_position;
  mat3d orig_orientation = m_orientation;

  // position is easy
  m_position += dt * m_velocity;

  // rotation is easy too!
  const vec3d & rot = m_rotation;
  m_orientation += dt * Matrix3(0, rot[2], -rot[1],
                                -rot[2], 0, rot[0],
                                rot[1], -rot[0], 0 ) * orig_orientation;
  m_orientation.orthonormalise();

  // check the result, and roll-back if needed
  // hmmm probably due to velocity/ration being screwed, so reset them
  if (!m_position.sensible()){
    TRACE("coord is not sensible: this = %p", this);
    m_position.show("position");
    orig_position.show("orig position");
    m_velocity.show("velocity");
    while (1) {}
    m_position = orig_position;
    m_orientation = orig_orientation;
    m_velocity = vec3d(0);
    m_rotation = vec3d(0);
  }
  if (!m_orientation.sensible()){
    TRACE("mat3d is not sensible: this = %p", this);
    m_orientation.show("orientation");
    orig_orientation.show("orig orientation");
    rot.show("rot");
    while (1) {}
    m_orientation = orig_orientation;
    m_position = orig_position;
    m_velocity = vec3d(0);
    m_rotation = vec3d(0);
  }

  m_inv_orientation = transpose(m_orientation);
 
  // recalculate the world inertia
  m_world_inv_inertia = m_orientation * m_body_inv_inertia * m_inv_orientation;

  CollBody * cb = collision_body();
  if ( cb ){
    cb->setPosition(m_position);
    cb->setOrientation(m_orientation);
  }
}

// state_to_str
static const char * state_to_str(RigidBody::Activity state){
  switch (state){
  case RigidBody::ACTIVE: return "ACTIVE";
  case RigidBody::FROZEN: return "FROZEN";
  }
  return "Invalid state";
}

// set_deactivation_time
void RigidBody::set_deactivation_time(double seconds) {m_deactivation_time = seconds;}
// set_activity_threshold
void RigidBody::set_activity_threshold(double vel, double rot){
  m_sqr_velocity_activity_threshold = vel * vel;
  m_sqr_rotation_activity_threshold = deg_to_rad(rot) * deg_to_rad(rot);
}

// set_activation_factor
void RigidBody::set_activation_factor(double factor){
  m_sqr_activation_factor = factor * factor;
}

// set_activity_state
void RigidBody::set_activity_state(Activity state, double activity_factor){
  TRACE_FILE_IF(FRAME_1)
    TRACE("this = %p: old state = %s, new state = %s\n", 
      this, state_to_str(m_activity), state_to_str(state));
  if(m_allow_freezing) m_activity = state;    
  else m_activity = ACTIVE;
  if (m_activity == ACTIVE){
    m_inactive_time = (1.0f - activity_factor) * m_deactivation_time;
  }
  double frac = m_inactive_time / m_deactivation_time;
  double r = 0.5f;
  double scale = 1.0f - frac / r;
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;

  m_velocity *= scale;
  m_rotation *= scale;
}

// set_allow_freezing
void RigidBody::set_allow_freezing(bool allow){
  m_allow_freezing = allow;
  set_activity_state(ACTIVE);
}

// move_to
void RigidBody::move_to(const coord & pos){
  if (get_activity_state() == FROZEN){
    m_physics->activate_object(this);
  }
  set_position(pos);
}
