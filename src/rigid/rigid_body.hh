#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include "../collision/collision_body.hh"
#include "../simulation/physics.hh"
#include "../common/types.hh"

class RigidBody{
public:
  RigidBody(Physics* physics);
  virtual ~RigidBody() {}

  /// This sets the position, but it also tries to make sure that any
  /// frozen bodies resting against this one get activated if
  /// necessary.  Not very efficient. Be a little careful about when
  /// you call it - it will mess with the physics collision list.
  /// Also, after this call the object will be active.
  void move_to(const coord& pos);

  /// set/get methods
  void set_position(const coord& pos);
  const coord& get_position() const { return m_position; }

  inline void set_orientation(const mat3d & orient);
  const mat3d & get_orientation() const { return m_orientation; }

  void set_velocity(const vec3d & vel) { m_velocity = vel; }
  const vec3d & get_velocity() const { return m_velocity; }

  void set_rotation(const vec3d & rot) { m_rotation = rot; }
  const vec3d & get_rotation() const { return m_rotation; }

  void set_force(const vec3d & f) { m_force = f; }
  const vec3d & get_force() const { return m_force; }

  void set_torque(const vec3d & t) { m_torque = t; }
  const vec3d & get_torque() const { return m_torque; }
  
  // if mass = 0, then inv_mass will be very very large, but not infinite.
  // similarly if inv_mass = 0
  void set_mass(double mass);
  void set_inv_mass(double inv_mass);
  double get_mass() const {return m_mass;}
  double get_inv_mass() const {return m_inv_mass;}

  void set_body_inertia(double Ixx, double Iyy, double Izz);
  void set_body_inv_inertia(double inv_Ixx, double inv_Iyy, double inv_Izz);
  const Matrix3 & get_body_inertia() const {return m_body_inertia;}
  const Matrix3 & get_body_inv_inertia() const {return m_body_inv_inertia;}
  const Matrix3 & get_world_inv_inertia() const { return m_world_inv_inertia; }
  const Matrix3 & get_world_inertia() const { return m_world_inertia; }

  double & elasticity() {return m_elasticity;}
  const double & elasticity() const {return m_elasticity;}

  double & static_friction() {return m_static_friction;}
  const double & static_friction() const {return m_static_friction;}

  double & dynamic_friction() {return m_dynamic_friction;}
  const double & dynamic_friction() const {return m_dynamic_friction;}

  // adds the global gravitational force to this object
  inline void add_gravity();
  
  // functions to add forces etc in the world coordinate frame
  void add_world_force(const vec3d& force) {m_force += force;}
  void add_world_force(const vec3d& force, const coord & pos);
  void add_world_torque(const vec3d& torque) {m_torque += torque;}
  void add_world_torque(const vec3d& torque, const coord & pos);

  inline void apply_world_impulse(const vec3d & impulse);
  inline void apply_world_impulse(const vec3d & impulse, const coord & pos);
  inline void apply_world_ang_impulse(const vec3d & ang_impulse);
  void apply_world_ang_impulse(const vec3d & ang_impulse, const coord & pos);
  
  /// may return 0 if this body doen't engage in collisions
  virtual CollBody* collision_body() { return 0; }
  /// suggest that the derived class implements this
  virtual void enable_collisions(bool enable) {}
  /// allow the body to add on any additional forces (including
  /// gravity)/impulses etc
  virtual void add_external_forces(double dt) {}
  /// This just sets all forces/impulses etc to zero. over-ride if you
  /// want to do more.
  virtual void clear_forces();
  /// implementation updates the velocity/angular rotation with the
  /// force/torque/impulses.  can over-ride to add additional
  /// functionality
  virtual void update_velocity(double dt);
  /// implementation updates the position/orientation with the current
  /// velocties. Can over-ride to add additional functionality
  virtual void update_position(double dt);

  
  /// ensures that this object never moves, and reduces collision checking
  void set_immovable(bool immovable) {
    m_immovable = immovable; 
    set_inv_mass(0.0f); 
    set_body_inv_inertia(0.0f, 0.0f, 0.0f);
  }

  /// indicates if we ever move (change our position - may still have
  /// a non-zero velocity!)
  bool get_immovable() const {return m_immovable;}

  enum Activity {ACTIVE, FROZEN};
  Activity get_activity_state() const {return m_activity;}
  /// allow the activity to be explicitly set - be careful about
  /// explicitly freezing an object (may become unfrozen anyway).
  /// If setting to ACTIVE, then the activity factor indicates how
  /// active the object is considered to be - a value of 1 means
  /// completely active - a value nearer 0 means that freezing
  /// will happen much sooner (assuming no further movement).
  void set_activity_state(Activity state, double activity_factor = 1.0f);

  /// set how long it takes to deactivate
  void set_deactivation_time(double seconds);

  /// set what the velocity threshold is for deactivation
  /// rot is in deg per second.
  void set_activity_threshold(double vel, double rot);

  /// values > 1 make the body less likely to wake up following an impulse.
  void set_activation_factor(double factor);

  /// allows setting of whether this body ever freezes
  void set_allow_freezing(bool allow);

  /// function provided for the use of Physics
  inline void try_to_freeze(double dt);

  // function provided for use of physics - indicates if the velocity
  // is above the threshold for freezing
  inline bool should_be_active() {
    return ( (m_velocity.mag2() > m_sqr_activation_factor * m_sqr_velocity_activity_threshold) ||
             (m_rotation.mag2() > m_sqr_activation_factor * m_sqr_rotation_activity_threshold) );}

  /// function provided for use of physics. Activates any body in its
  /// list if it's moved more than a certain distance, in which case
  /// it also clears its list.
  inline void do_movement_activations();

  /// adds the other body to the list of bodies to be activated if
  /// this body moves more than a certain distance from either a
  /// previously stored position, or the position passed in.
  inline void add_movement_activation(const coord & pos, 
                                      RigidBody * other_body);

  /// set/read this variable as a hint for whether objects should
  /// render in a way that makes it obvious they're frozen. Not
  /// actually used by RigidBody.
  static bool m_indicate_frozen_objects;

private:
  Physics* m_physics;

  coord m_position;
  mat3d m_orientation, m_inv_orientation;
  vec3d m_velocity;
  vec3d m_rotation;

  double m_mass, m_inv_mass;
  bool m_immovable;

  // inertia is expected to be diagonal
  Matrix3 m_body_inertia, m_body_inv_inertia;
  // inertia tensor in world space - not diagonal!
  Matrix3 m_world_inertia, m_world_inv_inertia;

  // elasticity - gets multiplied with the other body in
  // collisions. default = 0
  double m_elasticity;

  // friction
  double m_static_friction;
  double m_dynamic_friction;

  // forces etc in world frame. torque is about com.
  vec3d m_force;
  vec3d m_torque;

  /// for deactivation
  Activity m_activity;
  /// How long we've been still
  double m_inactive_time;

  /// how long it takes to go from active to frozen when stationary
  double m_deactivation_time;

  /// factor applied during should_be_active(). Value greater than one decrease
  /// the tendency to wake up from little impulses.
  double m_sqr_activation_factor;

  /// velocity below which we're considered still
  double m_sqr_velocity_activity_threshold;
  /// velocity below which we're considered still - in (radians per sec)^2
  double m_sqr_rotation_activity_threshold;

  /// The position stored when we need to notify other bodies
  coord m_stored_position_for_activation;
  /// The list of bodies that need to be activated when we move away
  /// from our stored position
  std::vector<RigidBody*> m_bodies_to_be_activated_on_movement;
  
  /// whether this body can freeze (assuming Physics freezing is
  /// enabled)
  bool m_allow_freezing;
};

// do_movement_activations
inline void RigidBody::do_movement_activations(){
  if (m_bodies_to_be_activated_on_movement.empty()) return;
  // TODO don't use hard-coded distance
  if ((m_position - m_stored_position_for_activation).mag2() < 0.05f)
    return;

  const int num_bodies = m_bodies_to_be_activated_on_movement.size();
  for (int i = 0 ; i < num_bodies ; ++i){
    m_physics->activate_object(m_bodies_to_be_activated_on_movement[i]);
  }
  m_bodies_to_be_activated_on_movement.clear();
}

// add_movement_activation
inline void RigidBody::add_movement_activation(const coord& pos, RigidBody * other_body){
  const int num_bodies = m_bodies_to_be_activated_on_movement.size();
  for (int i = 0 ; i < num_bodies ; ++i){
    if (m_bodies_to_be_activated_on_movement[i] == other_body)
      return; // already made a note of this body
  }
  if (num_bodies == 0) m_stored_position_for_activation = pos;
  m_bodies_to_be_activated_on_movement.push_back(other_body);
}

// update_activity_status
inline void RigidBody::try_to_freeze(double dt){
  if ((!m_allow_freezing) || (m_immovable)) return;

  if (m_activity == FROZEN){
    m_inactive_time += dt;
    return;
  }

  // active - check the thresholds
  if(should_be_active() ){
    m_inactive_time = 0.0f;
    return;
  }

  m_inactive_time += dt;

  if (m_inactive_time > m_deactivation_time){// sleep!
    set_activity_state(FROZEN);
  }
}

// set_orientation
inline void RigidBody::set_orientation(const mat3d & orient){ 
  m_orientation = orient; 
  m_inv_orientation = transpose(m_orientation);
  m_world_inv_inertia = m_orientation * m_body_inv_inertia * m_inv_orientation;
  m_world_inertia = m_orientation * m_body_inertia * m_inv_orientation;
}

// apply_world_impulse
inline void RigidBody::apply_world_impulse(const vec3d & impulse){
  vec3d orig_velocity = m_velocity;
  m_velocity += impulse * m_inv_mass ;
  if (!m_velocity.sensible()){
    TRACE("vec3d is not sensible after impulse: this = %p\n", this);
    orig_velocity.show("orig vel");
    impulse.show("impulse");
    m_velocity.show("velocity");
    while(1) {}
  }
}

// apply_world_impulse
inline void RigidBody::apply_world_impulse(const vec3d & impulse, const coord & pos){
  vec3d orig_velocity = m_velocity;
  vec3d orig_rotation = m_rotation;
  m_velocity += impulse * m_inv_mass ;
  m_rotation += m_world_inv_inertia * cross(pos - m_position, impulse);
  if (!m_rotation.sensible()){
    TRACE("rotation is not sensible after impulse: this = %p\n", this);
    orig_rotation.show("orig vel");
    impulse.show("impulse");
    m_rotation.show("rotation");
    while (1) {}
  }
  if (!m_velocity.sensible()){
    TRACE("vec3d is not sensible after impulse: this = %p\n", this);
    orig_velocity.show("orig vel");
    impulse.show("impulse");
    m_velocity.show("velocity");
    while (1) {}
  }
}

// apply_world_ang_impulse
inline void RigidBody::apply_world_ang_impulse(const vec3d& ang_impulse){
  m_rotation += m_world_inv_inertia * ang_impulse;
}

// apply_world_ang_impulse
inline void RigidBody::add_gravity(){
  if ((!m_immovable) && (m_activity == ACTIVE))
    add_world_force(m_mass * Physics::get_gravity());
}

#endif
