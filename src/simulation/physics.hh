#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "../collision/coll_detect.hh"
#include "../collision/collision_body.hh"

#include <vector>

class SimHeightMap;
class Physics_controller;
class RigidBody;

class Physics{
public:
  Physics();

  void set_timestep(double timestep) {m_timestep = timestep;}
  void set_num_collision_iterations(int num) {m_num_collision_iterations = num;}
  void set_num_contact_iterations(int num) {m_num_contact_iterations = num;}
  void set_num_penetration_iterations(int num) {m_num_penetration_iterations = num;}  
  void set_penetration_resolve_fraction(double frac) {m_penetration_resolve_fraction = frac;}
  void set_time_scale(double scale) {m_time_scale = scale;}

  void add_body(RigidBody * body);
  void add_heightmap(SimHeightMap * heightmap);  
  std::vector<SimHeightMap *> get_heightmaps() {return m_heightmaps;}
  void add_controller(Physics_controller * controller);

  void integrate(double total_time);

  double physics_time() const {return m_physics_time;}

  static const vec3d & get_gravity() {return m_gravity;}
  static void set_gravity(const vec3d & gravity) {m_gravity = gravity;}

  // activate the object, and also pick up any collisions between it
  // and the rest of the world.  Also activates any adjacent objects
  // that would move if this object moved away Be careful about when
  // you use this function - bear in mind that it adds elements to the
  // internal list of collisions, which can relocate the collision
  // list. I.e. be careful about calling it from within a traversal of
  // the collision list.
  void activate_object(RigidBody * body);

  /// enable/disable object freezing. if freezing is disabled, all
  /// frozen object will be activated
  void enable_freezing(bool freeze);

  /// indicates if freezing is currently allowed
  bool is_freezing_enabled() const {return m_freezing_enabled;}
private:
  void do_timestep(double dt);

  // functions working on multiple bodies etc
  void handle_all_collisions(double dt);
  void get_all_external_forces(double dt);
  void update_all_velocities(double dt);
  void update_all_positions(double dt);
  void detect_all_collisions(double dt);
  void clear_all_forces();
  void try_to_freeze_all_objects(double dt);
  void try_to_activate_all_frozen_objects(double step_frac);
  /// try to activate frozen objects that are affected by a touching
  /// active object moving away from them
  void activate_all_frozen_objects_left_hanging();

  // ======== helpers for individual cases =========
  /// Handle an individual collision by classifying it, calculating
  /// impulse, applying impulse and updating the velocities of the
  /// objects. Allows over-riding of the elasticity. Ret val indicates
  /// if an impulse was applied
  bool process_collision(CollInfo & collision, 
                         double dt, 
                         bool override_elasticity,
                         double epsilon = 0.0f);

  /// Sets up any parameters that will not change as the collision
  /// gets processed - e.g. the relative position, elasticity etc.
  void preprocess_collision(CollInfo & collision, double dt);

  /// separate two objects involved in a penetration. This will update
  /// not only the objects, but all other collision structures involved
  /// with the object.  factor indicates how much of the penetration
  /// should be resolved.
  void separate_objects(CollInfo & collision, double factor);

private:
  std::vector<RigidBody*> m_rigid_bodies;
  std::vector<CollInfo> m_collisions;
  std::vector<SimHeightMap *> m_heightmaps;
  std::vector<Physics_controller *> m_controllers;

  struct Stored_data{
    inline Stored_data(RigidBody * rb);
    Stored_data() {}
    coord position;
    mat3d orientation;
    vec3d velocity;
    vec3d rotation;
  };
  std::vector<Stored_data> m_stored_data;

  /// list of the collision bodies - keep as a member var so it
  /// doesn't keep getting resized
  std::vector<CollBody *> m_collision_bodies;

  /// time left over from the last step
  double m_overlap_time;

  /// do a fixed timestep
  double m_timestep;
  
  /// our idea of time
  double m_physics_time;

  /// number of collision iterations
  int m_num_collision_iterations;
  /// number of contact iteratrions
  int m_num_contact_iterations;
  /// number of penetration-resolution iterations
  int m_num_penetration_iterations;
  
  /// amount to resolve by each penetration resolution iteration
  double m_penetration_resolve_fraction;

  /// traverse the contact list forward or backward (to reduce bias)
  enum Traversal_dir {TRAVERSE_FORWARD, TRAVERSE_BACKWARD} m_traverse_dir;

  /// global gravity acceleration
  static vec3d m_gravity;

  /// allow objects to freeze
  bool m_freezing_enabled;

  /// allow physics time to run faster/slower
  double m_time_scale;
};

#endif
