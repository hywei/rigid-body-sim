#include "physics.hh"
#include "rigid_body.hh"
#include "sim_heightmap.hh"
#include "physics_controller.hh"

using namespace std;

// limit the extra velocity during collision/penetration calculations
static const double max_vel_mag = 30.0f;

vec3d Physics::m_gravity(0.0f, 0.0f, -9.81f);

inline Physics::Stored_data::Stored_data(RigidBody * rb)
:
  position(rb->get_position()),
  orientation(rb->get_orientation()),
  velocity(rb->get_velocity()),
  rotation(rb->get_rotation())
{}

Physics::Physics(){
  TRACE_METHOD_ONLY(ONCE_1);

  m_timestep = 1.0f / 120;
  m_overlap_time = 0.0f;
  m_num_collision_iterations = 5;
  m_num_contact_iterations = 10;
  m_num_penetration_iterations = 10;
  m_penetration_resolve_fraction = 0.008;
  m_traverse_dir = TRAVERSE_FORWARD;
  m_freezing_enabled = true;
  m_time_scale = 1.0f;
}

void Physics::add_body(RigidBody * body){
  m_rigid_bodies.push_back(body);
}

void Physics::add_heightmap(SimHeightMap * heightmap){
  m_heightmaps.push_back(heightmap);
}


// add_controller
void Physics::add_controller(Physics_controller* controller){
  m_controllers.push_back(controller);
}

// activate_object
// activate object and bring it into the collision list/do collision detection
void Physics::activate_object(RigidBody * body){
  if ( (body->get_activity_state() == RigidBody::ACTIVE) ||
       (body->get_immovable()) ) return;

  body->set_activity_state(RigidBody::ACTIVE, 1.0f);

  int orig_num = m_collisions.size();
  if (!body->collision_body()) return;

  ::detect_frozen_collisions_with_body(body->collision_body(),
                                       m_collision_bodies, m_heightmaps, m_collisions);
  m_collision_bodies.push_back(body->collision_body());

  // now check that any adjacent touching bodies wouldn't accelerate towards us if we moved away
  int new_num = m_collisions.size();
  for (int i = orig_num ; i < new_num ; ++i){
    // must be a body-body interaction to be interesting
    if (m_collisions[i].body1){
      RigidBody * other_body = m_collisions[i].body0->rigid_body();
      // the collision normal pointing from body to other_body
      vec3d this_body_normal = m_collisions[i].dir_to_0;
      if (other_body == body){
        other_body = m_collisions[i].body1->rigid_body();
        this_body_normal *= -1;
      }
      if (other_body->get_activity_state() == RigidBody::FROZEN){
        // remember that the RigidBody doesn't apply gravity to sleeping bodies
        vec3d force_on_other = other_body->get_mass() * get_gravity() + other_body->get_force();
        if (dot(force_on_other, this_body_normal) <= 0.0f){
          // wake it up recursively. after this, the contents of our m_collisions may have been relocated
          activate_object(other_body);
        }
      }
    }
  }
}

// preprocess_collision
inline void Physics::preprocess_collision(CollInfo & collision, double dt){
  RigidBody * body0 = collision.body0->rigid_body();
  RigidBody * body1 = 0;

  if (collision.body1){
    body1 = collision.body1->rigid_body();
  }

  collision.R0 = collision.position - body0->get_position();

  const double allowed_penetration = 0.0f;
  const double timescale = 5 * dt;
  if (collision.penetration_depth > allowed_penetration){
    collision.vr_extra = ((collision.penetration_depth - allowed_penetration) / timescale);
    if (collision.vr_extra > max_vel_mag)
      collision.vr_extra = max_vel_mag;
  }else
    collision.vr_extra = 0;

  const vec3d & N = collision.dir_to_0;
  if (body1){
    collision.R1 = collision.position - body1->get_position();
    collision.elasticity = body0->elasticity() * body1->elasticity();
    collision.static_friction = 0.5f * (body0->static_friction() + body1->static_friction());
    collision.dynamic_friction = 0.5f * (body0->dynamic_friction() + body1->dynamic_friction());
    collision.denominator = body0->get_inv_mass() + body1->get_inv_mass() + 
      dot(N, cross(body0->get_world_inv_inertia() * (cross(collision.R0, N)), collision.R0)) + 
      dot(N, cross(body1->get_world_inv_inertia() * (cross(collision.R1, N)), collision.R1));
  }else{
    collision.elasticity = body0->elasticity() * collision.heightmap->elasticity();
    collision.static_friction = 0.5f * (body0->static_friction() + collision.heightmap->static_friction());
    collision.dynamic_friction = 0.5f * (body0->dynamic_friction() + collision.heightmap->dynamic_friction());
    collision.denominator = body0->get_inv_mass() + 
      dot(N, cross(body0->get_world_inv_inertia() * (cross(collision.R0, N)), collision.R0));
  }
}

// process_collision
bool Physics::process_collision(CollInfo & collision, 
                                double dt,
                                bool override_elasticity,
                                double epsilon){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  if (collision.denominator < 0.0001f)
    return false;

  RigidBody * body0 = collision.body0->rigid_body();
  RigidBody * body1 = 0;

  if (collision.body1){
    body1 = collision.body1->rigid_body();
  }

  const vec3d & N = collision.dir_to_0;
  double normal_vel;
  if (body1){
    normal_vel =  dot((body0->get_velocity() + cross(body0->get_rotation(), collision.R0)) - 
                      (body1->get_velocity() + cross(body1->get_rotation(), collision.R1)), N); 
  }else{
    normal_vel =  dot((body0->get_velocity() + cross(body0->get_rotation(), collision.R0)), N); 
  }

  // coefficient of restitution
  if (!override_elasticity){
    epsilon = collision.elasticity;
  }

  double final_normal_vel = -epsilon * normal_vel;

  // delta_vel is the change in normal velocity. Only add the extra vel if over-riding elasticity
  // during the second phase.
  if (override_elasticity){
    double extra = collision.vr_extra - final_normal_vel;
    if (extra > 0.0f)
      final_normal_vel += extra;
  }

  double delta_vel = final_normal_vel - normal_vel;
  if (delta_vel <= 0.0f)
    return false;

  double numerator = delta_vel;
  double normal_impulse = numerator / collision.denominator;
  bool body0_frozen_pre = body0->get_activity_state() == RigidBody::FROZEN;
  bool body1_frozen_pre = false;
  if (body1)
    body1_frozen_pre = body1->get_activity_state() == RigidBody::FROZEN;

  body0->apply_world_impulse(normal_impulse * N, collision.position);
  if (body1)
    body1->apply_world_impulse(-normal_impulse * N, collision.position);

  // if one of the bodies is frozen, then if it is to stay frozen the other body should
  // take the whole impulse
  if (body0_frozen_pre){
    if (!body0->should_be_active()){
      // OK - reverse the impulse on body0, and apply it to body1
      body0->apply_world_impulse(-normal_impulse * N, collision.position);
      if (body1)
        body1->apply_world_impulse(-normal_impulse * N, collision.position);
    }
  }else if (body1_frozen_pre){
    if (!body1->should_be_active()){
      // OK - reverse the impulse on body1, and apply it to body0
      body0->apply_world_impulse(normal_impulse * N, collision.position);
      body1->apply_world_impulse(normal_impulse * N, collision.position);
    }
  }
  
  // For friction, work out the impulse in the opposite direction to the tangential
  // velocity that would be required to bring this point to a halt. Apply the minimum 
  // of this impulse magnitude, and the one obtained from the normal impulse. This 
  // prevents reversing the velocity direction.
  //
  // However, recalculate the velocity since it's changed. 
  vec3d Vr_new = body0->get_velocity() + cross(body0->get_rotation(), collision.R0);
  if (body1)
    Vr_new -= (body1->get_velocity() + cross(body1->get_rotation(), collision.R1));

  vec3d tangent_vel = Vr_new - dot(Vr_new, N)  * N;
  double tangent_speed = tangent_vel.mag();
  if (tangent_speed > 0.0f){
    vec3d T = -tangent_vel / tangent_speed;
    numerator = tangent_speed;
    double denominator;
    if (body1){
      denominator = body0->get_inv_mass() + body1->get_inv_mass() + 
        dot(T, cross(body0->get_world_inv_inertia() * (cross(collision.R0, T)), collision.R0)) + 
        dot(T, cross(body1->get_world_inv_inertia() * (cross(collision.R1, T)), collision.R1));
    }else{
      denominator = body0->get_inv_mass() + 
        dot(T, cross(body0->get_world_inv_inertia() * (cross(collision.R0, T)), collision.R0));
    }

    if (denominator > 0.0f){
      double impulse_to_reverse = numerator / denominator;
      double static_friction = collision.static_friction;
      double dynamic_friction = collision.dynamic_friction;

      double impulse_from_normal_impulse = static_friction * normal_impulse;
      double friction_impulse;

      if (impulse_to_reverse < impulse_from_normal_impulse)
        friction_impulse = impulse_to_reverse;
      else
        friction_impulse = dynamic_friction * normal_impulse;
        
      body0->apply_world_impulse(friction_impulse * T, collision.position);
      if (body1)
        body1->apply_world_impulse((-friction_impulse) * T, collision.position);
    }
  } // end of friction
    
  return true;
}


// try_to_activate_all_frozen_objects
void Physics::try_to_activate_all_frozen_objects(double step_frac){

  for (size_t i = 0 ; i < m_rigid_bodies.size(); ++i){
    if (m_rigid_bodies[i]->get_activity_state() == RigidBody::FROZEN){
      if (m_rigid_bodies[i]->should_be_active())
        activate_object(m_rigid_bodies[i]);
      else{
        m_rigid_bodies[i]->set_velocity(vec3d(0.0f));
        m_rigid_bodies[i]->set_rotation(vec3d(0.0f));        
      }
    }
  }
}
  

// detect_all_collisions
void Physics::detect_all_collisions(double dt) {
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  int num_bodies = m_rigid_bodies.size();
  m_collision_bodies.clear();
  m_collisions.clear();
  
  for(int i = 0 ; i < num_bodies ; ++i){
    if(m_rigid_bodies[i]->collision_body()){      
      m_collision_bodies.push_back(m_rigid_bodies[i]->collision_body());
      m_rigid_bodies[i]->collision_body()->getCollisions().clear();
    }
    
  }
  ::detect_all_collisions(m_collision_bodies, m_heightmaps[0], m_collisions);
}


// handle_all_collisions
void Physics::handle_all_collisions(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  // handle collisions by:
  // 1. store the original position/velocity/force info.
  // 2. predict a new velocity using the external forces (and zero the forces)
  // 3. use the new velocity to predict a new position
  // 4. detect contacts
  // 5. reset the velocity to its original, zero forces
  // 6. classify the contacts based on the "current" velocity, and add/apply impulses
  // 7. repeat step 6 a few times...
  // 8. reset the position/forces to the original

  // then in the next functions
  // 9. apply forces

  // 10.0 detect collisions and handle using inelastic collisions

  int num_bodies = m_rigid_bodies.size();
  // step 1
  m_stored_data.resize(num_bodies);
  for (int i = 0 ; i < num_bodies ; ++i){
    m_stored_data[i] = Stored_data(m_rigid_bodies[i]);
  }

  // step 2
  update_all_velocities(dt);
  // step 3
  update_all_positions(dt);
  // step 4
  detect_all_collisions(dt);
  int orig_num_collisions = m_collisions.size();
  // step 5
  for(int i = 0 ; i < num_bodies ; ++i){
    m_rigid_bodies[i]->set_velocity(m_stored_data[i].velocity);
    m_rigid_bodies[i]->set_rotation(m_stored_data[i].rotation);
  }
  // prepare for the collisions
  for(int i = 0 ; i < orig_num_collisions ; ++i){
    preprocess_collision(m_collisions[i], dt);
  }

  // iterate over the collisions
  for (int step = 0 ; step < m_num_collision_iterations ; ++step){
    bool got_one = false;
    // step 6
    int i_start, i_end, i_dir;
    int num_collisions = m_collisions.size();
    if (m_traverse_dir == TRAVERSE_FORWARD){    
      i_start = 0;
      i_end = num_collisions;
      i_dir = 1;
      m_traverse_dir = TRAVERSE_BACKWARD;
    }else{
      i_start = num_collisions - 1;
      i_end = -1;
      i_dir = -1;
      m_traverse_dir = TRAVERSE_FORWARD;
    }
    for(int i = i_start ; i != i_end; i += i_dir){
      if (process_collision(m_collisions[i], dt, false))
        got_one = true;
    }

    // wake up any previously staionary frozen objects that were frozen. 
    // Also, if they're moving less than the threshold, freeze them
    if (m_freezing_enabled)
      try_to_activate_all_frozen_objects(step / (m_num_collision_iterations - 1.0f));

    // number of collisions may have increased...
    num_collisions = m_collisions.size();

    // preprocess any new collisions.
    for(int i = orig_num_collisions ; i < num_collisions ; ++i){
      preprocess_collision(m_collisions[i], dt);
    }
    orig_num_collisions = num_collisions;
    
    if (!got_one) break;
  }
  // step 7
  for(int i = 0 ; i < num_bodies ; ++i){
    m_rigid_bodies[i]->set_position(m_stored_data[i].position);
    m_rigid_bodies[i]->set_orientation(m_stored_data[i].orientation);
  }
}


// separate_objects
void Physics::separate_objects(CollInfo & collision, double factor){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  RigidBody * body0 = collision.body0->rigid_body();
  // TODO wake up if separation is needed?
  if ( (body0->get_immovable()) || (body0->get_activity_state() == RigidBody::FROZEN) )
    return;
  
  if ( (collision.body1 == 0) || 
       (collision.body1->rigid_body()->get_immovable()) ||
       (collision.body1->rigid_body()->get_activity_state() == RigidBody::FROZEN) ){
      // ground - just update body 0
      vec3d delta = (factor * collision.penetration_depth) * collision.dir_to_0;
      coord orig = body0->get_position();
      body0->set_position(orig + delta);
      std::vector<int> & colls = collision.body0->getCollisions();
      int num_colls = colls.size();
      for (int i = 0 ; i < num_colls ; ++i){
        CollInfo & info = m_collisions[colls[i]];
        info.penetration_depth -= dot(delta, info.dir_to_0);
      }
  }else{
    RigidBody* body1 = collision.body1->rigid_body();
    
    vec3d delta = (factor * collision.penetration_depth) * collision.dir_to_0;
    // how should we partition the separation? It's not physical, so just 50-50
    delta *= 0.5f;
    const coord orig0 = body0->get_position();
    const coord orig1 = body1->get_position();
    body0->set_position(orig0 + delta);
    body1->set_position(orig1 - delta);
    // now update all the collisions affected
    std::vector<int> & colls0 = collision.body0->getCollisions();
    // collisions associated with body 0
    for(int i = 0 ; i < colls0.size() ; ++i){
      CollInfo & info = m_collisions[colls0[i]];
      if (info.body0 == collision.body0)
        info.penetration_depth -= dot(delta, info.dir_to_0);
      else
        info.penetration_depth += dot(delta, info.dir_to_0);
    }
    std::vector<int> & colls1 = collision.body1->getCollisions();
    // collisions associated with body 1
    for(int i = 0 ; i < colls1.size(); ++i){
      CollInfo & info = m_collisions[colls1[i]];
      if (info.body0 == collision.body1)
        info.penetration_depth += dot(delta, info.dir_to_0);
      else
        info.penetration_depth -= dot(delta, info.dir_to_0);
    }
  }
}
 
// do_all_external_forces
void Physics::get_all_external_forces(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  for (size_t i = 0 ; i < m_rigid_bodies.size() ; ++i){
    m_rigid_bodies[i]->add_external_forces(dt);
  }
  for (size_t i = 0 ; i < m_controllers.size() ; ++i){
      m_controllers[i]->update(dt);
  }
}

// update_all_velocities
void Physics::update_all_velocities(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  for (size_t i = 0 ; i < m_rigid_bodies.size() ; ++i){
    m_rigid_bodies[i]->update_velocity(dt);
  }
}
 
// clear_all_forces
void Physics::clear_all_forces(){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  for (size_t i = 0 ; i < m_rigid_bodies.size() ; ++i){
    m_rigid_bodies[i]->clear_forces();
  }
}


// update_all_positions
void Physics::update_all_positions(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  for (size_t i = 0 ; i < m_rigid_bodies.size(); ++i) {    
    m_rigid_bodies[i]->update_position(dt);
  }
}


// try_to_freeze_all_objects
void Physics::try_to_freeze_all_objects(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  for (size_t i = 0 ; i < m_rigid_bodies.size(); ++i) {    
    m_rigid_bodies[i]->try_to_freeze(dt);
  }
}


// activate_all_frozen_objects_left_hanging
void Physics::activate_all_frozen_objects_left_hanging(){
  for (size_t i = 0 ; i < m_rigid_bodies.size(); ++i) { 
    if ( (m_rigid_bodies[i]->get_activity_state() == RigidBody::ACTIVE) &&
         (m_rigid_bodies[i]->collision_body()) ){
      RigidBody * this_body = m_rigid_bodies[i];
      // first activate any bodies due to the movement of this body
      this_body->do_movement_activations();

      // now record any movement notifications that are needed
      std::vector<int> & collisions = m_rigid_bodies[i]->collision_body()->getCollisions();
      if (!collisions.empty()){
        // walk through the object's contact list
        for (size_t j = 0 ; j < collisions.size() ; ++j){          
          const CollInfo & coll = m_collisions[collisions[j]];
          // must be a body-body interaction to be interesting
          if (coll.body1){            
            RigidBody * other_body = coll.body0->rigid_body();
            vec3d this_body_normal = coll.dir_to_0;
            if (other_body == this_body){
              other_body = coll.body1->rigid_body();
              this_body_normal *= -1;
            }
            if (other_body->get_activity_state() == RigidBody::FROZEN){
              this_body->add_movement_activation(this_body->get_position(), other_body);
            }
          }
        }
      }
    }
  }
}


// do_timestep
void Physics::do_timestep(double dt){
  TRACE_METHOD_ONLY(MULTI_FRAME_1);
  get_all_external_forces(dt);
  handle_all_collisions(dt);
  update_all_velocities(dt);
  clear_all_forces();
  if (m_freezing_enabled){
    try_to_freeze_all_objects(dt);
    activate_all_frozen_objects_left_hanging();
  }
  update_all_positions(dt);
}
// 
void Physics::integrate(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  dt *= m_time_scale;
  double total_time = dt + m_overlap_time;
  if (total_time > 0.1f){
    TRACE("dt > 0.1f\n");
    total_time = 0.1f;
  }  
  // split the timestep into fixed size chunks
  int num_loops = (int) (total_time / m_timestep);
  double timestep = m_timestep;
  
  m_overlap_time = total_time - num_loops * timestep;
  
  for (int i = 0 ; i < num_loops ; ++i){
    m_physics_time += timestep;
    do_timestep(timestep);
  }
}


// enable_freezing
void Physics::enable_freezing(bool freeze){
  m_freezing_enabled = freeze;
  if (!m_freezing_enabled){
    for (size_t i = 0 ; i < m_rigid_bodies.size() ; ++i){      
      m_rigid_bodies[i]->set_activity_state(RigidBody::ACTIVE);
    }
  }
}
 
