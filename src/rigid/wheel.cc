#include "wheel.hh"
#include "object.hh"
#include "../common/segment.hh"
#include "../collision/sim_heightmap.hh"
#include "../common/log_trace.hh"
#include "../simulation/physics.hh"

using namespace std;


// Wheel
Wheel::Wheel(Object * parent,
             Physics * physics,
             const coord & pos, 
             const vec3d & axis_up,
             double spring,  // force per suspension offset
             double travel,  // suspension travel in each dir
             double inertia,
             double radius,
             double friction,
             double damping) :
m_parent(parent),
m_physics(physics),
m_pos(pos),
m_axis_up(axis_up),
m_spring(spring),
m_travel(travel),
m_inertia(inertia),
m_radius(radius),
m_friction(friction),
m_damping(damping),
m_ang_vel(0.0),
m_steer_angle(0.0),
m_torque(0.0),
m_drive_torque(0.0),
m_axis_angle(0.0),
m_displacement(0.0),
m_up_speed(0.0),
m_locked(false),
m_last_displacement(0.0),
m_last_on_floor(false),
m_display_list_num(0)
{}

// get_forces
bool Wheel::add_forces(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  vec3d force(0);
  vec3d torque(0);
  m_last_displacement = m_displacement;

  // find the best face to use - for now a simple single ray cast downwards
  coord world_pos = m_parent->get_position() + m_parent->get_orientation() * m_pos;
  vec3d world_axis = m_parent->get_orientation() * m_axis_up;

  double ray_len = 2.0 * m_radius + m_travel;
  double ray_offset = m_radius + m_travel;
  Segment wheel_ray(world_pos + ray_offset * world_axis, -ray_len * world_axis);
  coord ground_pos;
  vec3d ground_normal;

  const vector<SimHeightMap *> & terrains = m_physics->get_heightmaps();
  
  bool hit = false;
  double frac = 1.0;

  m_displacement = 0.0;

  for(size_t i = 0 ; i < terrains.size() ; ++i){
    if (terrains[i]->get_segment_collision(wheel_ray, frac, ground_pos, ground_normal)){
      hit = true;
      wheel_ray.dir *= frac;
    }
  }

  if (hit == false){
    m_last_on_floor = false;
    return false;
  }
  m_last_on_floor = true;
  double new_ray_len = wheel_ray.dir.mag();
  double offset_from_origin = new_ray_len - ray_len;

  m_displacement = -offset_from_origin;
  if (m_displacement > m_travel)
    m_displacement = m_travel;

  double displacement_force_mag = m_displacement * m_spring;

  // reduce force when suspension is par to ground
  displacement_force_mag *= dot(ground_normal, world_axis);

  // apply damping
  double damping_force_mag = m_up_speed * m_damping;
  double total_force_mag = displacement_force_mag + damping_force_mag;
  if (total_force_mag < 0.0)
    total_force_mag = 0.0;

  vec3d extra_force = total_force_mag * world_axis;

  force += extra_force;
  torque += -cross(extra_force, m_parent->get_orientation() * m_pos);

  // side-slip friction and drive force. Work out wheel- and floor-relative coordinate frame

  vec3d wheel_fwd = rotation_matrix(m_steer_angle, world_axis) * m_parent->get_orientation().get_col(0);
  vec3d wheel_up = world_axis;
  vec3d wheel_left = cross(wheel_up, wheel_fwd);
  wheel_left.normalise();
  wheel_up = cross(wheel_fwd, wheel_left);

  vec3d ground_up = ground_normal;
  vec3d ground_left = cross(ground_normal, wheel_fwd);
  ground_left.normalise();
  vec3d ground_fwd = cross(ground_left, ground_up);

  vec3d wheel_point_vel = m_parent->get_velocity() + 
    cross(m_parent->get_rotation(), m_parent->get_orientation() * m_pos);

  vec3d rim_vel = -m_ang_vel * cross(wheel_left, m_radius * wheel_up);
  wheel_point_vel += rim_vel;

  // sideways forces
  const double noslip_vel  = 0.2;
  const double slip_vel    = 0.4;
  const double slip_factor = 0.7;

  const double small_vel  = 3.0;
  double friction = m_friction;

  double side_vel = dot(wheel_point_vel, ground_left);
  if ( (side_vel >  slip_vel) || (side_vel < -slip_vel) )
    friction *= slip_factor;
  else if ( (side_vel >  noslip_vel) || (side_vel < -noslip_vel) )
    friction *= 1.0 -  (1.0 - slip_factor) * (fabs(side_vel) - noslip_vel) / (slip_vel - noslip_vel);

  if (side_vel < 0.0)
    friction *= -1.0;
  if (fabs(side_vel) < small_vel)
    friction *= fabs(side_vel) / small_vel;

  double side_force = -friction * total_force_mag;

  extra_force = side_force * ground_left;
  force += extra_force;
  torque += -cross(extra_force, m_parent->get_orientation() * m_pos);

  // fwd/back forces
  friction = m_friction;
  double fwd_vel = dot(wheel_point_vel, ground_fwd);
  if ( (fwd_vel >  slip_vel) || (fwd_vel < -slip_vel) )
    friction *= slip_factor;
  else if ( (fwd_vel >  noslip_vel) || (fwd_vel < -noslip_vel) )
    friction *= 1.0 -  (1.0 - slip_factor) * (fabs(fwd_vel) - noslip_vel) / (slip_vel - noslip_vel);

  if(fwd_vel < 0.0) friction *= -1.0;    
  if (fabs(fwd_vel) < small_vel)
    friction *= fabs(fwd_vel) / small_vel;

  double fwd_force = -friction * total_force_mag;
  extra_force = fwd_force * ground_fwd;
  force += extra_force;
  torque += -cross(extra_force, m_parent->get_orientation() * m_pos);

  // fwd force also spins the wheel
  m_torque += -fwd_force * m_radius;
  m_parent->add_world_torque(torque);
  m_parent->add_world_force(force);
  return true;
}

void Wheel::draw_wheel(){
  double width = m_radius * 0.5f;
  double width2 = 0.5 * width;
  const int num_seg = 16;

  GLCOLOR3(1.0f, 0.0f, 0.0f);
  for (int i = 0 ; i < num_seg ; ++i){
    double ang0 = i * 360.0 / num_seg;
    double ang1 = (i + 1) * 360.0 / num_seg;

    if (i == (num_seg / 2))
      GLCOLOR3(0.0f, 0.0f, 1.0f);

    glBegin(GL_TRIANGLES);

    GLNORMAL3(0.0, 1.0, 0.0);
    GLVERTEX3(0.0, width2, 0.0);
    GLVERTEX3(m_radius * cos_deg(ang1), width2, m_radius * sin_deg(ang1));
    GLVERTEX3(m_radius * cos_deg(ang0), width2, m_radius * sin_deg(ang0));

    GLNORMAL3(0.0, -1.0, 0.0);
    GLVERTEX3(0.0, -width2, 0.0);
    GLVERTEX3(m_radius * cos_deg(ang0), -width2, m_radius * sin_deg(ang0));
    GLVERTEX3(m_radius * cos_deg(ang1), -width2, m_radius * sin_deg(ang1));

    glEnd();
  }
}

void Wheel::display_object() {
  TRACE_METHOD_ONLY(FRAME_1);
  Save_GL_matrix_state GL_state;
  
  vec3d actual_pos = m_pos + m_displacement * m_axis_up;
  GLTRANSLATE(actual_pos[0], actual_pos[1], actual_pos[2]);

  GLROTATE(m_steer_angle, 0.0, 0.0, 1.0);
  GLROTATE(m_axis_angle, 0.0, 1.0, 0.0);

  if (m_display_list_num == 0){
    m_display_list_num = glGenLists(1);
    glNewList(m_display_list_num, GL_COMPILE);
    draw_wheel();
    glEndList();
  }

  glCallList(m_display_list_num);
}


// update
void Wheel::update(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  if (dt <= 0.0f)
    return;

  double orig_ang_vel = m_ang_vel;
  m_up_speed = (m_displacement - m_last_displacement) / dt;

  if (m_locked){
    m_ang_vel = 0.0;
    m_torque = 0.0;
  }else{
    m_ang_vel += m_torque * dt / m_inertia;
    m_torque = 0.0;

    // prevent friction from reversing dir
    if ( (fabs(orig_ang_vel) > 0.00001) && (orig_ang_vel * m_ang_vel < 0.0) )
      m_ang_vel = 0.0;

    m_ang_vel += m_drive_torque * dt / m_inertia;
    m_drive_torque = 0.0;

    const double max_ang_vel = 100.0;
    if (m_ang_vel > max_ang_vel)
      m_ang_vel = max_ang_vel;
    else if (m_ang_vel < -max_ang_vel)
      m_ang_vel = -max_ang_vel;
    m_axis_angle += m_ang_vel * dt * 180 / PI;
  }
}

// reset
void Wheel::reset(){
  m_ang_vel = 0.0;
  m_axis_angle = 0.0;
}
