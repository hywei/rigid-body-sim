#include "simulator.hh"
#include "flat_simple_heightmap.hh"
#include "box.hh"
#include "ellipsoid.hh"
#include "car.hh"
#include "car_ai.hh"

using namespace std;

coord object_offset(0.0f);

Simulator::Simulator(int & argc, char * argv[])
    :GlutApp(argc, argv, "simulator.cfg"),
     m_viewport("Simulator-viewport", &m_render_manager, 
                &m_camera, 0.0f, 0.0f, 1.0f, 1.0f),
     m_axes(10.0f),
     m_object_controller(&m_physics),
     m_frame_dt(0.0f),
     m_frame_start_time(-1.0f),
     m_frame_counter(0){
  
  m_car_ai = new CarAI(40.0f);
  m_physics.add_controller(m_car_ai);

  // set up physics parameters
  double physics_freq = 100.0f;
  double physics_time_scale = 1.0f;
  int physics_num_collision_iterations = 5;
  int physics_num_contact_iterations = 10;
  int physics_num_penetration_iterations = 10;
  bool physics_reuse_collisions = false;
  double physics_penetration_resolve_fraction = 0.008;
  double gravity = 9.81;
  bool physics_enable_freezing = true;
  bool interp_coll_mesh = true;

  get_config_file()->get_value("physics_freq", physics_freq);
  get_config_file()->get_value("physics_time_scale", physics_time_scale);
  get_config_file()->get_value("physics_num_collision_iterations", physics_num_collision_iterations);
  get_config_file()->get_value("physics_num_contact_iterations", physics_num_contact_iterations);
  get_config_file()->get_value("physics_num_penetration_iterations", physics_num_penetration_iterations);
  get_config_file()->get_value("physics_reuse_collisions", physics_reuse_collisions);
  get_config_file()->get_value("physics_penetration_resolve_fraction", physics_penetration_resolve_fraction);
  get_config_file()->get_value("physics_gravity", gravity);
  get_config_file()->get_value("physics_enable_freezing", physics_enable_freezing);
  get_config_file()->get_value("physics_indicate_frozen_objects", RigidBody::m_indicate_frozen_objects);

  m_physics.set_timestep(1.0f / physics_freq);
  m_physics.set_time_scale(physics_time_scale);
  m_physics.set_num_collision_iterations(physics_num_collision_iterations);
  m_physics.set_num_contact_iterations(physics_num_contact_iterations);
  m_physics.set_num_penetration_iterations(physics_num_penetration_iterations);
  m_physics.set_penetration_resolve_fraction(physics_penetration_resolve_fraction);
  Physics::set_gravity(vec3d(0, 0, -gravity));
  m_physics.enable_freezing(physics_enable_freezing);  

  // object offset
  get_config_file()->get_value("object_offset_x", object_offset[0]);
  get_config_file()->get_value("object_offset_y", object_offset[1]);
  get_config_file()->get_value("object_offset_z", object_offset[2]);
  
  setup_objects_from_config();  

  // add the remaining body/objects to the rendering
  m_render_manager.add_object(&m_axes);

  // set the controller
  m_object_controller.set_controller_object(m_objects[0]);
  double controller_force = 200.0f;
  get_config_file()->get_value("object_controller_force_per_vel_per_mass", 
                               controller_force);
  m_object_controller.set_force(controller_force);  
  m_physics.add_controller(&m_object_controller);
}

void Simulator::initialise(const std::string& app_name){
  GlutApp::initialise(app_name);
}

void Simulator::display(){
  check_errors("start of display_fn");
  m_viewport.display();
  glutSwapBuffers();
  check_errors("end of display_fn");
}

void Simulator::idle(){
  TRACE_METHOD_ONLY(FRAME_1);
  double last_frame_time = m_frame_start_time;
  m_frame_start_time = 0.001f * glutGet(GLUT_ELAPSED_TIME);
  if (m_frame_start_time >= 0.0f)
    m_frame_dt = m_frame_start_time - last_frame_time;
  else m_frame_dt = 0.0f;

  if (floor(last_frame_time) != floor(m_frame_start_time)){
    TRACE("FPS = %d\n", m_frame_counter);
    m_frame_counter = 0;
  }else
    ++m_frame_counter;

  m_physics.integrate(m_frame_dt);

  // sort out the viewpoint
  m_camera.update(m_frame_dt);

  glutPostRedisplay();
}

// special_fn
void Simulator::special(int key, int x, int y){
  switch (key){
    case GLUT_KEY_LEFT:
      move_camera(CAMERA_LEFT);
      break;
    case GLUT_KEY_RIGHT:
      move_camera(CAMERA_RIGHT);
      break;
    case GLUT_KEY_UP:
      move_camera(CAMERA_FWD);
      break;
    case GLUT_KEY_DOWN:
      move_camera(CAMERA_BACK);
      break;
  default:
    TRACE_FUNCTION(); TRACE("Unhandled key: %d at (%d, %d)\n", key, x, y);
  }
}

// keyboard_fn
void Simulator::keyboard(unsigned char key, int x, int y){
  if (key == 27) {
    TRACE("Pressed escape - exit\n");
    exit(0);
  }
  vec3d fwd = m_camera.get_target() - m_camera.get_position();
  fwd[2] = 0.0f;
  fwd.normalise();
  vec3d left = cross(m_camera.get_up(), fwd);
  left.normalise();
  
  double speed = 3.0f;
  fwd *= speed;
  left *= speed;
  vec3d up_vel(0, 0, speed);

  switch (key){
  case 'c':
    create_car();
    break;
  case 'b':
    create_box();
    break;
  case 'F':
    if (m_physics.is_freezing_enabled()){
      TRACE("disabling freezing\n");
      m_physics.enable_freezing(false);
    }else{
      TRACE("enabling freezing\n");
      m_physics.enable_freezing(true);
    }
  }
}
// keyboard_up
void Simulator::keyboard_up(unsigned char key, int x, int y){}

void Simulator::move_camera(Camera_dir dir){
  coord pos = m_camera.get_position();
  coord target = m_camera.get_target();
  vec3d up(0, 0, 1.0f);

  vec3d cam_dir = target - pos;
  double cam_dist = cam_dir.mag();
  cam_dir[2] = 0.0f;
  cam_dir.normalise();

  vec3d cam_left = cross(up, cam_dir);
  double dist = 2.0f * cam_dist * m_frame_dt;
  vec3d offset(0.0f);

  switch (dir){
  case CAMERA_LEFT:
    offset = cam_left * dist;
    break;
  case CAMERA_RIGHT:
    offset = -cam_left * dist;
    break;
  case CAMERA_FWD:
    offset = cam_dir * dist;
    break;
  case CAMERA_BACK:
    offset = -cam_dir * dist;
    break;
  }
  m_camera.set_position(pos + offset);
}

void Simulator::setup_objects_from_config(){
  int num_boxes = 5;
  int num_spheres = 1;
  int num_cars = 0;
  int num_walls = 0;
  get_config_file()->get_value("num_boxes", num_boxes);
  get_config_file()->get_value("num_spheres", num_spheres);
  get_config_file()->get_value("num_cars", num_cars);
  get_config_file()->get_value("num_walls", num_walls);

  double box_separation_z = 2.0f;
  get_config_file()->get_value("box_separation_z", box_separation_z);

  vec3d box_vel(0.0f);
  vec3d box_rot(0.0f);
  vec3d sphere_vel(0.0f);
  vec3d sphere_rot(0.0f);

  get_config_file()->get_value("box_vel_x", box_vel[0]);
  get_config_file()->get_value("box_vel_y", box_vel[1]);
  get_config_file()->get_value("box_vel_z", box_vel[2]);
  get_config_file()->get_value("box_rot_x", box_rot[0]);
  get_config_file()->get_value("box_rot_y", box_rot[1]);
  get_config_file()->get_value("box_rot_z", box_rot[2]);
  
  get_config_file()->get_value("sphere_vel_x", sphere_vel[0]);
  get_config_file()->get_value("sphere_vel_y", sphere_vel[1]);
  get_config_file()->get_value("sphere_vel_z", sphere_vel[2]);
  get_config_file()->get_value("sphere_rot_x", sphere_rot[0]);
  get_config_file()->get_value("sphere_rot_y", sphere_rot[1]);
  get_config_file()->get_value("sphere_rot_z", sphere_rot[2]);
  
  double box_elasticity = 0.2f;
  double sphere_elasticity = 0.2f;
  double world_elasticity = 1.0f;
  get_config_file()->get_value("box_elasticity", box_elasticity);
  get_config_file()->get_value("sphere_elasticity", sphere_elasticity);
  get_config_file()->get_value("world_elasticity", world_elasticity);

  double box_static_friction = 0.5f;
  double box_dynamic_friction = 0.3f;
  double sphere_static_friction = 0.4f;
  double sphere_dynamic_friction = 0.2f;
  double world_static_friction = 0.7f;
  double world_dynamic_friction = 0.3f;
  get_config_file()->get_value("box_static_friction", box_static_friction);
  get_config_file()->get_value("box_dynamic_friction", box_dynamic_friction);
  get_config_file()->get_value("sphere_static_friction", sphere_static_friction);
  get_config_file()->get_value("sphere_dynamic_friction", sphere_dynamic_friction);
  get_config_file()->get_value("world_static_friction", world_static_friction);
  get_config_file()->get_value("world_dynamic_friction", world_dynamic_friction);

  Object* obj = 0;  
  for (int i = 0 ; i < num_cars ; ++i){
    create_car();
  }
  for (int i = 0 ; i < num_walls ; ++i){
    create_wall();
  }
  for (int i = 0 ; i < num_boxes ; ++i){
    double mass = 100.0f;
    double scale = 1.0f - (0.5f * i) / num_boxes;

    obj = new Box(&m_physics, 2.5f * scale, 2.0f * scale, 0.7f * scale, mass, true);
    obj->elasticity() = box_elasticity;
    obj->static_friction() = box_static_friction;
    obj->dynamic_friction() = box_dynamic_friction;

    obj->set_velocity(box_vel);
    obj->set_rotation(box_rot);
    obj->set_position(object_offset + 
		              coord( 0.0f, 0.0f, 0.7f + i * box_separation_z));

    m_objects.push_back(obj);
    m_render_manager.add_object(obj);
    m_physics.add_body(obj);
  }
  
  for (int i = 0 ; i < num_spheres ; ++i){
    double mass = 50.0f;
    double scale = 1.0f - (0.5f * i) / num_spheres;
    obj = new Ellipsoid(&m_physics, 1.0f * scale, 1.0f * scale, 1.0f * scale, 16, 10, mass);
    obj->elasticity() = sphere_elasticity;
    obj->static_friction() = sphere_static_friction;
    obj->dynamic_friction() = sphere_dynamic_friction;

    obj->set_velocity(sphere_vel);
    obj->set_rotation(sphere_rot);
    obj->set_position(object_offset + coord( 0, -2, 2.5f + i * 2.0f));
    m_objects.push_back(obj);
    m_render_manager.add_object(obj);
    m_physics.add_body(obj);
  }


  SimpleHeightMap * main_terrain;
  main_terrain = new Flat_simple_heightmap(1000.0, 1000.0, 2, 2);
  main_terrain->elasticity() = world_elasticity;
  main_terrain->static_friction() = world_static_friction;
  main_terrain->dynamic_friction() = world_dynamic_friction;

  m_render_manager.set_terrain(main_terrain);

  // and add to the physics
  m_physics.add_heightmap(main_terrain);

  // set up the camera
  coord camera_pos(0, 0, 2);
  get_config_file()->get_value("camera_pos_x", camera_pos[0]);
  get_config_file()->get_value("camera_pos_y", camera_pos[1]);
  get_config_file()->get_value("camera_pos_z", camera_pos[2]);

  m_camera.set_position(object_offset + camera_pos);
  m_camera.set_up(vec3d(0.0f, 0.0f, 1.0f));
  m_camera.set_target_object(m_objects[0]);
}

void initialise_car(Car * car){
  coord start_pos(0.0f, 4.0f, 2.0f);
  Simulator::get_config_file()->get_value("car_pos_x", start_pos[0]);
  Simulator::get_config_file()->get_value("car_pos_y", start_pos[1]);
  Simulator::get_config_file()->get_value("car_pos_z", start_pos[2]);

  car->set_position(start_pos);
  car->set_mass(400.0f);
  car->set_body_inertia(100.0f, 100.0f, 100.0f);
  car->static_friction() = 0.2f;
  car->dynamic_friction() = 0.1f;
  car->elasticity() = 0.1f;
}

Car* Simulator::create_car(){
  Car* car = new Car(&m_physics, true, true);
  initialise_car(car);

  m_objects.push_back(car);
  int index = m_objects.size() - 1;
  m_cars.push_back(car);
  m_render_manager.add_object(car);
  m_physics.add_body(car);
	
  car->set_position(object_offset + coord(0.0f, 5 + index*4.0f, 2.0f));
  m_car_ai->add_controlled_car(car);
  TRACE("total number of objects = %d\n", m_objects.size());
  return car;
}

// create_wall
void Simulator::create_wall(){
  int n_along = 5, n_up = 8;
  // wall goes along the x dir
  double dx = 2.0f, dy = 0.8f, dz = 0.8f;
  double mass = 10.0f * dx * dy * dz;
  static coord start(0.0f, 0.0f, 0.0f);
  for (int j = 0 ; j < n_up ; ++j){
    for (int i = 0 ; i < n_along ; ++i){
      if ((j % 2) && (i == 0)) continue;
      double offset = 0.0f;
      if(j%2) offset = -dx * 0.5f;

      Box * box = new Box(&m_physics,
        0.99f * dx, dy, dz,
        mass, true);
      box->elasticity() = 0.0f;
      box->static_friction() = 0.7f;
      box->dynamic_friction() = 0.5f;

      box->set_position(start + vec3d(i * dx + offset, 0.0f, (0.5f + j) * dz));

      m_objects.push_back(box);
      m_render_manager.add_object(box);
      m_physics.add_body(box);
    }
  }

  start += vec3d(0.0f, 5 * dy, 0.0f);
}

// create_box
Box * Simulator::create_box(){
  double mass = 100.0f;
  double box_elasticity = 0.2f;
  get_config_file()->get_value("box_elasticity", box_elasticity);

  double box_static_friction = 0.5f;
  double box_dynamic_friction = 0.3f;
  get_config_file()->get_value("box_static_friction", box_static_friction);
  get_config_file()->get_value("box_dynamic_friction", box_dynamic_friction);

  double min_side = 0.2f;
  double max_side = 2.0f;
  Box * obj = new Box(&m_physics, 
                ranged_random(min_side, max_side), 
                ranged_random(min_side, max_side), 
                ranged_random(min_side, max_side), 
                mass, true);
  obj->elasticity() = box_elasticity;
  obj->static_friction() = box_static_friction;
  obj->dynamic_friction() = box_dynamic_friction;

  obj->set_position(object_offset + coord( 0.0f, 0.0f, 10.0f));

  m_objects.push_back(obj);
  m_render_manager.add_object(obj);
  m_physics.add_body(obj);

  TRACE("total number of objects = %d\n", m_objects.size());
  return obj;
}



