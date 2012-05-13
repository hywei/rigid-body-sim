#include "car.hh"
#include "wheel.hh"
#include "../mainwindow/glut_app.hh"
#include "../simulation/physics.hh"
using namespace std;

static void add_quad(vector<CollTriangle> & triangles,
                     coord & p0, coord & p1, coord & p2, coord & p3);

Car * Car::m_original_car = 0;

Car::Car(Physics * physics, bool fw_drive, bool rw_drive) :
Object(physics),
m_collision_body(this),
m_fw_drive(fw_drive),
m_rw_drive(rw_drive),
m_dest_steering(0.0),
m_dest_accelerate(0.0),
m_steering(0.0),
m_accelerate(0.0),
m_hbrake(0.0){
  TRACE_FILE_IF(ONCE_1)
    TRACE("Creating Car\n");
  // so we get an update
  physics->add_controller(this);

  // create the car structure - for now just a box
  int mesh_num_x = 32;
  int mesh_num_y = 16;
  int mesh_num_z = 18;
  int num_edge_samples = 6;
  GlutApp::get_config_file()->get_value("car_mesh_num", mesh_num_x);
  GlutApp::get_config_file()->get_value("car_mesh_num", mesh_num_y);
  GlutApp::get_config_file()->get_value("car_mesh_num", mesh_num_z);
  GlutApp::get_config_file()->get_value("car_num_edge_samples", num_edge_samples);

  vector<coord> points;
  vector<coord> edge_points;
  vector<CollEdge> edges;

#define CAR_X_FWD 1.8
#define CAR_X_BACK 2.0
#define CAR_Y 1.0f
#define CAR_Z_UP 0.4f
#define CAR_Z_DOWN 0.2f
  // Front low left
  coord FLL(CAR_X_FWD, CAR_Y, -CAR_Z_DOWN);
  // back low left
  coord BLL(-CAR_X_BACK, CAR_Y, -CAR_Z_DOWN);
  // back low right
  coord BLR(-CAR_X_BACK, -CAR_Y, -CAR_Z_DOWN);
  // Front low right
  coord FLR(CAR_X_FWD, -CAR_Y, -CAR_Z_DOWN);

  // Front high left
  coord FHL(CAR_X_FWD - 0.0, CAR_Y, CAR_Z_UP);
  // back high left
  coord BHL(-CAR_X_BACK, CAR_Y, CAR_Z_UP);
  // back high right
  coord BHR(-CAR_X_BACK, -CAR_Y, CAR_Z_UP);
  // Front high right
  coord FHR(CAR_X_FWD - 0.0, -CAR_Y, CAR_Z_UP);
  // add a cabin

  double C_X_LOW = CAR_X_FWD - 1.0;
  double C_X_HIGH = CAR_X_FWD - 1.5;
  double C_X_BACK_HIGH = -CAR_X_BACK + 0.4;
  double C_Y = CAR_Y - 0.2;
  double C_Z = CAR_Z_UP + 0.7;

  // Front low left
  coord CFLL(C_X_LOW, CAR_Y, CAR_Z_UP);
  // Front low right
  coord CFLR(C_X_LOW, -CAR_Y, CAR_Z_UP);

  // Front high left
  coord CFHL(C_X_HIGH, C_Y, C_Z);
  // back high left
  coord CBHL(C_X_BACK_HIGH, C_Y, C_Z);
  // back high right
  coord CBHR(C_X_BACK_HIGH, -C_Y, C_Z);
  // Front high right
  coord CFHR(C_X_HIGH, -C_Y, C_Z);

  // create the faces
  // front
  add_quad(m_triangles, FLL, FHL, FHR, FLR);
  // left
  add_quad(m_triangles, BLL, BHL, FHL, FLL);
  // right
  add_quad(m_triangles, FLR, FHR, BHR, BLR);
  // back
  add_quad(m_triangles, BLL, BLR, BHR, BHL);
  // bottom
  add_quad(m_triangles, FLL, FLR, BLR, BLL);
  // top
  add_quad(m_triangles, FHL, CFLL, CFLR, FHR);
	
  // cabin

  // front
  add_quad(m_triangles, CFLL, CFHL, CFHR, CFLR);
  // left
  add_quad(m_triangles, CFLL, BHL, CBHL, CFHL);
  // right
  add_quad(m_triangles, CFLR, CFHR, CBHR, BHR);
  // back
  add_quad(m_triangles, CBHR, CBHL, BHL, BHR);
  // top
  add_quad(m_triangles, CFHL, CBHL, CBHR, CFHR);

  // points
  points.push_back(FLL);//0
  points.push_back(FHL);//1
  points.push_back(FLR);//2
  points.push_back(FHR);//3
  points.push_back(BLL);//4
  points.push_back(BHL);//5
  points.push_back(BLR);//6
  points.push_back(BHR);//7

  // cabin
  points.push_back(CFHL);//8
  points.push_back(CFHR);//9
  points.push_back(CBHL);//10
  points.push_back(CBHR);//11
  points.push_back(CFLL);//12
  points.push_back(CFLR);//13

  // edges
  edge_points = points;
  edges.push_back(CollEdge(0, 2, num_edge_samples)); // front bottom
  edges.push_back(CollEdge(1, 3, num_edge_samples)); // front top
  edges.push_back(CollEdge(0, 4, num_edge_samples)); // left bottom
  edges.push_back(CollEdge(2, 6, num_edge_samples)); // right bottom
  edges.push_back(CollEdge(8, 9, num_edge_samples)); // windscreen top
  edges.push_back(CollEdge(9, 11, num_edge_samples)); // cabin top right
  edges.push_back(CollEdge(11, 10, num_edge_samples)); // cabin top back
  edges.push_back(CollEdge(10, 8, num_edge_samples)); // cabin top left
  // cabin struts
  edges.push_back(CollEdge(8, 12, num_edge_samples)); // windscreen left
  edges.push_back(CollEdge(9, 13, num_edge_samples)); // windscreen right
  edges.push_back(CollEdge(10, 5, num_edge_samples)); // rear screen left
  edges.push_back(CollEdge(11, 7, num_edge_samples)); // rear screen right
  // screen base
  edges.push_back(CollEdge(12, 13, num_edge_samples)); // bottom of the windscreen

  edges.push_back(CollEdge(1, 5, num_edge_samples)); // left high
  edges.push_back(CollEdge(3, 7, num_edge_samples)); // right high
  edges.push_back(CollEdge(4, 6, num_edge_samples)); // back bottom
  edges.push_back(CollEdge(5, 7, num_edge_samples)); // back top
  edges.push_back(CollEdge(0, 1, num_edge_samples)); // front left strut
  edges.push_back(CollEdge(2, 3, num_edge_samples)); // front right strut
  edges.push_back(CollEdge(4, 5, num_edge_samples)); // back left strut 
  edges.push_back(CollEdge(6, 7, num_edge_samples)); // back right strut

  // don't need the points at the corners - but put one in each face
  points.clear();

  points.push_back(0.25f * (FLL + FLR + FHL + FHR)); // front
  points.push_back(0.25f * (FLL + FHL + BHL + BLL)); // left
  points.push_back(0.25f * (FLR + FHR + BLR + BHR)); // right
  points.push_back(0.25f * (BLL + BLR + BHL + BHR)); // back
  points.push_back(0.25f * (FLL + FLR + BLL + BLR)); // bottom
  points.push_back(0.25f * (FHL + FHR + CFLL + CFLR)); // bonnet
  points.push_back(0.25f * (CFHL + CFHR + CBHL + CBHR)); // top
  points.push_back(0.25f * (CFHL + CFHR + CFLL + CFLR)); // windscreen
  points.push_back(0.25f * (CBHL + CBHR + BHL + BHR)); // rear window
  points.push_back(0.25f * (CFHL + CBHL + CFLL + BHL)); // left window
  points.push_back(0.25f * (CFHR + CBHR + CFLR + BHR)); // right window

  if (m_original_car){
    m_collision_body.initialise(mesh_num_x, mesh_num_y, mesh_num_z, 
                                1.1f,
                                m_triangles,
                                points,
                                edge_points,
                                edges,
                                m_original_car->collision_body()->getCollisionMesh());
  }else{
    m_collision_body.initialise(mesh_num_x, mesh_num_y, mesh_num_z, 
                                1.1f,
                                m_triangles,
                                points,
                                edge_points,
                                edges);
  }

  double wheel_travel = 0.4f;
  double wheel_radius = 0.4f;
  double wheel_inertia = 0.3 * 0.3 * 20.0f;
  double wheel_friction = 2.0f;
  double wheel_spring = 6000.0f / wheel_travel;
  double wheel_damping = 800.0f; // force per vel
  double wheel_offset_frac = 1.7;
  double wheel_offset_in = wheel_radius * 0.2;

  m_wheels[WHEEL_FR] = new Wheel(
    this, 
    physics,
    coord(CAR_X_FWD - wheel_offset_frac * wheel_radius,
          -CAR_Y + wheel_offset_in,
          -CAR_Z_DOWN), 
    vec3d(0.0f, 0.0f, 1.0f),
    wheel_spring,
    wheel_travel,
    wheel_inertia,
    wheel_radius,
    wheel_friction,
    wheel_damping);
  m_wheels[WHEEL_FL] = new Wheel(
    this,
    physics,
    coord(CAR_X_FWD - wheel_offset_frac * wheel_radius,
             CAR_Y - wheel_offset_in, 
             -CAR_Z_DOWN), 
    vec3d(0.0f, 0.0f, 1.0f),
    wheel_spring,
    wheel_travel,
    wheel_inertia,
    wheel_radius,
    wheel_friction,
    wheel_damping);
  m_wheels[WHEEL_BR] = new Wheel(
    this, 
    physics,
    coord(-CAR_X_BACK + wheel_offset_frac * wheel_radius, -CAR_Y + wheel_offset_in, -CAR_Z_DOWN), 
    vec3d(0.0f, 0.0f, 1.0f),
    wheel_spring,
    wheel_travel,
    wheel_inertia,
    wheel_radius,
    wheel_friction,
    wheel_damping);
  m_wheels[WHEEL_BL] = new Wheel(
    this, 
    physics,
    coord(-CAR_X_BACK + wheel_offset_frac * wheel_radius, 
             CAR_Y - wheel_offset_in,
             -CAR_Z_DOWN), 
    vec3d(0.0f, 0.0f, 1.0f),
    wheel_spring,
    wheel_travel,
    wheel_inertia,
    wheel_radius,
    wheel_friction,
    wheel_damping);
  m_colour = vec3d(ranged_random(0.2f, 1.0f), ranged_random(0.2f, 1.0f), ranged_random(0.2f, 1.0f));

  if (m_original_car == 0)
    m_original_car = this;
}


// ~Car
Car::~Car(){
  delete m_wheels[WHEEL_FL];
  delete m_wheels[WHEEL_FR];
  delete m_wheels[WHEEL_BL];
  delete m_wheels[WHEEL_BR];

  if (m_original_car == this)
    m_original_car = 0;
}


// draw_triangles
void Car::draw_triangles() const{
  glBegin(GL_TRIANGLES);
  int num_triangles = m_triangles.size();
  for(int i = 0 ; i < num_triangles ; ++i){
    vec3d normal = cross(m_triangles[i].v1 - m_triangles[i].v0, 
                           m_triangles[i].v2 - m_triangles[i].v0);
    normal.normalise();
    GLNORMAL3V(normal.get_data());
    GLVERTEX3V(m_triangles[i].v0.get_data());
    GLVERTEX3V(m_triangles[i].v1.get_data());
    GLVERTEX3V(m_triangles[i].v2.get_data());
  }
  glEnd();
}

// display_object
void Car::display_object() {
  TRACE_METHOD_ONLY(FRAME_1);
  GLuint display_list_num = glGenLists(1);
  glNewList(display_list_num, GL_COMPILE);
  GLCOLOR4(m_colour[0], m_colour[1], m_colour[2], 0.5f);
  draw_triangles();
  glEndList();
  
  apply_transformation(get_position(), get_orientation());
  bool translucent = ((RigidBody::m_indicate_frozen_objects) && (get_activity_state() == FROZEN));
  if (translucent){
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  glCallList(display_list_num);
  if (translucent){
    glDisable(GL_BLEND);
  }
  
  for (int i = 0 ; i < 4 ; ++i){
    m_wheels[i]->display_object();
  }
}

// accumulate_force_and_moment
void Car::add_external_forces(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  // use base class to get gravity
  add_gravity();

  // add on from the wheels
  for (int i = 0 ; i < 4 ; ++i){
    m_wheels[i]->add_forces(dt);
  }
}

// update
void Car::update(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  for (int i = 0 ; i < 4 ; ++i){
    m_wheels[i]->update(dt);
  }

  // control inputs
  double delta_accelerate = dt * 4.0;
  double delta_steering = dt * 4.0;

  // update the actual values
  double d_accelerate = m_dest_accelerate - m_accelerate;
  if (d_accelerate > delta_accelerate)
    d_accelerate = delta_accelerate;
  else if (d_accelerate < -delta_accelerate)
    d_accelerate = -delta_accelerate;
  m_accelerate += d_accelerate;

  double d_steering = m_dest_steering - m_steering;
  if (d_steering > delta_steering)
    d_steering = delta_steering;
  else if (d_steering < -delta_steering)
    d_steering = -delta_steering;
  m_steering += d_steering;

  // apply these inputs
  double max_torque = 1200.0;
  if (m_fw_drive && m_rw_drive)
    max_torque *= 0.5;
  if (m_fw_drive){
    m_wheels[WHEEL_FL]->add_torque(max_torque * m_accelerate);
    m_wheels[WHEEL_FR]->add_torque(max_torque * m_accelerate);
  }
  if (m_rw_drive){
    m_wheels[WHEEL_BL]->add_torque(max_torque * m_accelerate);
    m_wheels[WHEEL_BR]->add_torque(max_torque * m_accelerate);
  }

  m_wheels[WHEEL_BL]->set_lock(m_hbrake > 0.5);
  m_wheels[WHEEL_BR]->set_lock(m_hbrake > 0.5);

  // steering angle applies to the inner wheel. The outer one needs to match it
  double max_steer_angle = 30.0;
  unsigned inner, outer;
  if (m_steering > 0.0){
    inner = WHEEL_FL; outer = WHEEL_FR;
  }else{
    inner = WHEEL_FR; outer = WHEEL_FL;
  }

  double alpha = fabs(max_steer_angle * m_steering);
  double angle_sgn = m_steering > 0.0 ? 1.0 : -1.0;

  m_wheels[inner]->set_steer(angle_sgn * alpha);
  double beta;
  if (alpha == 0.0f){
    beta = alpha;
  }else{
    double dx = (m_wheels[WHEEL_FR]->get_pos()[0] - m_wheels[WHEEL_BR]->get_pos()[0]);
    double dy = (m_wheels[WHEEL_FL]->get_pos()[1] - m_wheels[WHEEL_FR]->get_pos()[1]);
    beta = atan2_deg(dy, dx + (dy / tan_deg(alpha)));
  }
  m_wheels[outer]->set_steer(angle_sgn* beta);
}

void Car::set_accelerate(double val){
  m_dest_accelerate = val;
}

void Car::set_steer(double val){
  m_dest_steering = val;
}

void Car::set_hbrake(double val){
  m_hbrake = val;
}

void Car::reset(){
  for (int i = 0 ; i < 4 ; ++i) m_wheels[i]->reset();
}

// get_num_wheels_on_floor
int Car::get_num_wheels_on_floor() const{
  int num = 0;
  unsigned i;
  for (i = 0 ; i < 4 ; ++i)
    num += m_wheels[i]->get_on_floor();
  return num;
}

// add_quad
static void add_quad(vector<CollTriangle> & triangles,
                            coord & p0, coord & p1, coord & p2, coord & p3){
  triangles.push_back(CollTriangle(p0, p1, p2));
  triangles.push_back(CollTriangle(p0, p2, p3));
}

