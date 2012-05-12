#include "ellipsoid.hh"
#include "../mainwindow/glut_app.hh"
#include "../mainwindow/config_file.hh"
#include <vector>

using namespace std;


// Ellipsoid
Ellipsoid::Ellipsoid(Physics * physics, 
                     double diameter_x, double diameter_y, double diameter_z,
                     int slices, int stacks,
                     double mass)
:Object(physics), m_collision_body(this), 
m_radius_x(diameter_x * 0.5f), m_radius_y(diameter_y * 0.5f),
m_radius_z(diameter_z * 0.5f), m_collision_enabled(true){
  TRACE_FILE_IF(ONCE_1)
    TRACE("Creating Ellipsoid\n");
  set_mass(mass);
  double Ix = (1.0f / 5.0f) * mass * (m_radius_y * m_radius_y + m_radius_z * m_radius_z);
  double Iy = (1.0f / 5.0f) * mass * (m_radius_x * m_radius_x + m_radius_z * m_radius_z);
  double Iz = (1.0f / 5.0f) * mass * (m_radius_x * m_radius_x + m_radius_y * m_radius_y);
  set_body_inertia(Ix, Iy, Iz);

  int mesh_num = 10;
  GlutApp::get_config_file()->get_value("ellipsoid_mesh_num", mesh_num);

  vector<coord> points;
  vector<coord> edge_points;
  vector<CollEdge> edges;
  
  // do the top and bottom separately since they have degenerate triangles
  for (int stack = 0 ; stack < stacks-1 ; ++stack){
    for (int slice = 0 ; slice < slices ; ++slice){
      // four points of interest, labelled from left to right
      // "longitude"
      double long0 = ((0.0f + slice) / (slices - 1.0f)) * 360.0f;
      double long1 = ((0.0f + slice) / (slices - 1.0f)) * 360.0f;
      double long2 = ((1.0f + slice) / (slices - 1.0f)) * 360.0f;
      double long3 = ((1.0f + slice) / (slices - 1.0f)) * 360.0f;
      // "latitude"
      double lat0 = -90.0f + ((0.0f + stack) / (stacks - 1.0f)) * 180.0f;
      double lat1 = -90.0f + ((1.0f + stack) / (stacks - 1.0f)) * 180.0f;

      // the "reduced radius"
      double rp0 = cos_deg(lat0);
      double rp1 = cos_deg(lat1);
      double rp2 = rp0;
      double rp3 = rp1;

      double sin_lat0 = sin_deg(lat0);
      double sin_lat1 = sin_deg(lat1);
      double sin_lat2 = sin_lat0;
      double sin_lat3 = sin_lat1;

      coord pos[4] = {
        coord(rp0 * cos_deg(long0), rp0 * sin_deg(long0), sin_lat0),
        coord(rp1 * cos_deg(long1), rp1 * sin_deg(long1), sin_lat1),
        coord(rp2 * cos_deg(long2), rp2 * sin_deg(long2), sin_lat2),
        coord(rp3 * cos_deg(long3), rp3 * sin_deg(long3), sin_lat3) 
      };
      
      for (int j = 0 ; j < 4 ; ++j){
        pos[j][0] *= m_radius_x;
        pos[j][1] *= m_radius_y;
        pos[j][2] *= m_radius_z;
      }

      if (stack == 0){
        if (slice == 0) points.push_back(pos[0]);          
        m_triangles.push_back(CollTriangle(pos[2], pos[3], pos[1]));
      }else if (stack == stacks - 2) {
        points.push_back(pos[0]);
        if (slice == 0) points.push_back(pos[1]);
        m_triangles.push_back(CollTriangle(pos[0], pos[2], pos[1]));
      }else {
        points.push_back(pos[0]);
        m_triangles.push_back(CollTriangle(pos[0], pos[2], pos[1]));
        m_triangles.push_back(CollTriangle(pos[2], pos[3], pos[1]));
      }
    }
  }
  m_collision_body.initialise(mesh_num, mesh_num, mesh_num, 
                              1.1f,
                              m_triangles,
                              points,
                              edge_points,
                              edges,
                              this);
  m_colour_front = vec3d(1.0f, ranged_random(0.2f, 1.0f), 0.0f);
  m_colour_back = vec3d(0.0f, ranged_random(0.2f, 1.0f), 1.0f);
}


// display_object
void Ellipsoid::display_object(){
    GLuint display_list_num = glGenLists(1);
    glNewList(display_list_num, GL_COMPILE);
    int num_triangles = m_triangles.size();
    double frac;
    vec3d col;
    bool sharp_col = true;
    
    glBegin(GL_TRIANGLES);
    for (int i = 0 ; i < num_triangles ; ++i){
      vec3d normal = cross(m_triangles[i].v1 - m_triangles[i].v0, 
                             m_triangles[i].v2 - m_triangles[i].v0);
      normal.normalise();
      GLNORMAL3V(normal.get_data());
      if (sharp_col){
        if (m_triangles[i].v0[0] > 0.0f)
          GLCOLOR4(m_colour_front[0], m_colour_front[1], m_colour_front[2], 0.5f);
        else
          GLCOLOR4(m_colour_back[0], m_colour_back[1], m_colour_back[2], 0.5f);
      }else{
        frac = 0.5f + 0.5f * (m_triangles[i].v0[0] / m_radius_x);
        col = m_colour_front * frac + m_colour_back * (1 - frac);
        GLCOLOR4(col[0], col[1], col[2], 0.5f);
      }
      GLVERTEX3V(m_triangles[i].v0.get_data());

      if (sharp_col){
        if (m_triangles[i].v1[0] > 0.0f)
          GLCOLOR4(m_colour_front[0], m_colour_front[1], m_colour_front[2], 0.5f);
        else
          GLCOLOR4(m_colour_back[0], m_colour_back[1], m_colour_back[2], 0.5f);
      }else {
        frac = 0.5f + 0.5f * (m_triangles[i].v1[0] / m_radius_x);
        col = m_colour_front * frac + m_colour_back * (1 - frac);
        GLCOLOR4(col[0], col[1], col[2], 0.5f);
      }
      GLVERTEX3V(m_triangles[i].v1.get_data());

      if (sharp_col){
        if (m_triangles[i].v2[0] > 0.0f)
          GLCOLOR4(m_colour_front[0], m_colour_front[1], m_colour_front[2], 0.5f);
        else
          GLCOLOR4(m_colour_back[0], m_colour_back[1], m_colour_back[2], 0.5f);
      }else{
        frac = 0.5f + 0.5f * (m_triangles[i].v2[0] / m_radius_x);
        col = m_colour_front * frac + m_colour_back * (1 - frac);
        GLCOLOR4(col[0], col[1], col[2], 0.5f);
      }
      GLVERTEX3V(m_triangles[i].v2.get_data());
    }
    glEnd();
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
}


// getBoundingSphere
void Ellipsoid::getBoundingSphere(coord & pos, double & radius){
  m_collision_body.getBoundingSphere(pos, radius);
}


// add_external_forces
void Ellipsoid::add_external_forces(double dt){
  add_gravity();
}


// get_mesh_info
bool Ellipsoid::get_mesh_info(const coord & pos, 
                              bool & inside,
                              vec3d & vector_to_surface) const{
  Ellipsoid_shape e(vec3d(m_radius_x, m_radius_y, m_radius_z));
  coord closest;

  double dist = distance_sqr(pos, e, closest);

  if (dist < 0.0f)
    TRACE("ow - unable to get distance to ellipsoid\n");

  vector_to_surface = closest - pos;

  double v = (pos[0] / m_radius_x) * (pos[0] / m_radius_x) + 
             (pos[1] / m_radius_y) * (pos[1] / m_radius_y) + 
             (pos[2] / m_radius_z) * (pos[2] / m_radius_z);
  inside = (v > 1.0) ? false : true;
  return true;
}
