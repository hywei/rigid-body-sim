#include "box.hh"
#include "../mainwindow/glut_app.hh"
#include "../mainwindow/config_file.hh"
#include <vector>

using namespace std;

// Box
Box::Box(Physics * physics, double side_x, double side_y, double side_z, double mass, bool solid)
  : Object(physics),
    m_collision_body(this),
    m_side_x(side_x),
    m_side_y(side_y),
    m_side_z(side_z){
  TRACE_FILE_IF(ONCE_1)
    TRACE("Creating box\n");
  set_mass(mass);
  double Ix = (1.0f / 3.0f) * mass * (side_y * side_y + side_z * side_z);
  double Iy = (1.0f / 3.0f) * mass * (side_x * side_x + side_z * side_z);
  double Iz = (1.0f / 3.0f) * mass * (side_x * side_x + side_y * side_y);
  set_body_inertia(Ix, Iy, Iz);

  int mesh_num = 16;
  int num_edge_samples = 6;
  int num_extra_face_points_per_side = 5;
  GlutApp::get_config_file()->get_value("box_mesh_num", mesh_num);
  GlutApp::get_config_file()->get_value("box_num_edge_samples", num_edge_samples);
  GlutApp::get_config_file()->get_value("box_num_extra_face_points_per_side", num_extra_face_points_per_side);

  vector<coord> points;
  vector<coord> edge_points;
  vector<CollEdge> edges;

  double dx = side_x * 0.5f;
  double dy = side_y * 0.5f;
  double dz = side_z * 0.5f;

  enum {TFL = 0, TBL = 1, TBR = 2, TFR = 3, BFL = 4, BBL = 5, BBR = 6, BFR = 7};
  points.resize(8);
  points[TFL] = coord( dx,  dy,  dz);
  points[TBL] = coord(-dx,  dy,  dz);
  points[TBR] = coord(-dx, -dy,  dz);
  points[TFR] = coord( dx, -dy,  dz);

  points[BFL] = coord( dx,  dy, -dz);
  points[BBL] = coord(-dx,  dy, -dz);
  points[BBR] = coord(-dx, -dy, -dz);
  points[BFR] = coord( dx, -dy, -dz);

  edge_points = points;

  edges.resize(12);
  edges[0] = CollEdge(TFL, TBL, num_edge_samples);
  edges[1] = CollEdge(TBL, TBR, num_edge_samples);
  edges[2] = CollEdge(TBR, TFR, num_edge_samples);
  edges[3] = CollEdge(TFR, TFL, num_edge_samples);

  edges[4] = CollEdge(BFL, BBL, num_edge_samples);
  edges[5] = CollEdge(BBL, BBR, num_edge_samples);
  edges[6] = CollEdge(BBR, BFR, num_edge_samples);
  edges[7] = CollEdge(BFR, BFL, num_edge_samples);

  edges[8] = CollEdge(TFL, BFL, num_edge_samples);
  edges[9] = CollEdge(TBL, BBL, num_edge_samples);
  edges[10] = CollEdge(TBR, BBR, num_edge_samples);
  edges[11] = CollEdge(TFR, BFR, num_edge_samples);

  m_triangles.resize(12);
  // top
  m_triangles[0] = CollTriangle(edge_points[TFL], edge_points[TBL], edge_points[TBR]);
  m_triangles[1] = CollTriangle(edge_points[TFL], edge_points[TBR], edge_points[TFR]);

  // bottom
  m_triangles[2] = CollTriangle(edge_points[BFL], edge_points[BBR], edge_points[BBL]);
  m_triangles[3] = CollTriangle(edge_points[BFL], edge_points[BFR], edge_points[BBR]);

  // left
  m_triangles[4] = CollTriangle(edge_points[BFL], edge_points[BBL], edge_points[TBL]);
  m_triangles[5] = CollTriangle(edge_points[BFL], edge_points[TBL], edge_points[TFL]);

  // right
  m_triangles[6] = CollTriangle(edge_points[BFR], edge_points[TFR], edge_points[TBR]);
  m_triangles[7] = CollTriangle(edge_points[BFR], edge_points[TBR], edge_points[BBR]);

  // front
  m_triangles[8] = CollTriangle(edge_points[TFL], edge_points[BFR], edge_points[BFL]);
  m_triangles[9] = CollTriangle(edge_points[TFL], edge_points[TFR], edge_points[BFR]);

  // back
  m_triangles[10] = CollTriangle(edge_points[TBL], edge_points[BBR], edge_points[TBR]);
  m_triangles[11] = CollTriangle(edge_points[TBL], edge_points[BBL], edge_points[BBR]);

  // add some extra points to the box faces
  int num = num_extra_face_points_per_side; // number of points along each side
  int n = num-1;
  for (int i = 0 ; i < num ; ++i){
      for(int j = 0 ; j < num ; ++j){
          for(int k = 0 ; k < num ; ++k){
              if ( (i == 0) || (i == n )||
                   (j == 0) || (j == n )||
                   (k == 0) || (k == n ) ){
                  bool on_x_end = ( (i == 0) || (i == n) );
                  bool on_y_end = ( (j == 0) || (j == n) );
                  bool on_z_end = ( (k == 0) || (k == n) );
                  if ( (on_x_end && (!on_y_end && !on_z_end)) ||
                       (on_y_end && (!on_z_end && !on_x_end)) ||
                       (on_z_end && (!on_x_end && !on_y_end)) ){
                      double x = -dx + (side_x * i) / n;
                      double y = -dy + (side_y * j) / n;
                      double z = -dz + (side_z * k) / n;
                      //points.push_back(coord(x, y, z));
                  }
              }
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
  m_colour = vec3d(ranged_random(0.2f, 1.0f),
                     ranged_random(0.2f, 1.0f), 
                     ranged_random(0.2f, 1.0f));
}


// draw_triangles
void Box::draw_triangles() const{
  glBegin(GL_TRIANGLES);
  int num_triangles = m_triangles.size();
  for (int i = 0 ; i < num_triangles ; ++i){
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

void Box::display_object(){
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
}


// add_external_forces
void Box::add_external_forces(double dt){
  add_gravity();
}


// get_mesh_info, pos to box, 
bool Box::get_mesh_info(const coord& pos, 
                        bool& inside,
                        vec3d& vector_to_surface) const{
  
  vec3d delta(m_side_x * 0.5f, m_side_y * 0.5f, m_side_z * 0.5f);
  coord face_point;
  double dists[3];
  inside = true;
  
  for(int i = 0; i < 3 ; ++i){
    if (pos[i] > delta[i]){
      face_point[i] = delta[i];
      inside = false;
    }else if (pos[i] < -delta[i]){
      face_point[i] = -delta[i];
      inside = false;
    }else{
      if (pos[i] > 0.0f)
        face_point[i] = delta[i];
      else
        face_point[i] = -delta[i];
    }
    dists[i] = fabs(face_point[i] - pos[i]);
  }
  int best_i = 0;
  // work out the component that leads to the closest face
  if (dists[1] < dists[best_i]) best_i = 1;
  if (dists[2] < dists[best_i]) best_i = 2;
  for (int i = 0 ; i < 3 ; ++i){
    if (i != best_i){
      face_point[i] = pos[i];
      if (face_point[i] > delta[i])
        face_point[i] = delta[i];
      else if (face_point[i] < -delta[i])
        face_point[i] = -delta[i];
    }
  }
  vector_to_surface = face_point - pos;
  return true;
}
