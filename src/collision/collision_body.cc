#include "collision_body.hh"
#include "../render/glut_utils.hh"


// CollBody
CollBody::CollBody(RigidBody * rigid_body){
  rigid_body_ = rigid_body;
}


// initialise
void CollBody::initialise(int mesh_nx, int mesh_ny, int mesh_nz, 
                                double mesh_extra_frac, 
                                const std::vector<CollTriangle> & triangles,
                                const std::vector<coord> & points,
                                const std::vector<coord> & edge_points,
                                const std::vector<CollEdge> & edges,
                                const CollisionMeshObject * mesh_object){
  body_mesh_.constructMesh(mesh_nx, mesh_ny, mesh_nz, mesh_extra_frac, triangles, mesh_object);

  body_points_ = points;
  body_edge_points_ = edge_points;
  edges_ = edges;

  pos_.set_to(0.0f);
  orientation_ = matrix3_identity();

  calculateWorldProperties();
}


void CollBody::initialise(int mesh_nx, int mesh_ny, int mesh_nz, 
                                double mesh_extra_frac, 
                                const std::vector<CollTriangle> & triangles,
                                const std::vector<coord> & points,
                                const std::vector<coord> & edge_points,
                                const std::vector<CollEdge> & edges,
                                const CollBodyMesh & mesh){  
  body_mesh_.constructMesh(mesh);

  body_points_ = points;
  body_edge_points_ = edge_points;
  edges_ = edges;

  pos_.set_to(0.0f);
  orientation_ = matrix3_identity();

  calculateWorldProperties();
}


// calculate_world_properties
void CollBody::calculateWorldProperties(){
  world_points_.resize(body_points_.size());
  for (size_t i = 0 ; i < body_points_.size(); ++i){
    world_points_[i] = pos_ + orientation_ * body_points_[i];
  }
  
  world_edge_points_.resize(body_edge_points_.size());
  for (size_t i = 0 ; i < body_edge_points_.size(); ++i){
    world_edge_points_[i] = pos_ + orientation_ * body_edge_points_[i];
  }
}

// display_object
void CollBody::display_object(){
  Save_GL_state state;
  apply_transformation(pos_, orientation_);
  //  body_mesh_.display_object();

}
