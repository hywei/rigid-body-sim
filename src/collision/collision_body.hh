#ifndef COLLISION_BODY_HPP
#define COLLISION_BODY_HPP

#include "types.hh"
#include "distance.hh"
#include "collision_body_mesh.hh"
#include "render_object.hh"

#include <vector>

class RigidBody;
class CollisionMeshObject;


struct CollEdge{
  CollEdge() {}
  CollEdge(int i0, int i1, int num_samples) 
    : i0(i0), i1(i1), num_samples(num_samples > 2 ? num_samples : 2){}  
  int i0, i1, num_samples;
};

/// Holds the data used in testing for collisions
class CollBody{
public:
  CollBody(RigidBody* rigid_body);
  RigidBody* rigid_body() const {return rigid_body_;}

  /// initialise the body. If the flag is set the mesh will be constructed 
  /// based on the triangle list, and will be mesh_extra_frac times bigger 
  /// in each direction. Otherwise the body will be interrogated for each
  // point on the mesh using get_mesh_info(...)
  void initialise(int mesh_nx, int mesh_ny, int mesh_nz, 
                  double mesh_extra_frac, 
                  const std::vector<CollTriangle>& triangles_for_mesh,
                  const std::vector<coord>& points,
                  const std::vector<coord>& edge_points,
                  const std::vector<CollEdge>& edges,
                  const CollisionMeshObject* mesh_object = 0);

  /// initialises by cloning the mesh passed in
  void initialise(int mesh_nx, int mesh_ny, int mesh_nz, 
                  double mesh_extra_frac, 
                  const std::vector<CollTriangle> & triangles,
                  const std::vector<coord> & points,
                  const std::vector<coord> & edge_points,
                  const std::vector<CollEdge> & edges,
                  const CollBodyMesh& mesh);

  void setPosition(coord pos){ pos_ = pos; }  
  void setOrientation(const mat3d& orientation){
    orientation_ = orientation;
    inv_orientation_ = transpose(orientation);
  }
  
  void calculateWorldProperties();  

  bool getPointInfo(coord pos, vec3d& dir, double& dist) const
  { return body_mesh_.getPointInfo(pos, dir, dist); }

  double getSqrDist2BoundingBox(const coord & pos) const
  { return body_mesh_.getSqrDist2BoundingBox(pos); }

  /// the world points. Assumes calculate_world_properties has been called.
  const std::vector<coord>& getWorldPoints() const {return world_points_;}

  /// the world edge points. Assumes calculate_world_properties has been called.
  const std::vector<coord>& getWorldEdgePoints() const {return world_edge_points_;}

  /// the edges (indexes into world_edge_points)
  const std::vector<CollEdge>& getCollEdges() const {return edges_;}

  /// Intended for internal use by Physics - we get told about the collisions
  /// we're involved with. Used to resolve penetrations.
  /// The value is the index into the list of collisions owned by physics.
  std::vector<int> & getCollisions() {return collisions_;}

  // Inherited from RenderObject
  void display_object();
  inline void getBoundingSphere(coord & pos, double & radius)const;
  const coord& getPosition() const {return pos_;}
  const mat3d& getOrientation() const {return orientation_;}
  const mat3d& getInvOrientation() const {return inv_orientation_;}

  const CollBodyMesh& getCollisionMesh() const {return body_mesh_;}

private:
  RigidBody* rigid_body_;
  coord pos_;
  mat3d orientation_, inv_orientation_;

  /// points in this body used for testing against other bodies.
  std::vector<coord> body_points_;
  /// points in this body used by the edges
  std::vector<coord> body_edge_points_;
  /// edges in this body used for testing against other bodies
  std::vector<CollEdge> edges_;
  /// points converted to world space
  std::vector<coord> world_points_;
  /// edge points converted to world space
  std::vector<coord> world_edge_points_;

  /// the volume information used when other bodies test their points etc
  CollBodyMesh body_mesh_;

  /// all the collisions we're involved with
  std::vector<int> collisions_;
};

// getBoundingSphere
inline void CollBody::getBoundingSphere(coord& pos, double& radius) const{
  pos = pos_;
  radius = body_mesh_.getBoundingRadius();
}

#endif


