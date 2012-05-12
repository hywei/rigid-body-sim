#ifndef COLLISION_BODY_MESH_HPP
#define COLLISION_BODY_MESH_HPP

#include "../common/types.hh"
#include "../common/grid3d.hh"

class CollBody;

/// structure used to initialise the mesh
struct CollTriangle{
  CollTriangle() {}
  CollTriangle(const coord& v0, const coord& v1, const coord& v2) 
    : v0(v0), v1(v1), v2(v2) {}
  coord v0, v1, v2;
};

/// interface used when populating the mesh
class CollisionMeshObject{
public:
  CollisionMeshObject() {}
  virtual ~CollisionMeshObject() {}

  /// if the second version of initialise is used the derived class should implement this
  /// to return the relevant info. It should return true (to indicate the info has been set).
  virtual bool get_mesh_info(const coord& pos, bool& inside, vec3d& vec2suf) const=0;
};


class CollBodyMesh{
public:
  CollBodyMesh();

  /// initialises the mesh. extra_frac indicates hw much bigger the mesh should be
  /// than the bounding volume. The triangle are passed in in the body coordinates.
  /// If body is zero, the triangle mesh MUST be closed, as it is used to populate 
  /// the mesh.
  /// If body is non-zero it will be used to populate the mesh by calls to 
  /// get_mesh_info
  void constructMesh(int nx, int ny, int nz, 
                      double extra_frac, 
                      const std::vector<CollTriangle>& triangles,
                      const CollisionMeshObject* body);

  /// clones the mesh passed in
  void constructMesh(const CollBodyMesh& mesh);

  /// Returns the info at the specified position (in body coords). Direction (normalised) and
  /// distance to body surface are returned by reference. dist will be -ve if outside. retval indicates
  /// if the position is inside the body.
  /// if you want an accurate distance when outside, specify the flag
  bool getPointInfo(coord p, vec3d& dir, double& dist) const;
  inline double getSqrDist2BoundingBox(const coord & pos) const;
  double getBoundingRadius() const {return bounding_radius_;}

  /// displays the mesh in terms of the vectors
  void display_object(){}

private:
  /// helper fn to walk through the triangles and populate our mesh
  void populateMesh(const std::vector<CollTriangle>& triangles);
  void populateMesh(const CollisionMeshObject& body);

  /// nx etc are the number of _points_ in each direction, so 
  /// max_x_ = min_x_ + (nx_ - 1) * dx_
  int nx_, ny_, nz_;
  double min_x_, min_y_, min_z_;
  double max_x_, max_y_, max_z_;
  double dx_, dy_, dz_;
  double bounding_radius_;
  struct Datum{
    vec3d dir;  // normalised direction pointing away from the body centre.
    double  dist; // distance to the inside - +ve if inside
  };

  Grid3D<Datum> data_;
};

inline double CollBodyMesh::getSqrDist2BoundingBox(const coord & pos) const{
  // closest point on the box
  coord box_pos(pos[0], pos[1], pos[2]);
  if (pos[0] > max_x_) box_pos[0] = max_x_;
  else if (pos[0] < min_x_) box_pos[0] = min_x_;
  if (pos[1] > max_y_) box_pos[1] = max_y_;
  else if (pos[1] < min_y_) box_pos[1] = min_y_;
  if (pos[2] > max_z_) box_pos[2] = max_z_;
  else if (pos[2] < min_z_) box_pos[2] = min_z_;
  return (pos - box_pos).mag2();
}


#endif
