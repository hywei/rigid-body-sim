#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

#include "object.hh"

// Represents a regular Ellipsoid. collision mesh subdivision is taken from the config file
// defaults to 10^3
class Ellipsoid : public Object, public CollisionMeshObject{
public:
  // generate an ellipsoid based on slices and stacks
  Ellipsoid(Physics * physics, double diameter_x, double diameter_y, double diameter_z,
            int slices, int stacks,
            double mass);

  // RigidBody fns
  CollBody * collision_body() { 
    if (m_collision_enabled) 
      return &m_collision_body; 
    else 
      return 0;
  }

  /// inherited from RigidBody
  void add_external_forces(double dt);
  
  void display_object();
  void getBoundingSphere(coord & pos, double & radius);

  void enable_collisions(bool enable) {m_collision_enabled = enable;}

  // used to set up the collision mesh - from CollisionMeshObject
  bool get_mesh_info(const coord & pos, 
                     bool & inside,
                     vec3d & vector_to_surface) const;
private:
  CollBody m_collision_body;
  std::vector<CollTriangle> m_triangles;

  // gradient in colour from front to back so we can see rotations
  vec3d m_colour_front;
  vec3d m_colour_back;

  double m_radius_x, m_radius_y, m_radius_z;
  bool m_collision_enabled;
};

#endif
