#ifndef BOX_HPP
#define BOX_HPP

#include "object.hh"

// Represents a regular box. collision mesh subdivision is taken from
// the config file.
class Box : public Object, public CollisionMeshObject{
public:
  // box can be solid or just a shell - affects the inertia
  Box(Physics* physics, double side_x, double side_y, double side_z, double mass, bool solid);

  // RigidBody fns
  CollBody* collision_body() { return &m_collision_body; }

  void add_external_forces(double dt);
  
  void display_object();

  void getBoundingSphere(coord & pos, double & radius)
  {m_collision_body.getBoundingSphere(pos, radius);}

  // colour must be set before first display...
  void set_colour(const vec3d & col) {m_colour = col;}

  // used to set up the collision mesh - from CollisionMeshObject
  bool get_mesh_info(const coord& pos, 
                     bool& inside,
                     vec3d& vector_to_surface) const;
private:
  void draw_triangles() const;

private:
  double m_side_x, m_side_y, m_side_z;
  
  CollBody m_collision_body;
  std::vector<CollTriangle> m_triangles;
  
  vec3d m_colour;  
};

#endif
