#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "rigid_body.hh"
#include "render_object.hh"

/// represents an object that can be rendered and does physics
class Object : public RigidBody, public RenderObject{
public:
  Object(Physics * physics) : RigidBody(physics) {}
  virtual const coord & get_render_position() const {return get_position();}
  virtual const mat3d & get_render_orientation() const {return get_orientation();}
};

#endif
