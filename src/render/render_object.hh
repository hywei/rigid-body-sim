#ifndef RENDER_OBJECT_HPP
#define RENDER_OBJECT_HPP

#include "types.hh"

class RenderObject{
public:
  RenderObject() {};
  virtual ~RenderObject() {};
 
  /// This should draw the object in its local reference frame
  virtual void display_object() = 0;

  /// return a bounding sphere - used for culling (maybe!)
  virtual void getBoundingSphere(coord & pos, double & radius) = 0;

  /// return a position suitable for use as a camera focus
  virtual const coord& get_render_position() const = 0;

  /// return an orientation suitable for use when calculating camera orientation
  virtual const mat3d& get_render_orientation() const = 0;
};

#endif
