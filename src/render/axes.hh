#ifndef AXES_HPP
#define AXES_HPP

#include "render_object.hh"

class Axes : public RenderObject
{
public:
  Axes(double length);

  void display_object();

  const coord & get_render_position() const {return m_position;}
  const mat3d & get_render_orientation() const {return m_orientation;}
  void getBoundingSphere(coord & pos, double & radius) {pos = m_position; radius = m_length;}

private:
  coord m_position;
  mat3d m_orientation;
  double m_length;
};

#endif
