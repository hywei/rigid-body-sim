#include "axes.hh"

#include "glut_utils.hh"

Axes::Axes(double length) :
m_position(0),
m_orientation(matrix3_identity()),
m_length(length)
{}

void Axes::display_object(){
  GLCOLOR3(1.0f, 0.0f, 0.0f);
  glBegin(GL_LINES);
  GLVERTEX3(0.0f, 0.0f, 0.0f);
  GLVERTEX3(m_length, 0.0f, 0.0f);
  glEnd();

  GLCOLOR3(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  GLVERTEX3(0.0f, 0.0f, 0.0f);
  GLVERTEX3(0.0f, m_length, 0.0f);
  glEnd();

  GLCOLOR3(0.0f, 0.0f, 1.0f);
  glBegin(GL_LINES);
  GLVERTEX3(0.0f, 0.0f, 0.0f);
  GLVERTEX3(0.0f, 0.0f, m_length);
  glEnd();
}

