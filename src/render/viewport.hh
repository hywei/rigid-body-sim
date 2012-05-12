#ifndef VIEWPORT_HPP
#define VIEWPORT_HPP

#include <string>

class RenderManager;
class Camera;

class Viewport{
public:
  Viewport(const char * name,
           RenderManager * render_manager,
           Camera * camera,
           double offset_x, double offset_y,
           double width, double height);

  void display();

private:
  std::string m_name;
  RenderManager * m_render_manager;
  Camera * m_camera;

  double m_offset_x, m_offset_y;
  double m_width, m_height;

  double m_clip_near;
  double m_clip_far;
  double m_fov;
};

#endif

