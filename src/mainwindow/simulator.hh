#ifndef JIGGLE_HPP
#define JIGGLE_HPP

#include "glut_app.hh"
#include "render_manager.hh"
#include "camera.hh"
#include "viewport.hh"
#include "axes.hh"
#include "physics.hh"
#include "object_controller.hh"

#include <vector>

class Object;
class Car;
class CarAI;
class Box;

class Simulator : public GlutApp{
public:
  Simulator(int & argc, char * argv[]);

  void idle();
  void display();

  void special(int key, int x, int y);
  void keyboard(unsigned char key, int x, int y);
  void keyboard_up(unsigned char key, int x, int y);

  enum Camera_dir {CAMERA_LEFT, CAMERA_RIGHT, CAMERA_FWD, CAMERA_BACK};
  void move_camera(Camera_dir dir);

  /// does extra stuff after calling the base class
  void initialise(const std::string& app_name);

private:
  /// helper for setting up
  void setup_objects_from_config();
  Car * create_car();
  Box * create_box();
  void create_wall();

  RenderManager m_render_manager;
  Camera m_camera;
  Viewport m_viewport;
  Axes m_axes;
  Physics m_physics;
  ObjectController m_object_controller;
  CarAI* m_car_ai;

  std::vector<Object*> m_objects;
  std::vector<Car*> m_cars;

  double m_frame_dt;
  double m_frame_start_time;
  int m_frame_counter;
};

#endif
