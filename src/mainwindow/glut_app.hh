#ifndef APPLICATION_HPP
#define APPLICATION_HPP

// may make use of the freeglut extensions.
#include "../render/glut_utils.hh"
#include "config_file.hh"

#include <string>

/// This is a base class for a GLUT-based application. The application just implements the 
/// relevant virtual functions. It should only be created once.
/// Since freeglut will/may be used, it will allow for deletion.
class GlutApp{
public:
  // constructor generates a config file (which can be subsequently used by the application)
  // and sorts out tracing
  GlutApp(int& argc, char * argv[], std::string config_file_name);
  virtual ~GlutApp();

  /// creates a double buffered window according to the parameters in 
  /// the config file, except the name is passed in (so it can be hard-coded).
  virtual void initialise(const std::string app_name);

  /// start the main loop - if using freeglut this may return. Should have already called
  /// initialise
  virtual void start_main_loop();

  static Config_file * get_config_file() {return m_instance->m_config_file;}

  void go_fullscreen() {glutFullScreen();}

  // application can over-ride these. Default implementations do nothing.
  virtual void idle();
  virtual void display();
  virtual void mouse(int button, int state, int x, int y);
  virtual void motion(int x, int y);
  virtual void keyboard(unsigned char key, int x, int y);
  virtual void keyboard_up(unsigned char key, int x, int y);
  virtual void special(int key, int x, int y);
  virtual void special_up(int key, int x, int y);

  /// default implementations do something sensible - tho you might want to over-ride
  virtual void reshape(int w, int h);

private:
  // fns to be called by GLUT
  static void _idle();
  static void _display();
  static void _mouse(int button, int state, int x, int y);
  static void _motion(int x, int y);
  static void _reshape(int w, int h);
  static void _keyboard(unsigned char key, int x, int y);
  static void _keyboard_up(unsigned char key, int x, int y);
  static void _special(int key, int x, int y);
  static void _special_up(int key, int x, int y);

  static GlutApp * m_instance;

  int m_window_w, m_window_h;

  Config_file * m_config_file;
};

#endif
