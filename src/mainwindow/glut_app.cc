#include "glut_app.hh"
#include "log_trace.hh"

GlutApp * GlutApp::m_instance = 0;

GlutApp::GlutApp(int & argc, char * argv[], 
                         std::string config_file_name)
:m_window_w(800),m_window_h(600), m_config_file(0){
  // let GLUT modify the arg list
  glutInit(&argc, argv);

  if (m_instance){
    TRACE("Error - GlutApp being created more than once\n");
  }
  m_instance = this;

  // setup config/tracing
  bool config_file_ok;
  if (argc > 1)
    config_file_name = string(argv[1]);
  m_config_file = new Config_file(config_file_name, config_file_ok);

  if (!config_file_ok){
    TRACE("Warning: Unable to open main config file: %s\n", config_file_name.c_str());
  }

  // set up tracing properly
  bool trace_enabled = true;
  int trace_level = 10;
  bool trace_all_strings = true;
  vector<string> trace_strings;

  m_config_file->get_value("trace_enabled", trace_enabled);
  m_config_file->get_value("trace_level", trace_level);
  m_config_file->get_value("trace_all_strings", trace_all_strings);
  m_config_file->get_values("trace_strings", trace_strings);

  m_config_file->get_value("window_width", m_window_w);
  m_config_file->get_value("window_height", m_window_h);

  enable_trace(trace_enabled);
  set_trace_level(trace_level);
  enable_trace_all_strings(trace_all_strings);
  add_trace_strings(trace_strings);

  TRACE_FILE_IF(ONCE_1)
    TRACE("Logging set up\n");
}

GlutApp::~GlutApp(){
  delete m_config_file;
  m_instance = 0;
}

void GlutApp::initialise(const std::string app_name){
  m_config_file->get_value("window_w", m_window_w);
  m_config_file->get_value("window_h", m_window_h);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
  glutInitWindowSize(m_window_w, m_window_h);
  glutCreateWindow(app_name.c_str());

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glShadeModel(GL_SMOOTH);  

  static GLfloat white_light[]=  {1.0f,1.0f,1.0f,1.0f};
  static GLfloat ambient_light[]={0.1f,0.1f,0.1f,1.0f};
  static GLfloat light_pos[] = {1000.0f, 2000.0f, 1000.0f, 0.0f};
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  glLightfv(GL_LIGHT0,GL_DIFFUSE,white_light);
  glLightfv(GL_LIGHT0,GL_AMBIENT,ambient_light);
  glEnable(GL_LIGHT0);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_LIGHTING);

  // GLfloat pos1[] = { 1.0f,  1.0f, -0.2f, 0.0f};
  // GLfloat pos2[] = {-1.0f,  1.0f, -0.2f, 0.0f};
  // GLfloat pos3[] = { 0.0f,  0.0f,  1.0f,  0.0f};


  // GLfloat col1[] = { 0.7f,  0.7f,  0.8f,  1.0f};
  // GLfloat col2[] = { 0.8f,  0.7f,  0.7f,  1.0f};
  // GLfloat col3[] = { 1.0f,  1.0f,  1.0f,  1.0f};

  // glEnable(GL_LIGHT0);    
  // glLightfv(GL_LIGHT0,GL_POSITION, pos1);
  // glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
  // glLightfv(GL_LIGHT0,GL_SPECULAR, col1);

  // glEnable(GL_LIGHT1);  
  // glLightfv(GL_LIGHT1,GL_POSITION, pos2);
  // glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
  // glLightfv(GL_LIGHT1,GL_SPECULAR, col2);

  // glEnable(GL_LIGHT2);  
  // glLightfv(GL_LIGHT2,GL_POSITION, pos3);
  // glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
  // glLightfv(GL_LIGHT2,GL_SPECULAR, col3);

  // glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  // glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  // glEnable(GL_LIGHTING);

  glEnable(GL_MULTISAMPLE);
  // move this too
  static GLfloat mat_diffuse[]=  {0.4,0.4,0.4,1.0};
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glEnable(GL_COLOR_MATERIAL);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  
  reshape(m_window_w, m_window_h);

  // register the funky functions
  glutIdleFunc(_idle);
  glutDisplayFunc(_display);
  glutMouseFunc(_mouse);
  glutMotionFunc(_motion);
  glutPassiveMotionFunc(_motion);
  glutReshapeFunc(_reshape);
  glutKeyboardFunc(_keyboard);
  glutKeyboardUpFunc(_keyboard_up);
  glutSpecialFunc(_special);
  glutSpecialUpFunc(_special_up);
}

void GlutApp::start_main_loop(){
  TRACE_FILE_IF(ONCE_1)
    TRACE("Entering main loop\n");
  glutMainLoop();
}

void GlutApp::_idle() { if (m_instance) m_instance->idle(); }
void GlutApp::_display() { if (m_instance) m_instance->display(); }
void GlutApp::_mouse(int button, int state, int x, int y) { if (m_instance) m_instance->mouse(button, state, x, y); }
void GlutApp::_motion(int x, int y) { if (m_instance) m_instance->motion(x, y); }
void GlutApp::_reshape(int w, int h) { if (m_instance) m_instance->reshape(w, h); }
void GlutApp::_keyboard(unsigned char key, int x, int y) { if (m_instance) m_instance->keyboard(key, x, y); }
void GlutApp::_keyboard_up(unsigned char key, int x, int y) { if (m_instance) m_instance->keyboard_up(key, x, y); }
void GlutApp::_special(int key, int x, int y) { if (m_instance) m_instance->special(key, x, y); }
void GlutApp::_special_up(int key, int x, int y) { if (m_instance) m_instance->special_up(key, x, y); }

void GlutApp::idle() {}
void GlutApp::display() {}
void GlutApp::mouse(int button, int state, int x, int y) {}
void GlutApp::motion(int x, int y) {}
void GlutApp::keyboard(unsigned char key, int x, int y) {}
void GlutApp::keyboard_up(unsigned char key, int x, int y) {}
void GlutApp::special(int key, int x, int y) {}
void GlutApp::special_up(int key, int x, int y) {}

void GlutApp::reshape(int w, int h){
  m_window_w = w;
  m_window_h = h;
  glutPostRedisplay();
}

