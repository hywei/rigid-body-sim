
#include "viewport.hh"

#include "camera.hh"
#include "render_manager.hh"

#include "../render/glut_utils.hh"

using namespace std;

Viewport::Viewport(const char * name,
                   RenderManager * render_manager,
                   Camera * camera,
                   double offset_x, double offset_y,
                   double width, double height) :
    m_name(name),
    m_render_manager(render_manager),
    m_camera(camera),
    m_offset_x(offset_x), m_offset_y(offset_y),
    m_width(width), m_height(height){
  m_clip_near = 0.1f;
  m_clip_far = 1000.0f;
  m_fov = 60.0f;
}

void Viewport::display(){
  TRACE_METHOD_ONLY(FRAME_1);
  double window_w = glutGet(GLUT_WINDOW_WIDTH);
  double window_h = glutGet(GLUT_WINDOW_HEIGHT);

  // setup the projection for this viewport
  glViewport((GLsizei) (m_offset_x * window_w), (GLsizei) (m_offset_y * window_h), 
             (GLsizei) (m_width * window_w), (GLsizei) (m_height * window_h));
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective( 
      m_fov , /* field of view in degree */
      (window_w * m_width)/(window_h * m_height), /* aspect ratio */ 
      m_clip_near, m_clip_far);

  // get the camera to look in the right direction etc
  m_camera->apply();

  glMatrixMode(GL_MODELVIEW);


  // glEnable(GL_LINE_SMOOTH);
  // glEnable(GL_BLEND);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
  // // Lighting - TODO move this
  // // static GLfloat white_light[]=  {1.0f,1.0f,1.0f,1.0f};
  // // static GLfloat ambient_light[]={0.1f,0.1f,0.1f,1.0f};
  // // static GLfloat light_pos[] = {1000.0f, 2000.0f, 1000.0f, 0.0f};
  // // glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  // // glLightfv(GL_LIGHT0,GL_DIFFUSE,white_light);
  // // glLightfv(GL_LIGHT0,GL_AMBIENT,ambient_light);
  // // glEnable(GL_LIGHT0);
  // // glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  // // glEnable(GL_LIGHTING);

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
    
  // // move this too
  // static GLfloat mat_diffuse[]=  {0.4,0.4,0.4,1.0};
  // glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  // glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  // glEnable(GL_COLOR_MATERIAL);

  // glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  // Clear
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // now get things drawn
  m_render_manager->display_all_objects();
}

