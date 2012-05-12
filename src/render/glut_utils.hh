// Work around the way that Mac OSX puts GLUT in GLUT/glut.h, instead
// of GL/glut.h (which is where it should be!).

#ifndef SSS_GLUT_H
#define SSS_GLUT_H

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif  // __APPLE__

// my stuff
#include "log_trace.hh"
#include "types.hh"

// warning - expensive!
struct Save_GL_state
{
  Save_GL_state() {glPushMatrix(); glPushAttrib(GL_ALL_ATTRIB_BITS);}
  ~Save_GL_state() {glPopMatrix(); glPopAttrib();}
};

// cheap!
struct Save_GL_matrix_state
{
  Save_GL_matrix_state() {glPushMatrix();}
  ~Save_GL_matrix_state() {glPopMatrix();}
};

inline void check_errors(char * str="")
{
  GLenum err_code = glGetError();
  if (err_code != GL_NO_ERROR)
  {
    TRACE("OpenGL Error: %s [%s]\n", gluErrorString(err_code), str);
  }
}

#define GLNORMAL2 glNormal2d
#define GLNORMAL3 glNormal3d
#define GLNORMAL4 glNormal4d

#define GLNORMAL2V glNormal2dv
#define GLNORMAL3V glNormal3dv
#define GLNORMAL4V glNormal4dv

#define GLVERTEX2 glVertex2d
#define GLVERTEX3 glVertex3d
#define GLVERTEX4 glVertex4d

#define GLVERTEX2V glVertex2dv
#define GLVERTEX3V glVertex3dv
#define GLVERTEX4V glVertex4dv

#define GLCOLOR2 glColor2d
#define GLCOLOR3 glColor3d
#define GLCOLOR4 glColor4d

#define GLTRANSLATE glTranslated
#define GLROTATE glRotated

#define GLMULTMATRIX glMultMatrixd


inline void apply_transformation(const vec3d & translation, const Matrix3 & rotation){
  // translation's easy...
  GLTRANSLATE(translation[0], translation[1], translation[2]);

  double matrix[] ={
    rotation(0, 0), // 1st column
    rotation(1, 0),
    rotation(2, 0),
    0,
    rotation(0, 1), // 2nd column
    rotation(1, 1),
    rotation(2, 1),
    0,
    rotation(0, 2), // 3rd column
    rotation(1, 2),
    rotation(2, 2),
    0,
    0, 0, 0, 1        // 4th column
  };
  GLMULTMATRIX(&matrix[0]);
}

#endif
