#include "camera.hh"
#include "render_object.hh"
#include "glut_utils.hh"

Camera::Camera() :
m_offset(0),
m_offset_absolute(false),
m_track_object(0),
m_position_object(0),
m_target_object(0),
m_up_object(0),
m_orientation_object(0){
  TRACE_METHOD_ONLY(ONCE_1);
}

void Camera::set_track_target(RenderObject * track_object, double dist, double delta_z, double speed){
  m_track_object = track_object;
  m_track_dist = dist;
  m_track_delta_z = delta_z;
  m_track_speed = speed;
}

void Camera::apply(){
  TRACE_METHOD_ONLY(FRAME_1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(m_position[0], m_position[1], m_position[2],
            m_target[0], m_target[1], m_target[2],
            m_up[0], m_up[1], m_up[2]);
}

void Camera::update(double dt){
  TRACE_METHOD_ONLY(FRAME_1);
  // update the camera properties
  if (m_track_object){
    coord ideal_pos = m_track_object->get_render_position() + 
      coord(0.0, 0.0, m_track_delta_z);
    vec3d delta = ideal_pos - m_position;

    // limit delta
    if (delta.mag() > m_track_speed * dt){
      delta = (m_track_speed * dt / delta.mag()) * delta;
    }

    m_position += delta;

    // make sure we're away from the object
    
    delta = m_position - (m_track_object->get_render_position() + coord(0.0, 0.0, m_track_delta_z));
    if (delta.mag() < m_track_dist){
      delta = (m_track_dist / delta.mag()) * delta;
      m_position = (m_track_object->get_render_position() + coord(0.0, 0.0, m_track_delta_z)) + delta;
    }

    m_up = vec3d(0.0, 0.0, 1.0);

    if (m_target_object)
      m_target = m_target_object->get_render_position();
  }else {
    if (m_position_object){
      if (m_offset_absolute)
        m_position = m_position_object->get_render_position() + m_offset;
      else
        m_position = m_position_object->get_render_position() + m_position_object->get_render_orientation() * m_offset;
    }
    if (m_orientation_object){
      m_target = m_position + m_orientation_object->get_render_orientation().get_col(0);
      m_up = m_orientation_object->get_render_orientation().get_col(2);
    }else{
      if (m_target_object)
        m_target = m_target_object->get_render_position();
      if (m_up_object)
        m_up = m_up_object->get_render_orientation().get_col(2);
    }
  }
}

void Camera::set_position(const coord & position){
  TRACE_METHOD_ONLY(FRAME_2);
  m_position = position;
}

void Camera::set_target(const coord & target){
  TRACE_METHOD_ONLY(FRAME_2);
  m_target = target;
}

void Camera::set_up(const vec3d & up){
  TRACE_METHOD_ONLY(FRAME_2);
  m_up = up;
}

void Camera::set_position_object(RenderObject * position_object){
  TRACE_METHOD_ONLY(ONCE_1);
  m_position_object = position_object;
}

void Camera::set_target_object(RenderObject * target_object){
  TRACE_METHOD_ONLY(ONCE_1);
  m_target_object = target_object;
}

void Camera::set_up_object(RenderObject * up_object){
  TRACE_METHOD_ONLY(ONCE_1);
  m_up_object = up_object;
}

void Camera::set_orientation_object(RenderObject * orientation_object){
  TRACE_METHOD_ONLY(ONCE_1);
  m_orientation_object = orientation_object;
}
