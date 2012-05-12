/*!
  Copyright (C) 2002 Danny Chapman - danny@rowlhouse.freeserve.co.uk

  \file camera.hh
*/

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "types.hh"

class RenderObject;

class Camera
{
public:
  Camera();

  /// Update the camera position etc (e.g. follow the target)
  void update(double dt);

  /// set up the perspective view for the current parameters
  void apply();

  void set_position(const coord & position);
  /// absolute if the offset is in world space (i.e. an absolute offset)
  void set_position_offset(const coord & offset, bool absolute = false) 
    {m_offset = offset; m_offset_absolute = absolute;} // when using target object
  void set_target(const coord & target);
  void set_up(const vec3d & up);

  void set_track_target(RenderObject * target_object, double dist, double delta_z, double speed);

  const coord & get_position() const {return m_position;}
  const coord & get_target() const {return m_target;}
  const vec3d & get_up() const {return m_up;}

  void set_position_object(RenderObject * position_object);
  // if orientation object is set this over-rides target/up objects
  void set_orientation_object(RenderObject * orientation_object);
  void set_target_object(RenderObject * target_object);
  void set_up_object(RenderObject * up_object);

private:
  coord m_position;
  vec3d  m_offset;
  bool m_offset_absolute;
  coord m_target;
  vec3d  m_up;

  RenderObject * m_track_object;
  double m_track_dist, m_track_delta_z, m_track_speed;
  coord m_track_position;

  // if any of these objects are zero, then the property stays fixed
  RenderObject * m_position_object;
  RenderObject * m_target_object;
  RenderObject * m_up_object;
  RenderObject * m_orientation_object;
};

#endif
