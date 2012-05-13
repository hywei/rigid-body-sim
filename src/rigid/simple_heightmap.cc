#include "simple_heightmap.hh"

SimpleHeightMap::SimpleHeightMap(Array_2D<double> & heightmap) :
m_heightmap(heightmap),
m_display_list_num(0){
  TRACE_METHOD_ONLY(ONCE_1);
}

void SimpleHeightMap::draw(){
  TRACE_METHOD_ONLY(FRAME_1);

  if (m_display_list_num == 0){
    glDeleteLists(m_display_list_num, 1);
    m_display_list_num = glGenLists(1);
    glNewList(m_display_list_num, GL_COMPILE);
  
    int nx, ny;
    get_nx_ny(nx, ny);
    double x0, y0, dx, dy;
    get_x0_y0(x0, y0);
    get_dx_dy(dx, dy);
    for (int j0 = 0 ; j0 < (ny - 1) ; ++j0){
      glBegin(GL_TRIANGLE_STRIP);
      for (int i = 0 ; i < nx ; ++i){
        // work out the normals
        for (int offset = 1 ; offset >= 0 ; --offset){
          int j = j0 + offset;
          int i_p(i+1), i_m(i-1), j_p(j+1), j_m(j-1);

          if (i_p >= nx) i_p = nx-1;            
          if (i_m < 0) i_m = 0;
          if (j_p >= ny) j_p = ny-1;            
          if (j_m < 0) j_m = 0;
          vec3d normal = cross(vec3d(2 * dx, 0.0f, m_heightmap(i_p, j) - m_heightmap(i_m, j)),
                                 vec3d(0.0f, 2 * dy, m_heightmap(i, j_p) - m_heightmap(i, j_m)));
          normal.normalise();
          GLNORMAL3V(normal.get_data());
          double z = m_heightmap(i, j);
          GLVERTEX3(x0 + i * dx, y0 + j * dy, z);
        }
      }
      glEnd();
    }    
    glEndList();
  }

  glCallList(m_display_list_num);
}

// from SimHeightMap
double SimpleHeightMap::get_z(int i, int j) const{
  int nx, ny;
  get_nx_ny(nx, ny);
  if (i >= nx) i = nx-1;
  if (j >= ny) j = ny-1;
  return m_heightmap(i, j);
}

void SimpleHeightMap::get_z_and_normal(double x, double y, double & z, 
                                        vec3d& normal) const{
  int nx, ny;
  get_nx_ny(nx, ny);
  double x0, y0;
  get_x0_y0(x0, y0);
  double dx, dy;
  get_dx_dy(dx, dy);

  int i0 = (int) ((x - x0)/dx);
  int j0 = (int) ((y - y0)/dy);
  if (i0 < 0) i0 = 0;
  else if (i0 >= nx) i0 = nx-1;
  if (j0 < 0) j0 = 0;
  else if (j0 >= ny) j0 = ny -1;
  
  int i1(i0+1), j1(j0+1);

  if (i1 >= nx) i1 = nx - 1;
  if (j1 >= ny) j1 = ny - 1;

  double i_frac = (x - (i0 * dx + x0))/dx;
  double j_frac = (y - (j0 * dy + y0))/dy;

  if (i_frac < 0.0f) i_frac = 0.0f;
  else if (i_frac > 1.0f) i_frac = 1.0f;

  if (j_frac < 0.0f) j_frac = 0.0f;
  else if (j_frac > 1.0f) j_frac = 1.0f;

  double v00 = m_heightmap(i0, j0);
  double v11 = m_heightmap(i1, j1);

  // All the triangles are orientated the same way.
  // work out the normal, then z is in the plane of this normal
  if ( (i0 == i1) || (j0 == j1) ){
    // this should only be if both are equal TODO
    normal = vec3d(0.0f, 0.0f, 1.0f);
  }else if (i_frac > j_frac){
    // lower right tri
    double v10 = m_heightmap(i1, j0);
    normal = cross(coord(dx, 0.0f, v10 - v00), 
                   coord(dx, dy, v11 - v00)).normalise();
  }else{
    // upper left tri
    double v01 = m_heightmap(i0, j1);
    normal = cross(coord(dx, dy, v11 - v00), 
                   coord(0.0f, dy, v01 - v00)).normalise();
  }

  // get the plane equation
  double dist = - dot(normal, coord(x0 + i0 * dx, y0 + j0 * dy, v00)); // v00 is in all the triangles    
  z = -(dist + normal[0] * x + normal[1] * y) / normal[2];
}

// position and normal are returned if there is a collision
bool SimpleHeightMap::get_segment_collision(const Segment & segment, double & frac, coord & position, vec3d & normal) const{
  // assume that the segment doesn't pass through the terrain and out.
  if (segment.dir[2] > -0.0001)
    return false;

  vec3d normal_start;
  double z_start;
  get_z_and_normal(segment.start[0], segment.start[1], z_start, normal_start);

  if (segment.start[2] < z_start)
    return false;

  vec3d normal_end;
  double z_end;
  coord end = segment.start + segment.dir;
  get_z_and_normal(end[0], end[1], z_end, normal_end);

  if (end[2] > z_end)
    return false;

  // start is above, end is below...
  double height_start = segment.start[2] - z_start;
  double depth_end = z_end - end[2];

  // normal is the weighted mean of these...
  double weight_start = 1.0 / (0.001 + height_start);
  double weight_end = 1.0 / (0.001 + depth_end);

  normal = (normal_start * weight_start + normal_end * weight_end) / (weight_start + weight_end);

  frac = height_start / (height_start + depth_end + 0.00001);

  position = segment.start + frac * segment.dir;

  return true;
}

