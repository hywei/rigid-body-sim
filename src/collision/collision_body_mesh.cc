#include "collision_body_mesh.hh"
#include "collision_body.hh"
#include "distance.hh"
#include "intersection.hh"

inline double min4(double a, double b,double c, double d){
  return std::min(a,std::min(b, std::min(c, d)));
}
inline double max4(double a, double b, double c, double d) {
  return std::max(a, std::max(b, std::max(c, d)));
}

CollBodyMesh::CollBodyMesh(){}

void CollBodyMesh::constructMesh(const CollBodyMesh& mesh){
  *this = mesh;
}

void CollBodyMesh::constructMesh(int nx, int ny, int nz, 
                                      double extra_frac,
                                      const std::vector<CollTriangle>& triangles,
                                      const CollisionMeshObject* body){
  // first set up the mesh bounds
  nx_ = nx; ny_ = ny; nz_ = nz;
  
  const double big_val = 99999999.0f;
  min_x_ = min_y_ = min_z_ = big_val;
  max_x_ = max_y_ = max_z_ = -big_val;

  for (size_t i = 0 ; i < triangles.size() ; ++i){
    min_x_ = min4(min_x_, triangles[i].v0[0], triangles[i].v1[0], triangles[i].v2[0]);
    min_y_ = min4(min_y_, triangles[i].v0[1], triangles[i].v1[1], triangles[i].v2[1]);
    min_z_ = min4(min_z_, triangles[i].v0[2], triangles[i].v1[2], triangles[i].v2[2]);
    max_x_ = max4(max_x_, triangles[i].v0[0], triangles[i].v1[0], triangles[i].v2[0]);
    max_y_ = max4(max_y_, triangles[i].v0[1], triangles[i].v1[1], triangles[i].v2[1]);
    max_z_ = max4(max_z_, triangles[i].v0[2], triangles[i].v1[2], triangles[i].v2[2]);
  }
  
  double mid_x = 0.5f * (min_x_ + max_x_);
  double mid_y = 0.5f * (min_y_ + max_y_);
  double mid_z = 0.5f * (min_z_ + max_z_);

  min_x_ = mid_x + extra_frac * (min_x_ - mid_x);
  min_y_ = mid_y + extra_frac * (min_y_ - mid_y);
  min_z_ = mid_z + extra_frac * (min_z_ - mid_z);
  max_x_ = mid_x + extra_frac * (max_x_ - mid_x);
  max_y_ = mid_y + extra_frac * (max_y_ - mid_y);
  max_z_ = mid_z + extra_frac * (max_z_ - mid_z);

  dx_ = (max_x_ - min_x_) / (nx_ - 1);
  dy_ = (max_y_ - min_y_) / (ny_ - 1);
  dz_ = (max_z_ - min_z_) / (nz_ - 1);

  coord corners[8];
  corners[0] = coord(max_x_, max_y_, max_z_);
  corners[1] = coord(max_x_, max_y_, min_z_);
  corners[2] = coord(max_x_, min_y_, max_z_);
  corners[3] = coord(max_x_, min_y_, min_z_);
  corners[4] = coord(min_x_, max_y_, max_z_);
  corners[5] = coord(min_x_, max_y_, min_z_);
  corners[6] = coord(min_x_, min_y_, max_z_);
  corners[7] = coord(min_x_, min_y_, min_z_);
  bounding_radius_ = corners[0].mag();
  for (int i = 1 ; i < 8 ; ++i){
    if (corners[i].mag() > bounding_radius_)
      bounding_radius_ = corners[i].mag();
  }

  if(body) populateMesh(*body);    
  else populateMesh(triangles);    
}

int countTriangleHits(const Segment& seg, const std::vector<CollTriangle>& triangles){
  int hits = 0;
  coord pos;
  double s;
  bool seg_in_dir;
  for (size_t i = 0 ; i < triangles.size(); ++i){
    Triangle tri(triangles[i].v0,
                 triangles[i].v1 - triangles[i].v0,
                 triangles[i].v2 - triangles[i].v0);
    if(intersect_segment_triangle(seg, tri, pos, s, seg_in_dir)){
      ++hits;
    }
  }
  return hits;
}

// find_closest_triangle
void findClosestTriangle(const coord& pos,
                           const std::vector<CollTriangle>& triangles, 
                           int& triangle_index, 
                           coord& point_on_tri){
  coord this_point;
  double closest_sqr = 1.0E10;
  bool got_one = false;
  bool point_on_triangle_edge;

  for (size_t i = 0 ; i < triangles.size(); ++i){
    double dist_sqr = distance_sqr_point_triangle(pos, 
                                                  triangles[i].v0, triangles[i].v1, triangles[i].v2, 
                                                  this_point,
                                                  point_on_triangle_edge);
    if (dist_sqr < closest_sqr){
      got_one = true;
      triangle_index = i;
      point_on_tri = this_point;
      closest_sqr = dist_sqr;
    }
  }
}

void CollBodyMesh::populateMesh(const CollisionMeshObject& body){
  data_.resize(nx_, ny_, nz_);
  for (int i = 0 ; i < nx_ ; ++i){
    for (int j = 0 ; j < ny_ ; ++j){
      for (int k = 0 ; k < nz_ ; ++k){
        coord pos(min_x_ + i * dx_, min_y_ + j * dy_, min_z_ + k * dz_);
        bool inside = false;
        vec3d vector_to_surface;
        if (body.get_mesh_info(pos, inside, vector_to_surface)){
          Datum& datum = data_(i, j, k);
          double dist = vector_to_surface.mag();
          if (dist > 0.00001f){
            if (inside){
              datum.dir = vector_to_surface / dist;
              datum.dist = dist;
            }else{
              datum.dir = -vector_to_surface / dist;
              datum.dist = -dist;
            }
          }else{
            datum.dir = pos;
            datum.dir.normalise();
            datum.dist = 0.00001f;
          }
        }
      }
    }
  }
}

void CollBodyMesh::populateMesh(const std::vector<CollTriangle> & triangles){
  if (triangles.size() == 0) return;
  data_.resize(nx_, ny_, nz_);

  vec3d basic_dir(max_x_ - min_x_, max_y_ - min_y_, max_z_ - min_z_);
  basic_dir *= 5.0f;

  static const int num_outside_dirs = 10;
  vec3d outside_dirs[num_outside_dirs];

  for (int i = 0 ; i < num_outside_dirs ; ++i){
    outside_dirs[i] = vec3d(ranged_random(-1.0f, 1.10f), ranged_random(-1.0f, 1.10f), ranged_random(-1.0f, 1.10f));
    outside_dirs[i].normalise();
    outside_dirs[i] *= 2.0f * getBoundingRadius();
  }

  for (int i = 0 ; i < nx_ ; ++i){
    for (int j = 0 ; j < ny_ ; ++j){
      for (int k = 0 ; k < nz_ ; ++k){
        Datum & datum = data_(i, j, k);
        coord pos(min_x_ + i * dx_, min_y_ + j * dy_, min_z_ + k * dz_);
        coord point_on_triangle;
        int triangle_index;
        findClosestTriangle(pos, triangles, triangle_index, point_on_triangle);
        // set up so correct for point inside
        datum.dir = point_on_triangle - pos;
        datum.dist = datum.dir.mag();
        if (datum.dist < 0.0001f){
          // indicate we've inside...
          datum.dist = 0.0001f;
          // and take the normal from the closest face
          datum.dir = cross(triangles[triangle_index].v1 - triangles[triangle_index].v0,
                            triangles[triangle_index].v2 - triangles[triangle_index].v0);
          datum.dir.normalise();
        }else{
          datum.dir /= datum.dist;
          // tweak depending on if we're inside or outside.
          int num_in = 0;
          int num_out = 0;
          for (int l = 0 ; l < num_outside_dirs ; ++l){
            Segment seg(pos, outside_dirs[l]);
            int hits = countTriangleHits(seg, triangles);
            if (hits % 2) ++num_in;
            else ++num_out;
          }
          if (num_out > num_in){
            // outside
            datum.dist = -datum.dist;
            datum.dir = -datum.dir;
          }
          if ( (num_in > 0) && (num_out > 0) && (fabs((double) num_out - num_in) / num_outside_dirs < 0.3f) )
            TRACE_FILE_IF(ONCE_3)
              TRACE("eeeeek: (%d, %d, %d) in = %d, out = %d\n", i, j, k, num_in, num_out);
        }
      }
    }
  }
}

// returns the value interpolated from the four corner points
// x,y,z must be 0.0-1.0
template<class T>
T interp3d(const T & val000,
           const T & val001,
           const T & val010,
           const T & val011,
           const T & val100,
           const T & val101,
           const T & val110,
           const T & val111,
           double x, double y, double z){
  T vxy0 = (1.0f - y) * (val000 * (1.0f - x) + val100 * x) + 
                    y * (val010 * (1.0f - x) + val110 * x);
  T vxy1 = (1.0f - y) * (val001 * (1.0f - x) + val101 * x) + 
                    y * (val011 * (1.0f - x) + val111 * x);
  return (1.0f - z) * vxy0 + z * vxy1;
}


bool CollBodyMesh::getPointInfo(coord pos, vec3d& dir, double& dist) const{
  // In the interests of speed over accuracy, just return the data at
  // the nearest point. This avoids having to average and normalise.
  if ( (pos[0] < min_x_) || (pos[0] > max_x_) ||
       (pos[1] < min_y_) || (pos[1] > max_y_) ||
       (pos[2] < min_z_) || (pos[2] > max_z_) ){
    return false;    
  }

  // the floating-point indices of the point.
  double fi = (pos[0] - min_x_) / dx_;
  double fj = (pos[1] - min_y_) / dy_;
  double fk = (pos[2] - min_z_) / dz_;

  int ii = (int) floor(fi + 0.5f);
  int ij = (int) floor(fj + 0.5f);
  int ik = (int) floor(fk + 0.5f);

  if (ii < 0) ii = 0; else if (ii >= nx_) ii = nx_ - 1;
  if (ij < 0) ij = 0; else if (ij >= ny_) ij = ny_ - 1;
  if (ik < 0) ik = 0; else if (ik >= nz_) ik = nz_ - 1;

  const Datum & datum = data_(ii, ij, ik);
  dir = datum.dir;
  dist = datum.dist;
  // need to account for the difference between the actual point and
  // the datum point.
  double extra_dist = dot(
      dir,
      pos - coord(min_x_ + ii * dx_, 
                  min_y_ + ij * dy_,
                  min_z_ + ik * dz_) );
  dist -= extra_dist;
  
  if (dist > 0) return true;
  else{
    dist = -dist;
    return false;
  }
}

// display_object
//void CollBodyMesh::display_object(){
  // if (m_display_list_num == 0){
  //   m_display_list_num = glGenLists(1);
  //   glNewList(m_display_list_num, GL_COMPILE);

  //   // assume that all translations have been done.
  //   double blob_r = dx_ * 0.06f;
  //   vec3d zero(0.0f, 0.0f, 0.0f);

  //   int i, j, k;
  //   for (i = 0 ; i < nx_ ; ++i){
  //     for (j = 0 ; j < ny_ ; ++j){
  //       for (k = 0 ; k < nz_ ; ++k){
  //         glPushMatrix();

  //         if (data_(i, j, k).dist > 0.0f)
  //           GLCOLOR3(0.0f, 0.0f, 1.0f);
  //         else
  //           GLCOLOR3(1.0f, 0.0f, 0.0f);
          
  //         coord pos(min_x_ + i * dx_, min_y_ + j * dy_, min_z_ + k * dz_);
  //         GLTRANSLATE(pos[0], pos[1], pos[2]);
  //         glutSolidSphere(blob_r, 4, 2);

  //         vec3d dir(data_(i, j, k).dir);
  //         dir.normalise();
  //         dir *= data_(i, j, k).dist;
  //         glBegin(GL_LINES);
  //         GLVERTEX3V(zero.get_data());
  //         GLVERTEX3V(dir.get_data());
  //         glEnd();          
  //         glPopMatrix();
  //       }
  //     }
  //   }
  //   glEndList();
  // }

  // glCallList(m_display_list_num);
//}
