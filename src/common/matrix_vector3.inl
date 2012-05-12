/*!
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk
  
	\file matrix_vector3.inl
*/

#include "misc.hh"
#include "log_trace.hh"

#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <string.h> // for memcpy

//########################################################################
// 
//                       Matrix3
//
//########################################################################
inline Matrix3::Matrix3() {};
inline Matrix3::~Matrix3() {};

inline void Matrix3::set_to(double val)
{
  data[0] = val;
  data[1] = val;
  data[2] = val;
  data[3] = val;
  data[4] = val;
  data[5] = val;
  data[6] = val;
  data[7] = val;
  data[8] = val;
}

inline Matrix3::Matrix3(double val) {set_to(val);}

inline Matrix3::Matrix3(double v11, double v21, double v31, // first column
                        double v12, double v22, double v32, // 2nd column
                        double v13, double v23, double v33  )
{
  data[0] = v11;
  data[1] = v21;
  data[2] = v31;
  
  data[3] = v12;
  data[4] = v22;
  data[5] = v32;
  
  data[6] = v13;
  data[7] = v23;
  data[8] = v33;
}
inline Matrix3::Matrix3(const vec3d & v1, // first column
                        const vec3d & v2, 
                        const vec3d & v3)
{
  data[0] = v1[0];
  data[1] = v1[1];
  data[2] = v1[2];
  
  data[3] = v2[0];
  data[4] = v2[1];
  data[5] = v2[2];
  
  data[6] = v3[0];
  data[7] = v3[1];
  data[8] = v3[2];
}

inline void Matrix3::set_data(const double * d)
{
  memcpy(data, d, 9*sizeof(double));
}

inline vec3d Matrix3::get_col(uint i) const
{
  const uint o = i*3; 
  return vec3d(data[o], data[o+1], data[o+2]);
}

inline void Matrix3::set_col(uint i, const vec3d & col)
{
  const uint o = i*3; 
  data[o]   = col[0];
  data[o+1] = col[1];
  data[o+2] = col[2];
}

inline bool Matrix3::sensible() const
{
  for (unsigned i = 0 ; i < 9 ; ++i)
  {
		if (!is_finite(data[i]))
	    return false;
  }
  return true;
}

inline void Matrix3::show(const char * str) const
{
  uint i, j;
  //TRACE("%s Matrix3::this = 0x%x \n", str, (int) this);
  for (i = 0 ; i < 3 ; i++)
  {
    for (j = 0 ; j < 3 ; j++)
    {
      TRACE("%4f ", operator()(i, j));
    }
    TRACE("\n");
  }
}

inline Matrix3 & Matrix3::operator+=(const Matrix3 & rhs)
{
  for (uint i = 9 ; i-- != 0 ;)
    data[i] += rhs.data[i];
  return *this;
}

inline Matrix3 & Matrix3::operator-=(const Matrix3 & rhs)
{
  for (uint i = 9 ; i-- != 0 ;)
    data[i] -= rhs.data[i];
  return *this;
}

inline Matrix3 & Matrix3::operator*=(const double rhs)
{
  for (uint i = 9 ; i-- != 0 ;)
		data[i] *= rhs;
  return *this;
}

inline Matrix3 & Matrix3::operator/=(const double rhs)
{
  const double inv_rhs = 1.0f/rhs;
  for (uint i = 9 ; i-- != 0 ;)
		data[i] *= inv_rhs;
  return *this;
}

inline Matrix3 Matrix3::operator+(const Matrix3 & rhs) const
{
  return Matrix3(*this) += rhs;
}

inline Matrix3 Matrix3::operator-(const Matrix3 & rhs) const
{
  return Matrix3(*this) -= rhs;
}

// global operators

inline Matrix3 operator*(const Matrix3 & lhs, const double rhs)
{
  Matrix3 result;
  
  for (uint i = 9 ; i-- != 0 ; )
    result.data[i] = rhs * lhs.data[i];
  return result;
}

inline Matrix3 operator*(const Matrix3 & lhs, const Matrix3 & rhs)
{
  static Matrix3 out; // avoid ctor/dtor
  
  for (uint oj = 3 ; oj-- != 0 ;)
  {
    for (uint oi = 3 ; oi-- != 0 ;)
    {
      out(oi, oj) =
        lhs(oi, 0)*rhs(0, oj) +
        lhs(oi, 1)*rhs(1, oj) +
        lhs(oi, 2)*rhs(2, oj);
    }
  }
  return out;
}

//########################################################################
// 
//                       vec3d
//
//########################################################################

inline void vec3d::set_to(double val)
{
  data[0] = val;
  data[1] = val;
  data[2] = val;
}

inline vec3d::vec3d(double val) {set_to(val);}

inline vec3d & vec3d::normalise()
{
  const double m2 = mag2();
	if (m2 > 0.0f)
	{
    const double inv_mag = 1.0f/sqrt(m2);
    data[0] = data[0] * inv_mag;
    data[1] = data[1] * inv_mag;
    data[2] = data[2] * inv_mag;
    return *this;
  }
  else
  {
    TRACE("magnitude = %f in normalise()\n", sqrt(m2));
    data[0] = 1.0f; 
    data[1] = data[2] = 0.0f;
    return *this;
  }
}

inline void vec3d::set_data(const double * d)
{
  memcpy(data, d, 3*sizeof(double));  
}

inline vec3d & vec3d::operator+=(const vec3d & rhs)
{
  data[0] += rhs.data[0];
  data[1] += rhs.data[1];
  data[2] += rhs.data[2];
  return *this;
}

inline vec3d & vec3d::operator-=(const vec3d & rhs)
{
  data[0] -= rhs.data[0];
  data[1] -= rhs.data[1];
  data[2] -= rhs.data[2];
  return *this;
}

inline vec3d & vec3d::operator*=(const double rhs)
{
  data[0] *= rhs;
  data[1] *= rhs;
  data[2] *= rhs;
  return *this;
}

inline vec3d & vec3d::operator/=(const double rhs)
{
  const double inv_rhs = 1.0f/rhs;
  data[0] *= inv_rhs;
  data[1] *= inv_rhs;
  data[2] *= inv_rhs;
  return *this;
}

inline vec3d vec3d::operator+(const vec3d & rhs) const
{
  return vec3d(data[0] + rhs.data[0], 
                 data[1] + rhs.data[1], 
                 data[2] + rhs.data[2]);
}

inline vec3d vec3d::operator-(const vec3d & rhs) const
{
  return vec3d(data[0] - rhs.data[0], 
                 data[1] - rhs.data[1], 
                 data[2] - rhs.data[2]);
}

inline bool vec3d::sensible() const
{
  for (unsigned i = 3 ; i-- != 0 ;)
  {
    if (!is_finite(data[i]))
      return false;
  }
  return true;
}

inline void vec3d::show(const char * str) const
{
  uint i;
  //TRACE("%s vec3d::this = 0x%x \n", str, (int) this);
  for (i = 0 ; i < 3 ; i++)
  {
    TRACE("%4f ", data[i]);
  }
  TRACE("\n");
}

// Helper for orthonormalise - projection of v2 onto v1
static inline vec3d proj(const vec3d & v1, const vec3d & v2)
{
  return dot(v1, v2) * v1 / v1.mag2();
}

inline void Matrix3::orthonormalise()
{
  vec3d u1(operator()(0, 0), operator()(1, 0), operator()(2, 0));
  vec3d u2(operator()(0, 1), operator()(1, 1), operator()(2, 1));
  vec3d u3(operator()(0, 2), operator()(1, 2), operator()(2, 2));
  
  vec3d w1 = u1.normalise();
  
  vec3d w2 = (u2 - proj(w1, u2)).normalise();
  vec3d w3 = (u3 - proj(w1, u3) - proj(w2, u3)).normalise();
  
  operator()(0, 0) = w1[0];
  operator()(1, 0) = w1[1];
  operator()(2, 0) = w1[2];
  
  operator()(0, 1) = w2[0];
  operator()(1, 1) = w2[1];
  operator()(2, 1) = w2[2];
  
  operator()(0, 2) = w3[0];
  operator()(1, 2) = w3[1];
  operator()(2, 2) = w3[2];
	
	if (sensible() == false)
  {
    TRACE("orthonormalise() resulted in bad matrix\n");
    *this = Matrix3(vec3d(1, 0, 0), vec3d(0, 1, 0), vec3d(0, 0, 1));
  }
}


// global operators

inline vec3d operator*(const vec3d & lhs, const double rhs)
{
  return vec3d(lhs.data[0] * rhs,
                 lhs.data[1] * rhs,
                 lhs.data[2] * rhs);
}

inline vec3d operator/(const vec3d & lhs, const double rhs)
{
  const double inv_rhs = 1.0f/rhs;
  return vec3d(lhs.data[0] * inv_rhs,
                 lhs.data[1] * inv_rhs,
                 lhs.data[2] * inv_rhs);
}

inline double dot(const vec3d & lhs, const vec3d & rhs)
{
  return (lhs.data[0] * rhs.data[0] +
          lhs.data[1] * rhs.data[1] +
          lhs.data[2] * rhs.data[2]);
}

inline vec3d cross(const vec3d & lhs, const vec3d & rhs)
{
  return vec3d(lhs[1]*rhs[2] - lhs[2]*rhs[1],
                 lhs[2]*rhs[0] - lhs[0]*rhs[2],
                 lhs[0]*rhs[1] - lhs[1]*rhs[0]);
}

// matrix * vector
inline vec3d operator*(const Matrix3 & lhs, const vec3d & rhs)
{
  return vec3d(
    lhs(0,0) * rhs[0] +
    lhs(0,1) * rhs[1] +
    lhs(0,2) * rhs[2],
    lhs(1,0) * rhs[0] +
    lhs(1,1) * rhs[1] +
    lhs(1,2) * rhs[2],
    lhs(2,0) * rhs[0] +
    lhs(2,1) * rhs[1] +
    lhs(2,2) * rhs[2]);
}

inline Matrix3 transpose(const Matrix3 & rhs)
{
  return Matrix3(rhs(0, 0), rhs(0, 1), rhs(0, 2),
                 rhs(1, 0), rhs(1, 1), rhs(1, 2),
                 rhs(2, 0), rhs(2, 1), rhs(2, 2) );
}

inline double trace(const Matrix3 & rhs)
{
  return rhs(0,0) + rhs(1,1) + rhs(2,2);
}

// Some useful rotation Matrix3's
inline Matrix3 m3alpha(double alpha)
{
  Matrix3 result(0);
  double s = (double) sin_deg(alpha);
  double c = (double) cos_deg(alpha);
  
  result(0,0) = 1;
  result(1,1) = c;
  result(2,2) = c;
  result(2,1) = s;
  result(1,2) = -s;
  
  return result;
}

inline Matrix3 m3beta(double beta)
{
  Matrix3 result(0);
  double s = (double) sin_deg(beta);
  double c = (double) cos_deg(beta);
  
  result(1,1) = 1;
  result(2,2) = c;
  result(0,0) = c;
  result(0,2) = s;
  result(2,0) = -s;
  
  return result;
}

inline Matrix3 m3gamma(double gamma)
{
  Matrix3 result(0);
  double s = (double) sin_deg(gamma);
  double c = (double) cos_deg(gamma);
  
  result(2,2) = 1;
  result(0,0) = c;
  result(1,1) = c;
  result(1,0) = s;
  result(0,1) = -s;
  
  return result;
}

inline Matrix3 rotation_matrix(double ang, const vec3d & dir)
{
  // from page 32(45) of glspec.dvi
  Matrix3 uut(dir[0]*dir[0], dir[1]*dir[0], dir[2]*dir[0],
              dir[0]*dir[1], dir[1]*dir[1], dir[2]*dir[1],
              dir[0]*dir[2], dir[1]*dir[2], dir[2]*dir[2]);
  
//    uut.set(0,0, dir[0]*dir[0]);
//    uut.set(0,1, dir[0]*dir[1]);
//    uut.set(0,2, dir[0]*dir[2]);
  
//    uut.set(1,0, dir[1]*dir[0]);
//    uut.set(1,1, dir[1]*dir[1]);
//    uut.set(1,2, dir[1]*dir[2]);
  
//    uut.set(2,0, dir[2]*dir[0]);
//    uut.set(2,1, dir[2]*dir[1]);
//    uut.set(2,2, dir[2]*dir[2]);
  
  Matrix3 s(0, dir[2], -dir[1],
            -dir[2], 0, dir[0],
            dir[1], -dir[0], 0);
  
//    s.set(0,1, -dir[2]);
//    s.set(0,2,  dir[1]);
  
//    s.set(1,0,  dir[2]);
//    s.set(1,2, -dir[0]);
  
//    s.set(2,0, -dir[1]);
//    s.set(2,1,  dir[0]);
  
  return (uut + (double) cos_deg(ang) * 
          (matrix3_identity() - uut) + (double) sin_deg(ang) * s);
}

// converts a rotation matrix into a rotation of degrees about axis
void calculate_rot_from_matrix(const Matrix3 & matrix, vec3d & axis, double & degrees)
{
  double factor = (trace(matrix) - 1.0f) * 0.5f;
  if (factor > 1.0f)
    factor = 1.0f;
  else if (factor < -1.0f)
    factor = -1.0f;
  degrees = acos_deg(factor);

  if (degrees == 0.0f)
  {
    axis = vec3d(1.0f, 0.0f, 0.0f);
    return;
  }
  else if (degrees == 180.0f)
  {
    if ( (matrix(0, 0) > matrix(1, 1)) && (matrix(0, 0) > matrix(2, 2)) )
    {
      axis[0] = 0.5f * sqrt(matrix(0, 0) - matrix(1, 1) - matrix(2, 2) + 1.0f);
      axis[1] = matrix(0, 1) / (2.0f * axis[0]);
      axis[2] = matrix(0, 2) / (2.0f * axis[0]);
    }
    else if (matrix(1, 1) > matrix(2, 2))
    {
      axis[1] = 0.5f * sqrt(matrix(1, 1) - matrix(0, 0) - matrix(2, 2) + 1.0f);
      axis[0] = matrix(0, 1) / (2.0f * axis[1]);
      axis[2] = matrix(1, 2) / (2.0f * axis[1]);
    }
    else
    {
      axis[2] = 0.5f * sqrt(matrix(2, 2) - matrix(0, 0) - matrix(1, 1) + 1.0f);
      axis[0] = matrix(0, 2) / (2.0f * axis[2]);
      axis[2] = matrix(1, 2) / (2.0f * axis[2]);
    }
  }
  else
  {
    axis[0] = matrix(2, 1) - matrix(1, 2);
    axis[1] = matrix(0, 2) - matrix(2, 0);
    axis[2] = matrix(1, 0) - matrix(0, 1);
  }
  axis.normalise();
}

