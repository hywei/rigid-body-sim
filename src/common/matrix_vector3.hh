/*
  Sss - a slope soaring simulater.
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk
*/
#ifndef MATRIX_VECTOR3
#define MATRIX_VECTOR3

#include <math.h>
#include "types.hh"
typedef unsigned int uint;

class vec3d;

//! A 3x3 matrix
class Matrix3
{
public:
  inline Matrix3(double v11, double v21, double v31, // first column
                 double v12, double v22, double v32, // 2nd column
                 double v13, double v23, double v33  );
  inline Matrix3(const vec3d & v1, // first column
                 const vec3d & v2, 
                 const vec3d & v3);
  inline Matrix3();
  explicit inline Matrix3(double val);
  inline ~Matrix3();

  inline void set_to(double val);
  inline void orthonormalise();
  
  inline bool sensible() const; // indicates if all is OK

  double & operator()(const uint i, const uint j) {return data[i + 3*j];}
  const double & operator()(const uint i, const uint j) const {return data[i + 3*j];}
  
  //! returns pointer to the first element
  inline const double * get_data() {return data;} 
  inline const double * get_data() const {return data;} 
  //! pointer to value returned from get_data
  inline void set_data(const double * d); 
  
  /// Returns a column - no range checking!
  inline vec3d get_col(uint i) const;

  // sets a column
  inline void set_col(uint i, const vec3d & col);

  // operators
  inline Matrix3 & operator+=(const Matrix3 & rhs);
  inline Matrix3 & operator-=(const Matrix3 & rhs);

  inline Matrix3 & operator*=(const double rhs);
  inline Matrix3 & operator/=(const double rhs);

  inline Matrix3 operator+(const Matrix3 & rhs) const;
  inline Matrix3 operator-(const Matrix3 & rhs) const;
  friend Matrix3 operator*(const Matrix3 & lhs, const double rhs);
  friend Matrix3 operator*(const double lhs, const Matrix3 & rhs);
  friend Matrix3 operator*(const Matrix3 & lhs, const Matrix3 & rhs);
  friend Matrix3 transpose(const Matrix3 & rhs);
  friend double trace(const Matrix3 & rhs);  
  friend vec3d operator*(const Matrix3 & lhs, const vec3d & rhs);

  inline void show(const char * str = "") const;
  
private:
  double data[9];
};

//############## vec3d ################
//! A 3x1 matrix (i.e. a vector)
class vec3d
{
public:
  inline vec3d() {};
  explicit inline vec3d(double val);
  inline vec3d(double x, double y, double z) 
    {data[0] = x; data[1] = y; data[2] = z;}
  inline ~vec3d() {};
  
  inline void set_to(double val); //!< Set all values to val
  
  inline bool sensible() const; // indicates if all is OK

  double & operator[](uint i) {return data[i];} //!< unchecked access
  const double & operator[](uint i) const {return data[i];}
  double & operator()(uint i) {return data[i];}
  const double & operator()(uint i) const {return data[i];}
  
  //! returns pointer to the first element
  inline const double * get_data() {return data;} 
  inline const double * get_data() const {return data;} 
  //! pointer to value returned from get_data
  inline void set_data(const double * d);
  
  //! calculate the square of the magnitude
  inline double mag2() const {
    return (double) (data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);}
  //! calculate the magnitude
  inline double mag() const {return (double) sqrt(mag2());}
  //! Normalise, and return the result
  inline vec3d & normalise();
  
  // operators
  inline vec3d & operator+=(const vec3d & rhs);
  inline vec3d & operator-=(const vec3d & rhs);

  inline vec3d & operator*=(const double rhs);
  inline vec3d & operator/=(const double rhs);

  inline vec3d operator-() const {return vec3d(-data[0], -data[1], -data[2]);}
  
  inline vec3d operator+(const vec3d & rhs) const;
  inline vec3d operator-(const vec3d & rhs) const;

  friend vec3d operator*(const vec3d & lhs, const double rhs);
  friend vec3d operator*(const double lhs, const vec3d & rhs);
  friend vec3d operator/(const vec3d & lhs, const double rhs);
  friend double dot(const vec3d & lhs, const vec3d & rhs);
  friend vec3d cross(const vec3d & lhs, const vec3d & rhs);
  
  friend vec3d operator*(const Matrix3 & lhs, const vec3d & rhs);
  
  friend Matrix3 rotation_matrix(double ang, const vec3d & dir);
  
  inline void show(const char * str = "") const;
  
private:
  double data[3];
};

// global operators

inline Matrix3 operator*(const Matrix3 & lhs, const double rhs);
inline Matrix3 operator*(const double lhs, const Matrix3 & rhs) {return rhs * lhs;}
inline Matrix3 operator*(const Matrix3 & lhs, const Matrix3 & rhs);

inline vec3d operator*(const vec3d & lhs, const double rhs);
inline vec3d operator/(const vec3d & lhs, const double rhs);
inline double dot(const vec3d & lhs, const vec3d & rhs);
inline vec3d cross(const vec3d & lhs, const vec3d & rhs);
inline vec3d operator*(const double lhs, const vec3d & rhs) {return rhs * lhs;}
inline Matrix3 transpose(const Matrix3 & rhs);

inline double trace(const Matrix3 & rhs);

// matrix * vector
inline vec3d operator*(const Matrix3 & lhs, const vec3d & rhs);

// Some useful rotation Matrix3's
// alpha returns a matrix that wil rotate alpha around the x axis (etc)
inline Matrix3 m3alpha(double alpha);
inline Matrix3 m3beta(double beta);
inline Matrix3 m3gamma(double gamma);

inline const Matrix3 & matrix3_identity()
{
  static const Matrix3 result(1, 0, 0,
                              0, 1, 0,
                              0, 0, 1);
  return result;
}

inline Matrix3 rotation_matrix(double ang, const vec3d & dir);

// converts a rotation matrix into a rotation of degrees about axis
inline void calculate_rot_from_matrix(const Matrix3 & matrix, vec3d & axis, double & degrees);


#include "matrix_vector3.inl"

#endif // include file



