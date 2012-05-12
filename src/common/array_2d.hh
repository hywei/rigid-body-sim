/*
  Sss - a slope soaring simulater.
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk

  \file array_2d.h
*/
#ifndef ARRAY_2D_H
#define ARRAY_2D_H

#include "log_trace.hh"

#include <string.h>
#include <stdlib.h>

#include <stdexcept>
using namespace std;

/*!
  Simple class describing a 2D array
*/
template<class T>
class Array_2D{
public:

  Array_2D(int nx, int ny) : nx(nx), ny(ny), data(new T[nx*ny]), m_wrap(false)
    {};
  Array_2D(int nx, int ny, const T & val);
  Array_2D(const Array_2D & orig);

  ~Array_2D() {delete [] data; data = 0;}
  
  Array_2D<T> & operator=(const Array_2D & rhs);

  /// enables/disables wrapping
  void set_wrap(bool wrap) {m_wrap = wrap;}
  
  //! Unchecked access - no wrapping
  T & operator()(unsigned int i, unsigned int j) {
    return data[i + j*nx];}
  //! Unchecked const access
  const T & operator()(unsigned int i, unsigned int j) const {
    return data[i + j*nx];}
  //! checked access - unwraps if wrapping set
  T & at(int i, int j);
  //! checked const access
  const T & at(int i, int j) const;
  
  //! Calculate gradient using centered differences in the x dir
  void gradient_x();
  //! Calculate gradient using centered differences in the y dir
  void gradient_y();

  // sets each value to its absolute value by comparison with T(0)
  void abs();

  // requires T::operator<(...)
  T get_min() const;
  T get_max() const;

  // scale to fit within range...
  void set_range(T val_min, T val_max);

  //! add
  Array_2D<T> & add(const Array_2D<T> & rhs);
  //! add scalar
  template <class T1>
  Array_2D<T> & operator+=(const T1 & rhs)
    {
      for (unsigned int i = 0 ; i < nx*ny ; ++i) data[i] += rhs;
      return *this;
    }

  //! subtract
  Array_2D<T> & subtract(const Array_2D<T> & rhs);
  //! subtract scalar
  template <class T1>
  Array_2D<T> & operator-=(const T1 & rhs)
    {
      for (unsigned int i = 0 ; i < nx*ny ; ++i) data[i] -= rhs;
      return *this;
    }

  //! multiply
  Array_2D<T> & multiply(const Array_2D<T> & rhs);
  //! multiply scalar
  template <class T1>
  Array_2D<T> & operator*=(const T1 & rhs)
    {
      for (unsigned int i = 0 ; i < nx*ny ; ++i) data[i] *= rhs;
      return *this;
    }
  template <class T1>
  Array_2D<T> operator*(const T1 & rhs)
    {
      Array_2D<T> result(*this);
      result *= rhs;
      return result;
    }

  //! divide
  Array_2D<T> & divide(const Array_2D<T> & rhs);
  //! divide scalar
  template <class T1>
  Array_2D<T> & operator/=(const T1 & rhs)
    {
      for (unsigned int i = 0 ; i < nx*ny ; ++i) data[i] /= rhs;
      return *this;
    }

  //! raise to a power
  template <class T1>
  Array_2D<T> & pow(const T1 & rhs)
    {
      for (unsigned int i = 0 ; i < nx*ny ; ++i) data[i] = ::pow(data[i], rhs);
      return *this;
    }

  //! Apply a Gaussian filter with length scale r, extending over a
  //! square of half-width n (so n=1 uses a square of 9 points, n = 2
  //! uses 25 etc). Suggest using n at least 2*r.
  void gaussian_filter(double r, int n);

  //! return the 'x' size of the array
  unsigned int get_nx() const {return nx;}
  //! return the 'y' size of the array
  unsigned int get_ny() const {return ny;}
  
  /// shifts all the elements...
  void shift(int offset_x, int offset_y);
  
  //! dumps to file, suitable for reading into e.g. scilab. 0 Indicates
  //! success. Requires ability to cast to double
  int dump(const string & file_name) const;

private:
  inline void unwrap_indices(int & i, int & y) const;

  unsigned int nx, ny;
  T * data;
  bool m_wrap;
};

template<class T>
void Array_2D<T>::unwrap_indices(int & i, int & j) const
{
  if (m_wrap == false)
    return;
  
  while (i < 0)
    i += nx;
  while (j < 0)
    j += ny;
  
  i = i % nx;
  j = j % ny;
}


template<class T>
Array_2D<T>::Array_2D(int nx, int ny, const T & val)
  :
  nx(nx), ny(ny), data(new T[nx*ny]), m_wrap(false)
{
  unsigned int num = nx*ny;
  for (unsigned int i = 0 ; i < num ; ++i)
    data[i] = val;
}

template<class T>
Array_2D<T>::Array_2D(const Array_2D & orig)
  : nx(orig.nx), ny(orig.ny), data(new T[nx*ny]), m_wrap(orig.m_wrap)
{
  memcpy(data, orig.data, nx*ny*sizeof(T));
}

template<class T>
Array_2D<T> & Array_2D<T>::operator=(const Array_2D & rhs)
{
  if (&rhs == this)
    return;
  
  if (data)
    delete [] data;
  
  nx = rhs.nx;
  ny = rhs.ny;
  data = new T[nx*ny];

  m_wrap = rhs.m_wrap;

  memcpy(data, rhs.data, nx*ny*sizeof(T));
}

template<class T>
T & Array_2D<T>::at(int i, int j)
{
  unwrap_indices(i, j);
  return operator()(i, j);
}

template<class T>
const T & Array_2D<T>::at(int i, int j) const
{
  unwrap_indices(i, j);
  return operator()(i, j);
}

template<class T>
void Array_2D<T>::gradient_x()
{
  unsigned int i, j;

  // could do this much more efficiently by using temporaries on the
  // stack in the loop?
  const Array_2D<T> orig(*this);

  if (m_wrap == false)
  {
    for (j = 0 ; j < ny ; ++j)
    {
      operator()(0, j) = orig(1, j) - orig(0, j);
      operator()(nx-1, j) = orig(nx-1, j) - orig(nx-2, j);
      
      for (i = 1 ; i < (nx-1) ; ++i)
      {
        operator()(i, j) = (orig(i+1, j) - orig(i-1, j))/2;
      }
    }
  }
  else
  {
    for (j = 0 ; j < ny ; ++j)
    {
      for (i = 0 ; i < nx ; ++i)
      {
        operator()(i, j) = (orig.at(i+1, j) - orig.at(i-1, j))/2;
      }
    }
  }
}

template<class T>
void Array_2D<T>::gradient_y()
{
  unsigned int i, j;
  const Array_2D<T> orig(*this);
  
  if (m_wrap == false)
  {
    for (i = 0 ; i < nx ; ++i)
    {
      operator()(i, 0) = (orig(i, 1) - orig(i, 0))/2;
      operator()(i, ny-1) = (orig(i, ny-1) - orig(i, ny-2))/2;
      for (j = 1 ; j < (ny-1) ; ++j)
      {
        operator()(i, j) = (orig(i, j+1) - orig(i, j-1))/2;
      }
    }
  }
  else
  {
    for (j = 0 ; j < ny ; ++j)
    {
      for (i = 0 ; i < nx ; ++i)
      {
        operator()(i, j) = (orig.at(i, j+1) - orig.at(i, j-1))/2;
      }
    }
  }    
}

template<class T>
void Array_2D<T>::shift(int offset_x, int offset_y)
{
  Array_2D orig(*this);
  for (unsigned i = 0 ; i < nx ; ++i)
  {
    for (unsigned j = 0 ; j < ny ; ++j)
    {
      unsigned i0 = (i + offset_x) % nx;
      unsigned j0 = (j + offset_y) % ny;
      this->at(i0, j0) = orig.at(i, j);
    }
  }
}


template<class T>
void Array_2D<T>::gaussian_filter(double r, int n)
{
  int i, j, ii, jj, iii, jjj;

  int size = (n*2 + 1);
  double * filter = new double[size * size];

  for (i = 0 ; i < size ; ++i)
  {
    for (j = 0 ; j < size ; ++j)
    {
      filter[i + j * size] = exp ( -( (i-n) * (i-n) + (j-n) * (j-n) ) /
                                     ((double) r * r) );
    }
  }
  
  for (i = 0 ; i < (int) nx ; ++i)
  {
    for (j = 0 ; j < (int) ny ; ++j)
    {
      T total(0);
      double weight_total = 0;
      for (ii = -n ; ii < (int) n ; ++ii)
      {
        if ( ( ( (iii = i + ii) >= 0 ) && 
               (iii < (int) nx) ) ||
          ( m_wrap ) )
        {
          for (jj = -n ; jj < (int) n ; ++jj)
          {
            if ( ( ( (jjj = j + jj) >= 0 ) && 
                   (jjj < (int) ny) ) ||
              ( m_wrap ) )
            {
              // in a valid location
              int index = (n+ii) + (n+jj)*size;
              weight_total += filter[index];
              total += filter[index] * at(iii, jjj);
            }
          }
        }
      }
      operator()(i, j) = total / weight_total;
    }
  }
  delete [] filter;
}

template<class T>
int Array_2D<T>::dump(const string & file_name) const
{
  unsigned int i, j;

  FILE * file = fopen(file_name.c_str(), "wb");
  if (!file)
  {
    TRACE("Unable to open %s for writing\n", file_name.c_str());
    return -1;
  }
  
  fwrite(&nx, sizeof(nx), 1, file);
  fwrite(&ny, sizeof(ny), 1, file);

  // write out coords
  for (i = 0 ; i < nx ; ++i)
  {  
    double val = (double) i;
    fwrite(&val, sizeof(val), 1, file);
  }
  for (i = 0 ; i < ny ; ++i)
  {
    double val = (double) i;
    fwrite(&val, sizeof(val), 1, file);
  }
  
  // write out the array

  for (j = 0 ; j < ny ; ++j)
  {
    for (i = 0 ; i < nx ; ++i)
    {
      double val = (double) operator()(i, j);
      fwrite(&val, sizeof(val), 1, file);
    }
  }
  
  TRACE("Written array out to %s\n", file_name.c_str());

  fclose(file);
  return 0;

}

template<class T>
void Array_2D<T>::abs()
{
  unsigned int i;
  for (i = 0 ; i < nx*ny ; ++i)
  {
    if (data[i] < T(0))
      data[i] = -data[i];
  }
}


template<class T>
Array_2D<T> & Array_2D<T>::add(const Array_2D<T> & rhs)
{
  if ((rhs.nx != nx) || (rhs.ny != ny))
    throw range_error("Array_2D");

  unsigned int i;
  for (i = 0 ; i < nx*ny ; ++i)
      data[i] += rhs.data[i];

  return *this;

}

template<class T>
Array_2D<T> & Array_2D<T>::subtract(const Array_2D<T> & rhs)
{
  if ((rhs.nx != nx) || (rhs.ny != ny))
    abort();

  unsigned int i;
  for (i = 0 ; i < nx*ny ; ++i)
      data[i] -= rhs.data[i];

  return *this;
}

template<class T>
Array_2D<T> & Array_2D<T>::multiply(const Array_2D<T> & rhs)
{
  if ((rhs.nx != nx) || (rhs.ny != ny))
    abort();

  unsigned int i;
  for (i = 0 ; i < nx*ny ; ++i)
      data[i] *= rhs.data[i];

  return *this;
}

template<class T>
Array_2D<T> & Array_2D<T>::divide(const Array_2D<T> & rhs)
{
  if ((rhs.nx != nx) || (rhs.ny != ny))
    abort();

  unsigned int i;
  for (i = 0 ; i < nx*ny ; ++i)
      data[i] /= rhs.data[i];

  return *this;
}
 
template<class T>
T Array_2D<T>::get_min() const
{
  T min = data[0];
  for (unsigned i = 0 ; i < nx*ny ; ++i)
  {
    if (data[i] < min)
      min = data[i];
  }
  return min;
}

template<class T>
T Array_2D<T>::get_max() const
{
  
  T max = data[0];
  for (unsigned i = 0 ; i < nx*ny ; ++i)
  {
    if (max < data[i])
      max = data[i];
  }
  return max;
}

template<class T>
void Array_2D<T>::set_range(T val_min, T val_max)
{
  unsigned i;
  T orig_min = get_min();
  T orig_max = get_max();
  // set min to 0 and scale...
  double scale = (val_max - val_min) / (orig_max - orig_min);
  double offset = val_min - scale * orig_min;
  
  for (i = 0 ; i < nx*ny ; ++i)
  {
    data[i] = scale * data[i] + offset;
  }
}



#endif // file included
