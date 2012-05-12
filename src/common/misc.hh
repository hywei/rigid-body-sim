/*
  Sss - a slope soaring simulater.
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk
*/

#ifndef SSS_MISC_H
#define SSS_MISC_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

// some useful angle things
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.28318530717958647692528676655901
#define PI_DIV_180 0.0174532925199432957692369076848861
#define _180_DIV_PI 57.2957795130823208767981548141052
inline double deg_to_rad(double deg) {return (double) (deg * PI_DIV_180);}
inline double rad_to_deg(double rad) {return (double) (rad * _180_DIV_PI);}

inline double sin_deg(double deg) {return (double) sin(deg_to_rad(deg));}
inline double cos_deg(double deg) {return (double) cos(deg_to_rad(deg));}
inline double asin_deg(double x) {return rad_to_deg(asin((double) x));}
inline double acos_deg(double x) {return rad_to_deg(acos((double) x));}
inline double tan_deg(double deg) {return (double) tan(deg_to_rad(deg));}
inline double atan2_deg(double x, double y) {return rad_to_deg((double) atan2(x, y));}
inline double atan_deg(double x) {return rad_to_deg((double) atan(x));}

// there is an STL version somewhere...
template<class T>
inline T sss_min(const T a, const T b) {return (a < b ? a : b);}
template<class T>
inline T sss_max(const T a, const T b) {return (a > b ? a : b);}

/*!
  Returns a random number between v1 and v2
*/
inline double ranged_random(double v1,double v2)
{
	return v1 + (v2-v1)*((double)rand())/((double)RAND_MAX);
}

/*! Indicates if the machine is little-endian */
bool is_little_endian();

//! Takes a 4-byte word and converts it to little endian (if necessary)
void convert_word_to_little_endian(void * orig);
//! Takes a 4-byte word and converts it from little endian to whatever
//! is this machine is (if necessary)
void convert_word_from_little_endian(void * orig);

template<typename T>
T square(T val) 
{
  return val * val;
}

#if 0

#if defined(__APPLE__) || defined(MACOSX) || defined(unix) || defined(linux)
#ifndef linux
#ifndef __APPLE__
#include <ieeefp.h>
#endif
#endif
inline bool is_finite(double val)
{
    return (finite(val) != 0);
}
#else
#include <float.h>
inline bool is_finite(double val)
{
    return (_finite(val) != 0);
}
#endif

#else // use a hand-crafted is_finite - catches more
inline bool is_finite(double val)
{
  return ((val < 1e8) && (val > -1e8));
}

#endif


#endif







