/*
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk
*/
#ifndef TYPES_H
#define TYPES_H

// bit hacky, but stick this here to get rid of STUPID MSVC warnings
#ifdef _MSC_VER
#pragma warning (disable : 4786)    // long names in debug info
#pragma warning (disable : 4355)    // 'this' : used in base member initializer list
#endif

#include "matrix_vector3.hh"

// The following refer to the global coordinate system...
typedef vec3d coord;
typedef vec3d vec3d;
typedef Matrix3 mat3d;
typedef vec3d vec3d; // apart from this, which is in the body's reference frame

typedef unsigned int uint;

#endif // file included
