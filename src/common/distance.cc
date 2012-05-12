#include "distance.hh"
#include "log_trace.hh"

inline double my_abs(double val) {return (val > (double) 0.0 ? val : -val);}


// distance_sqr
double distance_sqr(const coord & point,
                    const Triangle & triangle,
                    double * SParam,
                    double * TParam)
{
  vec3d kDiff = triangle.origin - point;
  double fA00 = triangle.edge0.mag2();
  double fA01 = dot(triangle.edge0,triangle.edge1);
  double fA11 = triangle.edge1.mag2();
  double fB0 = dot(kDiff, triangle.edge0);
  double fB1 = dot(kDiff, triangle.edge1);
  double fC = kDiff.mag2();
  double fDet = my_abs(fA00*fA11-fA01*fA01);
  double fS = fA01*fB1-fA11*fB0;
  double fT = fA01*fB0-fA00*fB1;
  double fSqrDist;
  
  if ( fS + fT <= fDet )
  {
    if ( fS < (double)0.0 )
    {
      if ( fT < (double)0.0 )  // region 4
      {
        if ( fB0 < (double)0.0 )
        {
          fT = (double)0.0;
          if ( -fB0 >= fA00 )
          {
            fS = (double)1.0;
            fSqrDist = fA00+((double)2.0)*fB0+fC;
          }
          else
          {
            fS = -fB0/fA00;
            fSqrDist = fB0*fS+fC;
          }
        }
        else
        {
          fS = (double)0.0;
          if ( fB1 >= (double)0.0 )
          {
            fT = (double)0.0;
            fSqrDist = fC;
          }
          else if ( -fB1 >= fA11 )
          {
            fT = (double)1.0;
            fSqrDist = fA11+((double)2.0)*fB1+fC;
          }
          else
          {
            fT = -fB1/fA11;
            fSqrDist = fB1*fT+fC;
          }
        }
      }
      else  // region 3
      {
        fS = (double)0.0;
        if ( fB1 >= (double)0.0 )
        {
          fT = (double)0.0;
          fSqrDist = fC;
        }
        else if ( -fB1 >= fA11 )
        {
          fT = (double)1.0;
          fSqrDist = fA11+((double)2.0)*fB1+fC;
        }
        else
        {
          fT = -fB1/fA11;
          fSqrDist = fB1*fT+fC;
        }
      }
    }
    else if ( fT < (double)0.0 )  // region 5
    {
      fT = (double)0.0;
      if ( fB0 >= (double)0.0 )
      {
        fS = (double)0.0;
        fSqrDist = fC;
      }
      else if ( -fB0 >= fA00 )
      {
        fS = (double)1.0;
        fSqrDist = fA00+((double)2.0)*fB0+fC;
      }
      else
      {
        fS = -fB0/fA00;
        fSqrDist = fB0*fS+fC;
      }
    }
    else  // region 0
    {
      // minimum at interior point
      double fInvDet = ((double)1.0)/fDet;
      fS *= fInvDet;
      fT *= fInvDet;
      fSqrDist = fS*(fA00*fS+fA01*fT+((double)2.0)*fB0) +
        fT*(fA01*fS+fA11*fT+((double)2.0)*fB1)+fC;
    }
  }
  else
  {
    double fTmp0, fTmp1, fNumer, fDenom;
    
    if ( fS < (double)0.0 )  // region 2
    {
      fTmp0 = fA01 + fB0;
      fTmp1 = fA11 + fB1;
      if ( fTmp1 > fTmp0 )
      {
        fNumer = fTmp1 - fTmp0;
        fDenom = fA00-2.0f*fA01+fA11;
        if ( fNumer >= fDenom )
        {
          fS = (double)1.0;
          fT = (double)0.0;
          fSqrDist = fA00+((double)2.0)*fB0+fC;
        }
        else
        {
          fS = fNumer/fDenom;
          fT = (double)1.0 - fS;
          fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
            fT*(fA01*fS+fA11*fT+((double)2.0)*fB1)+fC;
        }
      }
      else
      {
        fS = (double)0.0;
        if ( fTmp1 <= (double)0.0 )
        {
          fT = (double)1.0;
          fSqrDist = fA11+((double)2.0)*fB1+fC;
        }
        else if ( fB1 >= (double)0.0 )
        {
          fT = (double)0.0;
          fSqrDist = fC;
        }
        else
        {
          fT = -fB1/fA11;
          fSqrDist = fB1*fT+fC;
        }
      }
    }
    else if ( fT < (double)0.0 )  // region 6
    {
      fTmp0 = fA01 + fB1;
      fTmp1 = fA00 + fB0;
      if ( fTmp1 > fTmp0 )
      {
        fNumer = fTmp1 - fTmp0;
        fDenom = fA00-((double)2.0)*fA01+fA11;
        if ( fNumer >= fDenom )
        {
          fT = (double)1.0;
          fS = (double)0.0;
          fSqrDist = fA11+((double)2.0)*fB1+fC;
        }
        else
        {
          fT = fNumer/fDenom;
          fS = (double)1.0 - fT;
          fSqrDist = fS*(fA00*fS+fA01*fT+((double)2.0)*fB0) +
            fT*(fA01*fS+fA11*fT+((double)2.0)*fB1)+fC;
        }
      }
      else
      {
        fT = (double)0.0;
        if ( fTmp1 <= (double)0.0 )
        {
          fS = (double)1.0;
          fSqrDist = fA00+((double)2.0)*fB0+fC;
        }
        else if ( fB0 >= (double)0.0 )
        {
          fS = (double)0.0;
          fSqrDist = fC;
        }
        else
        {
          fS = -fB0/fA00;
          fSqrDist = fB0*fS+fC;
        }
      }
    }
    else  // region 1
    {
      fNumer = fA11 + fB1 - fA01 - fB0;
      if ( fNumer <= (double)0.0 )
      {
        fS = (double)0.0;
        fT = (double)1.0;
        fSqrDist = fA11+((double)2.0)*fB1+fC;
      }
      else
      {
        fDenom = fA00-2.0f*fA01+fA11;
        if ( fNumer >= fDenom )
        {
          fS = (double)1.0;
          fT = (double)0.0;
          fSqrDist = fA00+((double)2.0)*fB0+fC;
        }
        else
        {
          fS = fNumer/fDenom;
          fT = (double)1.0 - fS;
          fSqrDist = fS*(fA00*fS+fA01*fT+((double)2.0)*fB0) +
            fT*(fA01*fS+fA11*fT+((double)2.0)*fB1)+fC;
        }
      }
    }
    }
    
    if ( SParam )
      *SParam = fS;
    
    if ( TParam )
      *TParam = fT;
    
    return my_abs(fSqrDist);}

    // distance
    double distance(const coord & point,
      const Triangle & triangle,
      double * SParam,
      double * TParam)
    {
      return sqrt(distance_sqr(point, triangle, SParam, TParam));
    }
    
    //==============================================================
    // distance_sqr_point_triangle
    //==============================================================
    double distance_sqr_point_triangle(const coord & point,
      const coord & tri_pos0,
      const coord & tri_pos1,
      const coord & tri_pos2,
      coord & point_on_triangle,
      bool & point_on_triangle_edge)
    {
      Triangle triangle(tri_pos0, tri_pos1 - tri_pos0, tri_pos2 - tri_pos0);
      double S, T;
      
      double dist_sqr = distance_sqr(point, triangle, &S, &T);
      point_on_triangle = triangle.origin + S * triangle.edge0 + T * triangle.edge1;
      /*
      TRACE("=======================\n");
      point.show("point");
      tri_pos0.show("tri0");
      tri_pos1.show("tri1");
      tri_pos2.show("tri2");
      point_on_triangle.show("tri point");
      TRACE("distance = %5.2f\n", sqrt(dist_sqr));
      */
      const double epsilon = 0.00001f;
      if ( (S < epsilon) ||
        (T < epsilon) ||
        ( (S + T) > (0.5f - epsilon) ) )
        point_on_triangle_edge = true;
      else
        point_on_triangle_edge = false;
      
      return dist_sqr;
    }
    
//==============================================================
// distance_sqr_point_ellipsoid
//==============================================================
double distance_sqr(vec3d rkPoint, 
  const Ellipsoid_shape & rkEllipsoid,
  vec3d & rkClosest)
{
#define ACCURATE_BUT_WRONG
#ifdef ACCURATE_BUT_WRONG
  // goes wrong when the dimensions are < 1 or so... due to numerical accuracy I think.
  // so scale everything up...
  const double scale = 10.0f;
    
  const vec3d afExtent = scale * rkEllipsoid.extents;
  rkPoint *= scale;

  double fA2 = afExtent[0]*afExtent[0];
  double fB2 = afExtent[1]*afExtent[1];
  double fC2 = afExtent[2]*afExtent[2];
  double fU2 = rkPoint[0]*rkPoint[0];
  double fV2 = rkPoint[1]*rkPoint[1];
  double fW2 = rkPoint[2]*rkPoint[2];
  double fA2U2 = fA2*fU2, fB2V2 = fB2*fV2, fC2W2 = fC2*fW2;
  
  // initial guess
  double fURatio = rkPoint[0]/afExtent[0];
  double fVRatio = rkPoint[1]/afExtent[1];
  double fWRatio = rkPoint[2]/afExtent[2];
  double fT;
  if ( fURatio*fURatio+fVRatio*fVRatio+fWRatio*fWRatio < (double)1.0 )
  {
    fT = (double)0.0f;
  }
  else
  {
    double fMax = afExtent[0];
    if ( afExtent[1] > fMax )
      fMax = afExtent[1];
    if ( afExtent[2] > fMax )
      fMax = afExtent[2];
    
    fT = fMax*rkPoint.mag();
  }
  
  // Newton's method
  const int iMaxIteration = 64;
  double fP, fQ, fR;
  for (int i = 0; i < iMaxIteration; i++)
  {
    fP = fT+fA2;
    fQ = fT+fB2;
    fR = fT+fC2;
    double fP2 = fP*fP;
    double fQ2 = fQ*fQ;
    double fR2 = fR*fR;
    double fS = fP2*fQ2*fR2-fA2U2*fQ2*fR2-fB2V2*fP2*fR2-fC2W2*fP2*fQ2;
    if ( fabs(fS) < 1e-16 )
      break;
    
    double fPQ = fP*fQ, fPR = fP*fR, fQR = fQ*fR, fPQR = fP*fQ*fR;
    double fDS = ((double)2.0)*(fPQR*(fQR+fPR+fPQ)-fA2U2*fQR*(fQ+fR)-
      fB2V2*fPR*(fP+fR)-fC2W2*fPQ*(fP+fQ));
    fT -= fS/fDS;
  }
  
  rkClosest[0] = fA2*rkPoint[0]/fP;
  rkClosest[1] = fB2*rkPoint[1]/fQ;
  rkClosest[2] = fC2*rkPoint[2]/fR;
  vec3d kDiff = rkClosest - rkPoint;
  rkClosest /= scale;
  kDiff /= scale;
  return kDiff.mag2();
#else
  // this is inaccurate, but gives a sensible result for in and out
  coord position = rkPoint;
  const vec3d & afExtent = rkEllipsoid.extents;
  position[0] /= afExtent[0];
  position[1] /= afExtent[1];
  position[2] /= afExtent[2];
  
  double rad = position.mag();
  rkClosest = position * 1.0f / rad;
  
  rkClosest[0] *= afExtent[0];
  rkClosest[1] *= afExtent[1];
  rkClosest[2] *= afExtent[2];
  
  vec3d kDiff = rkClosest - rkPoint;
  return kDiff.mag2();
#endif
}
