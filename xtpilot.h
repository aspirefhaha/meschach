#pragma once

#include "matrix.h"
#include <math.h>
#define XTCOS cos
#define XTSIN sin
#define XTSQRT  sqrt
#define XTARCSIN asin
#define XTARCTAN2 atan2
#define XTARCTAN  atan
#define XTEXP exp

#define DELTATIME 0.005 // 5 ms

//TODO verify body omega
#define USE_BODYOMEGA

//TODO verify L is what
#define XT_L  0.0


#define XT_PI 3.14159265
#define A0 (270.6/XT_PI)  //TODO degree? launching direction
#define B0 (41.14/XT_PI)  //TODO degree? latitude of launching point
#define LAMBDA0 (100.25169/XT_PI) //TODO degree? longitude of launching point
#define R2D (57.29578)
#define D2R   (1/R2D)
#define EPSMIN  (1e-15)
#define H0 (1073.0) //meter elevation of launching point
#define g0 (9.80665)  //m/s2 gravitational acceleration on earth surface
#define Ae  (6378140.0) //meter earth long radius
#define Be  (6356755.28856) //meter earth short radius
#define fM  (398600500000000.0) //geocentric graviational constant
#define J2  (0.00108260)  //geodynamic shape factor
#define XT_J   (J2 * 1.5)
#define OMEGAe  (0.00007292115) //radius per second
#define ALPHAe  (1/298.257) //earth oblateness
#define E2  (0.00669438487525)  // square of first eccentricity ratio
#define EP2 (0.00673950169345)  // square of second eccentricity ratio

#define KCS 1.4 // K coefficient of gas
#define RSTAR 287.05287 // R* R ocefficent of gas
#define XT_E 2.718281828 // for nature base


#define XT_Tm (0.005)      //5 ms

#define SIGNFUNC(k) ( k>=0?1:-1)




//void init_AllSVals();
void init_Rxyz();
MAT * Rx(Real theta);
MAT * Ry(Real theta);
MAT * Rz(Real theta);
VEC * init_Quaternion();
VEC * update_Quaternion(Real dtx,Real dty, Real dtz);

void calc_Clb_Cbl();
void init_CoordinateTransformMatrix();
void calc_CoordinatetransformationMatrix();

#ifdef USE_BODYOMEGA
void init_OmegaBody();
VEC * update_OmegaBody(Real dtx,Real dty, Real dtz);
#endif

void update_Posture();

void init_naviProc();

void init_RoughAim(Real dvx,Real dvy,Real dvz,Real dwx,Real dwy,Real dwz);
void bind_Paratmeters();
void calc_atmosphere(Real height,VEC * vI);

#define get_linearinterpolation(x1,x2,y1,y2,xk) \
  (y1 + (y2-y1)/(x2-x1) * (xk - x1))

#define LINEAR_INTER(st,id,fname1,fname2,xk)  get_linearinterpolation(st[id].fname1,st[id+1].fname1,st[id].fname2,st[id+1].fname2,xk)
