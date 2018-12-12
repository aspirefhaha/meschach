#pragma once

#include "matrix.h"
#include <math.h>
#define XTCOS cos
#define XTSIN sin
#define XTSQRT  sqrt
#define XTARCSIN asin
#define XTARCTAN2 atan2
#define XTARCTAN  atan

#define DELTATIME 0.005 // 5 ms

//TODO verify body omega
#define USE_BODYOMEGA

//TODO verify L is what
#define XT_L  0.0


#define XT_PI 3.14159265
#define A0 (270.6/XT_PI)  //TODO degree? launching direction
#define B0 (41.14/XT_PI)  //TODO degree? latitude of launching point
#define LAMBDA0 (100.25169/XT_PI) //TODO degree? longitude of launching point
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



//void init_AllSVals();
void init_Rxyz();
MAT * Rx(Real theta);
MAT * Ry(Real theta);
MAT * Rz(Real theta);
VEC * init_Quaternion();
VEC * update_Quaternion(Real dtx,Real dty, Real dtz,Real dt);

void calc_Clb_Cbl();
void init_CoordinateTransformMatrix();

#ifdef USE_BODYOMEGA
void init_OmegaBody();
VEC * update_OmegaBody(Real dtx,Real dty, Real dtz,Real dt);
#endif

Real update_Psi();
Real update_Fai();
Real update_Gamma();

void init_naviProc();
