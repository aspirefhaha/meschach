#pragma once

#include "matrix.h"
#include <math.h>
#define XTCOS cos
#define XTSIN sin
#define XTSQRT  sqrt
#define XTARCSIN asin
#define XTARCTAN2 atan2

#define DELTATIME 0.005 // 5 ms
#define USE_BODYOMEGA


//void init_AllSVals();
void init_Rxyz();
MAT * Rx(Real theta);
MAT * Ry(Real theta);
MAT * Rz(Real theta);
VEC * init_Quaternion(Real gamma,Real psi,Real fai);
VEC * update_Quaternion(Real dtx,Real dty, Real dtz,Real dt);

void init_CMatrixs();
void calc_Clb_Cbl();

#ifdef USE_BODYOMEGA
void init_OmegaBody();
VEC * update_OmegaBody(Real dtx,Real dty, Real dtz,Real dt);
#endif

Real update_Psi();
Real update_Fai();
Real update_Gamma();
