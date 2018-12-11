#include <stdio.h>
#include "matrix2.h"
#include "xtpilot.h"


static MAT * sRx ;  //
static MAT * sRy ;  //
static MAT * sRz ;  //
static VEC * sQn ;  //
static MAT * sQMCalc ;   //Quaternion Matrix for update
static MAT * sClb;
static MAT * sCbl;


static VEC * slk ;  //calc factor in update quaternion

#ifdef USE_BODYOMEGA
static VEC * sOmegab;   // Omega in Body Coordinate System
#endif

void init_Rxyz()
{
  sRx = m_get(3,3);
  m_ident(sRx);
  sRy = m_get(3,3);
  m_ident(sRy);
  sRz = m_get(3,3);
  m_ident(sRy);
}

void init_CMatrixs()
{
  sClb = m_get(3,3);
  sCbl = m_get(3,3);
}

void calc_Clb_Cbl()
{
  Real * q = sQn->ve;
  sClb->me[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  sClb->me[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
  sClb->me[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
  sClb->me[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
  sClb->me[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3]* q[3];
  sClb->me[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
  sClb->me[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);
  sClb->me[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);
  sClb->me[2][2] = q[0] * q[0] - q[1] * q[1] - q[2]* q[2] + q[3] * q[3];
  sCbl = m_transp(sClb,sCbl);
}

//TODO use Clb
Real update_Psi()
{
  return XTARCTAN2(sCbl->me[0][1],sCbl->me[0][0]);
}

//TODO use Clb
Real update_Fai()
{
  return XTARCSIN(-sCbl->me[2][0]);
}

//TODO use Clb
Real update_Gamma()
{
  return XTARCTAN2(sCbl->me[1][2],sCbl->me[2][2]);
}

#ifdef USE_BODYOMEGA
void init_OmegaBody()
{
  sOmegab = v_get(3);
}

VEC * update_OmegaBody(Real dtx,Real dty, Real dtz,Real dt)
{
  sOmegab->ve[0] = dtx / dt;
  sOmegab->ve[1] = dty / dt;
  sOmegab->ve[2] = dtz / dt;
}
#endif

MAT * mk_QMatrix()
{
  sQMCalc->me[0][0] = sQMCalc->me[1][1]
  = sQMCalc->me[2][2] = sQMCalc->me[3][3] = sQn->ve[0];
  sQMCalc->me[0][1] = sQMCalc->me[2][3] = - sQn->ve[1];
  sQMCalc->me[1][0] = sQMCalc->me[3][2] = sQn->ve[1];
  sQMCalc->me[0][2] = sQMCalc->me[3][1] = - sQn->ve[2];
  sQMCalc->me[2][0] = sQMCalc->me[1][3] = sQn->ve[2];
  sQMCalc->me[0][3] = sQMCalc->me[1][2] = - sQn->ve[3];
  sQMCalc->me[3][0] = sQMCalc->me[2][1] = sQn->ve[3];
  return sQMCalc;
}

VEC * init_Quaternion(Real gamma,Real psi,Real fai)
{
  sQn = v_get(4);
  sQMCalc = m_get(4,4);
  Real cosg2 = XTCOS(gamma/2);
  Real cosp2 = XTCOS(psi/2);
  Real cosf2 = XTCOS(fai/2);
  Real sing2 = XTSIN(gamma/2);
  Real sinp2 = XTSIN(psi/2);
  Real sinf2 = XTSIN(fai/2);
  sQn->ve[0] = cosg2 * cosp2 * cosf2 + sing2 * sinp2 * sinf2;
  sQn->ve[1] = sing2 * cosp2 * cosf2 - cosg2 * sinp2 * sinf2;
  sQn->ve[2] = cosg2 * sinp2 * cosf2 + sing2 * cosp2 * sinf2;
  sQn->ve[3] = cosg2 * cosp2 * sinf2 - sing2 * sinp2 * cosf2;
  mk_QMatrix();
  return sQn;
}


//TODO verify dt is degree or radian
VEC * update_Quaternion(Real dtx,Real dty, Real dtz,Real dt)
{
  //TODO make quaternion matrix for calc
  // mk_QMatrix();

  Real dtheta2 = dtx * dtx + dty * dty + dtz  * dtz;
  Real ac = 1- dtheta2 / 8;
  Real as = 0.5 - dtheta2 / 28;
  slk->ve[0] = ac;
  slk->ve[1] = as * dtx;
  slk->ve[2] = as * dty;
  slk->ve[3] = as * dtz;
  mv_mlt(sQMCalc,slk,sQn);
  sv_mlt(v_norm2(sQn),sQn,sQn); // normalize
  mk_QMatrix();
#ifdef USE_BODYOMEGA
  update_OmegaBody(dtx,dty,dtz,dt);
#endif
  return sQn;
}

MAT * Rx(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  sRx->me[1][1] = cosv;
  sRx->me[2][2] = cosv;
  sRx->me[1][2] = sinv;
  sRx->me[2][1] = - sinv;
  return sRx;
}

MAT * Ry(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  sRy->me[0][0] = cosv;
  sRy->me[2][2] = cosv;
  sRy->me[0][2] = -sinv;
  sRy->me[2][0] = sinv;
  return sRy;
}

MAT * Rz(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  sRz->me[0][0] = cosv;
  sRz->me[1][1] = cosv;
  //TODO
  sRz->me[0][1] = sinv;
  sRz->me[1][0] = -sinv;
  return sRz;
}

void init_AllSVals()
{
  init_Rxyz();
  init_CMatrixs();
#ifdef USE_BODYOMEGA
  init_OmegaBody();
#endif
  //TODO
  //init_Quaternion();
  slk = v_get(4);
}
