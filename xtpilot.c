#include <stdio.h>
#include "matrix2.h"
#include "xtpilot.h"


// posture calc matrixes and vectors
static MAT * smRx ;  //Rotate Matrix for x
static MAT * smRy ;  //Rotate Matrix for y
static MAT * smRz ;  //Rotate Matrix for z
static VEC * svQn ;  //
static MAT * smQMCalc ;   //Quaternion Matrix for update

// matrixes and vectors for coordinate-transformation
static VEC * svOmegae;
static VEC * svR0; //for calc faie in update B(latitude)
static VEC * svr;
static VEC * svRnormal;
static Real sMue;
static Real sFaie;
static Real sR;     //
static Real sr;
static Real sB;     //latitude
static Real sLambda;  //longitude
static VEC * svMovemente;
static VEC * svMovementI;

//coordinate transofrormation Matrixes
static MAT * smCgI;
static MAT * smCIb;
static MAT * smCbI;
static MAT * smCgE;
static MAT * smCTE;
static MAT * smCnb;

//Posture
static Real sFai;
static Real sPsi;
static Real sGamma;


static VEC * svlk ;  //calc factor in update quaternion

#ifdef USE_BODYOMEGA
static VEC * svOmegab;   // Omega in Body Coordinate System
#endif

void update_CoordinateTransformationMatrix(Real dt)
{
  // Calc CgI
  m_mlt(Ry(-A0),Rz(-B0),smCgI);
  m_mlt(smCgI,Rx(dt * OMEGAe),smCgI);
  m_mlt(smCgI,Rz(B0),smCgI);
  m_mlt(smCgI,Ry(A0),smCgI);

  //Calc CbI
  m_mlt(Rx(sGamma),Ry(sPsi),smCbI);
  m_mlt(smCbI,Rz(sFai),smCbI);

  //Calc CgE
  m_mlt(Ry(- XT_PI /2 - A0),Rx(B0),smCgE);
  m_mlt(smCgE,Rz(sLambda - LAMBDA0 - XT_PI /2 ),smCgE);

  //Calc CTE
  m_mlt(Ry(- XT_PI /2),Rx(sB),smCTE);
  m_mlt(smCTE,Rz(sLambda - LAMBDA0 - XT_PI/2),smCTE);
}

void init_CoordinateTransformMatrix()
{
  //malloc for all coordinate transformation matrix
  smCIb = m_get(3,3);
  m_ident(smCIb);
  smCbI = m_get(3,3);
  m_ident(smCbI);
  smCgI = m_get(3,3);
  m_ident(smCgI);
  smCgE = m_get(3,3);
  m_ident(smCgE);
  smCTE = m_get(3,3);
  m_ident(smCTE);
  smCnb = m_get(3,3);
  m_ident(smCnb);

  //step 0
  svOmegae = v_get(3);
  svOmegae->ve[0] = XTCOS(B0) * XTCOS(A0);
  svOmegae->ve[1] = XTSIN(B0);
  svOmegae->ve[2] = - XTCOS(B0) * XTSIN(A0);

  svr = v_get(3);

  svR0 = v_get(3);

  svRnormal = v_get(3);

  svMovementI = v_get(3);
  v_zero(svMovementI);

  //TODO Faie 0 is 0
  Real Faie0 = 0;

  //step 0.1
  sMue = EP2 * XTSIN(2* sFaie ) * ( 1 - EP2 * XTSIN(sFaie) * XTSIN(sFaie)) / 2.0;
  //step 1
  sR = Ae * ( 1- ALPHAe) / XTSQRT( XTSIN(sFaie) * XTSIN(sFaie) + (1 - ALPHAe) * (1 - ALPHAe) * XTCOS(sFaie) * XTCOS(sFaie)) * H0;
  //step 2
  svMovemente = v_get(3);
  v_zero(svMovemente);
  svMovemente->ve[0] = sR * XTCOS(Faie0);
  svMovemente->ve[2] = sR * XTSIN(Faie0);

  //step 3 Calc svR0
  svR0->ve[0] = sR * (-XTSIN(sMue) * XTCOS(A0));
  svR0->ve[1] = sR * XTCOS(sMue);
  svR0->ve[2] = sR * XTSIN(sMue) * XTSIN(A0);

  //step 4
  /* svr =  */ v_add(svR0,svMovementI,svr);
  sr = v_norm2(svr);

  //step 5
  Real rreciprocal = 1/ sr;
  /* svRnormal = */ sv_mlt(rreciprocal,svr,svRnormal);
  //step 6
  sFaie = XTARCSIN(in_prod(svRnormal,svOmegae));
  //step 8.1
  sB = sFaie + sMue;
  //step 8.2
  sLambda = LAMBDA0 + XTARCTAN(svMovemente->ve[1]/svMovemente->ve[0]);


  //Initialize Coordinate Transformation Matrixes
  update_CoordinateTransformationMatrix(0.0);
}

void init_Rxyz()
{
  smRx = m_get(3,3);
  m_ident(smRx);
  smRy = m_get(3,3);
  m_ident(smRy);
  smRz = m_get(3,3);
  m_ident(smRy);
}

void init_Posture()
{
  sFai = XT_PI /2;
  sPsi = 0;
  sGamma = 2 * XT_PI / 180;
}

void calc_CIb_CbI()
{
  Real * q = svQn->ve;
  smCIb->me[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  smCIb->me[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
  smCIb->me[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
  smCIb->me[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
  smCIb->me[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3]* q[3];
  smCIb->me[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
  smCIb->me[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);
  smCIb->me[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);
  smCIb->me[2][2] = q[0] * q[0] - q[1] * q[1] - q[2]* q[2] + q[3] * q[3];
  smCbI = m_transp(smCIb,smCbI);
}

//TODO use CIb
Real update_Psi()
{
  return XTARCTAN2(smCbI->me[0][1],smCbI->me[0][0]);
}

//TODO use CIb
Real update_Fai()
{
  return XTARCSIN(-smCbI->me[2][0]);
}

//TODO use CIb
Real update_Gamma()
{
  return XTARCTAN2(smCbI->me[1][2],smCbI->me[2][2]);
}

#ifdef USE_BODYOMEGA
void init_OmegaBody()
{
  svOmegab = v_get(3);
}

VEC * update_OmegaBody(Real dtx,Real dty, Real dtz,Real dt)
{
  svOmegab->ve[0] = dtx / dt;
  svOmegab->ve[1] = dty / dt;
  svOmegab->ve[2] = dtz / dt;
}
#endif

MAT * mk_QMatrix()
{
  smQMCalc->me[0][0] = smQMCalc->me[1][1]
  = smQMCalc->me[2][2] = smQMCalc->me[3][3] = svQn->ve[0];
  smQMCalc->me[0][1] = smQMCalc->me[2][3] = - svQn->ve[1];
  smQMCalc->me[1][0] = smQMCalc->me[3][2] = svQn->ve[1];
  smQMCalc->me[0][2] = smQMCalc->me[3][1] = - svQn->ve[2];
  smQMCalc->me[2][0] = smQMCalc->me[1][3] = svQn->ve[2];
  smQMCalc->me[0][3] = smQMCalc->me[1][2] = - svQn->ve[3];
  smQMCalc->me[3][0] = smQMCalc->me[2][1] = svQn->ve[3];
  return smQMCalc;
}

VEC * init_Quaternion()
{
  svQn = v_get(4);
  smQMCalc = m_get(4,4);
  Real cosg2 = XTCOS(sGamma/2);
  Real cosp2 = XTCOS(sPsi/2);
  Real cosf2 = XTCOS(sFai/2);
  Real sing2 = XTSIN(sGamma/2);
  Real sinp2 = XTSIN(sPsi/2);
  Real sinf2 = XTSIN(sFai/2);
  svQn->ve[0] = cosg2 * cosp2 * cosf2 + sing2 * sinp2 * sinf2;
  svQn->ve[1] = sing2 * cosp2 * cosf2 - cosg2 * sinp2 * sinf2;
  svQn->ve[2] = cosg2 * sinp2 * cosf2 + sing2 * cosp2 * sinf2;
  svQn->ve[3] = cosg2 * cosp2 * sinf2 - sing2 * sinp2 * cosf2;
  mk_QMatrix();
  svlk = v_get(4);
  return svQn;
}

VEC * update_Quaternion(Real dtx,Real dty, Real dtz,Real dt)
{
  Real dtheta2 = dtx * dtx + dty * dty + dtz  * dtz;
  Real ac = 1- dtheta2 / 8;
  Real as = 0.5 - dtheta2 / 28;
  svlk->ve[0] = ac;
  svlk->ve[1] = as * dtx;
  svlk->ve[2] = as * dty;
  svlk->ve[3] = as * dtz;
  mv_mlt(smQMCalc,svlk,svQn);
  sv_mlt(v_norm2(svQn),svQn,svQn); // normalize
  mk_QMatrix();
#ifdef USE_BODYOMEGA
  update_OmegaBody(dtx,dty,dtz,dt);
#endif
  return svQn;
}

MAT * Rx(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  smRx->me[1][1] = cosv;
  smRx->me[2][2] = cosv;
  smRx->me[1][2] = sinv;
  smRx->me[2][1] = - sinv;
  return smRx;
}

MAT * Ry(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  smRy->me[0][0] = cosv;
  smRy->me[2][2] = cosv;
  smRy->me[0][2] = -sinv;
  smRy->me[2][0] = sinv;
  return smRy;
}

MAT * Rz(Real theta)
{
  Real cosv = XTCOS(theta);
  Real sinv = XTSIN(theta);
  smRz->me[0][0] = cosv;
  smRz->me[1][1] = cosv;
  //TODO
  smRz->me[0][1] = sinv;
  smRz->me[1][0] = -sinv;
  return smRz;
}

// update latitude
void update_B()
{
  sFaie = XTARCSIN(in_prod(svR0,svOmegae));
}

void init_AllSVals()
{
  init_Rxyz();

#ifdef USE_BODYOMEGA
  init_OmegaBody();
#endif

  init_Posture();
  init_CoordinateTransformMatrix();

  init_Quaternion();

}

//TODO verify calc formula and unit
void init_RoughAim(Real dvx,Real dvy,Real dvz,Real dwx,Real dwy,Real dwz,Real dt)
{
  Real g = g0 * ( sR / sr);
  //TODO verify formula and unit
  Real t31,t32,t33,t21,t22,t23;
  t31 =  dvx / g / dt;
  smCnb->me[2][0] = t31;
  t32 = dvy / g / dt;
  smCnb->me[2][1] = t32;
  t33 = dvz / g / dt;
  smCnb->me[2][2] = t33;

  t21 = 1 / ( dt * OMEGAe * XTCOS(B0)) * ( dwx - dt * t31 * OMEGAe * XTSIN(XT_L));
  smCnb->me[1][0] = t21;
  t22 = 1 / ( dt * OMEGAe * XTCOS(B0)) * ( dwy - dt * t32 * OMEGAe * XTSIN(XT_L));
  smCnb->me[1][1] = t22;
  t23 = 1 / ( dt * OMEGAe * XTCOS(B0)) * ( dwz - dt * t33 * OMEGAe * XTSIN(XT_L));
  smCnb->me[1][2] = t23;

  smCnb->me[0][0] = t22 * t33 - t23 * t32;
  smCnb->me[0][1] = t23 * t31 - t21 * t33;
  smCnb->me[0][2] = t21 * t32 - t22 * t31;

}
