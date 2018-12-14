#include <stdio.h>
#include "matrix2.h"
#include "xtpilot.h"
#include "naviguide.h"


static Real sDeltaTime;

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
static Real sr;   //normalize value of svr
static Real sB;     //latitude
static Real sLambda;  //longitude
static VEC * svMovemente;
static VEC * svMovementI;

static Real sH;
static Real sThetaT;

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

//calc factor in update quaternion
static VEC * svlk ;

#ifdef USE_BODYOMEGA
static VEC * svOmegab;   // Omega in Body Coordinate System
#endif

//navigation calc vectors and Matrixes
//for mid calc Gn
static MAT * sm33I;
static MAT * smrI3;
static VEC * svk2k3;
static VEC * sv1;
//for save gn gn-1 vn vn-1
static VEC * svGn;
static VEC * svGn1;
static VEC * svVI;
static VEC * svVI1;
static VEC * svVT;
static VEC * svMovementI;
static VEC * svMovementI1;

static Real skineticPressure;
static Real sMach;
static Real sAirPressure;

static Real sbAtomRow1;
static Real sbAtmoRow2;
static Real sbAtmoHeight1;
static Real sbAtmoHeight2;

static Real sTheta;   // theta in g coordination
static Real sEpsilon;
static Real sAlpha;
static Real sBeta;
static VEC * svVg;
static VEC * svVb;

static Real sGuideNormal;
static Real sGuideLand;

void init_Guidance()
{
  svVg = v_get(3);
  v_zero(svVg);
  svVb = v_get(3);
  v_zero(svVb);
}

void calc_GuidanceParameters()
{
  mv_mlt(smCgI,svVI,svVg);
  Real * vg = svVg->ve;
  mv_mlt(smCbI,svVI,svVb);
  Real * vb = svVb->ve;
  Real tVg = v_norm2(svVg);
  Real tVb = v_norm2(svVb);
  if(vg[0]>=0){ //vgx >= 0
    sTheta = XTARCSIN(vg[1]/XTSQRT(vg[0] * vg[0] + vg[1] * vg[1]));
  }
  else{ //vgx<0
    sTheta = XT_PI - XTARCSIN(vg[1]/XTSQRT(vg[0] * vg[0] + vg[1] * vg[1]));
  }
  sEpsilon = XTARCSIN( - vg[2] / tVg);

  if(vb[0]>=0){ //vbx >= 0
    sAlpha = XTARCSIN(vb[1] / XTSQRT(vb[0] * vb[0] + vb[1] * vb[1]));
    sBeta = XTARCSIN( vb[1] / tVb);
  }
  else {  //vbx < 0
    sAlpha = SIGNFUNC(vb[1]) * XT_PI - XTARCSIN( vb[1] / XTSQRT(vb[0] * vb[0] + vb[1] * vb[1]));
    sBeta = XT_PI * SIGNFUNC(vb[2]) - XTARCSIN(vb[2] / tVb);
  }
}

//TODO bind and get trajectory
Real get_trajectoryTheta()
{
  //get delta time and height
  return 0;
}

//TODO bind and get trajectory
Real get_trjactoryHeight()
{
  //get delta time and height
  return 0;
}

//TODO
Real get_k1fai()
{
  return 0;
}

Real get_k2fai()
{

}

Real get_k1psi()
{

}

Real get_k2psi()
{

}

void calc_GuidanceValues()
{
  Real deltaTheta = get_trajectoryTheta() - sTheta;
  Real deltaH = get_trjactoryHeight() - sH;
  if(sDeltaTime <= 6){
    sGuideNormal = 0;
    sGuideLand = 0;
  }
  else if(sDeltaTime <= 8){
    //sGuideNormal = (sDeltaTime - 6) / 2.0 * ( k1fai * );
  }
}

//TODO
void bind_Paratmeters()
{

}

void calc_atmosphere(Real height,VEC * vI)
{
  //step 1
  Real vnorm = v_norm2(vI);

  //TODO  in bind func calc
  Real beta = log(sbAtmoRow2/sbAtomRow1)/(sbAtmoHeight2-sbAtmoHeight1);
  Real rou = sbAtomRow1*XTEXP(beta*(height-sbAtmoHeight1));
  skineticPressure = 0.5 * rou * vnorm * vnorm;

  //step2
  //TODO calc Temperature of Atmosphere
  //sMach = vnorm / XT_SQRT( KCS * RSTAR * Ta)

  //step3
  //sAirPressure = rou * RSTAR * Ta
}

void update_CoordinateTransformationMatrix()
{
  // Calc CgI
  m_mlt(Ry(-A0),Rz(-B0),smCgI);
  m_mlt(smCgI,Rx(sDeltaTime * OMEGAe),smCgI);
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

  //step 0.1 TODO check formula
  Real mue0 = ALPHAe * XTSIN(2*B0) * ( 1- ALPHAe * ( 1- 4 * XTSIN(B0/2) * XTSIN(B0 / 2)));

  Real Faie0 = B0 - mue0 ;
  sFaie = Faie0;

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

  //step 7
  sMue = EP2 * XTSIN(2* sFaie ) * ( 1 - EP2 * XTSIN(sFaie) * XTSIN(sFaie)) / 2.0;

  //step 8.1
  sB = sFaie + sMue;
  //step 8.2
  sLambda = LAMBDA0 + XTARCTAN(svMovemente->ve[1]/svMovemente->ve[0]);

  //Initialize Coordinate Transformation Matrixes
  update_CoordinateTransformationMatrix(0.0);
}

void calc_CoordinatetransformationMatrix()
{
  //step 1
  sR = Ae * ( 1- ALPHAe) / XTSQRT( XTSIN(sFaie) * XTSIN(sFaie) + (1 - ALPHAe) * (1 - ALPHAe) * XTCOS(sFaie) * XTCOS(sFaie)) * H0;

  //step 2 TODO update from Posture
  // update svMovemente

  //step 4
  //TODO update svMovementI
  /* svr =  */ v_add(svR0,svMovementI,svr);
  sr = v_norm2(svr);

  //step 5
  Real rreciprocal = 1/ sr;
  /* svRnormal = */ sv_mlt(rreciprocal,svr,svRnormal);

  //step 6
  sFaie = XTARCSIN(in_prod(svRnormal,svOmegae));

  //step 7
  sMue = EP2 * XTSIN(2* sFaie ) * ( 1 - EP2 * XTSIN(sFaie) * XTSIN(sFaie)) / 2.0;

  //step 8.1
  sB = sFaie + sMue;
  //step 8.2
  sLambda = LAMBDA0 + XTARCTAN(svMovemente->ve[1]/svMovemente->ve[0]);

  //update coordinate transform matrixes
  update_CoordinateTransformationMatrix();
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
void update_Posture()
{
  sPsi =  XTARCSIN(-smCbI->me[2][0]);
  sFai = XTARCTAN2(smCbI->me[0][1],smCbI->me[0][0]); // -pi <= fai <= pi
  sGamma =  XTARCTAN2(smCbI->me[1][2],smCbI->me[2][2]);
}

#ifdef USE_BODYOMEGA
void init_OmegaBody()
{
  svOmegab = v_get(3);
}

VEC * update_OmegaBody(Real dtx,Real dty, Real dtz)
{
  svOmegab->ve[0] = dtx / XT_Tm;
  svOmegab->ve[1] = dty / XT_Tm;
  svOmegab->ve[2] = dtz / XT_Tm;
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

VEC * update_Quaternion(Real dtx,Real dty, Real dtz)
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
  update_OmegaBody(dtx,dty,dtz);
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
  sDeltaTime = 0;
  sGuideNormal = 0;
  sGuideLand = 0;
  init_Rxyz();

#ifdef USE_BODYOMEGA
  init_OmegaBody();
#endif

  init_Posture();
  init_CoordinateTransformMatrix();

  init_Quaternion();

  init_naviProc();

  init_Guidance();

}

//TODO verify input parameters and relations between dw dv and tm
void init_RoughAim(Real dvx,Real dvy,Real dvz,Real dwx,Real dwy,Real dwz)
{
  Real g = g0 * ( sR / sr);
  //TODO verify formula and unit
  Real t31,t32,t33,t21,t22,t23;
  t31 =  dvx / g / XT_Tm;
  smCnb->me[2][0] = t31;
  t32 = dvy / g / XT_Tm;
  smCnb->me[2][1] = t32;
  t33 = dvz / g / XT_Tm;
  smCnb->me[2][2] = t33;

  t21 = 1 / ( XT_Tm * OMEGAe * XTCOS(B0)) * ( dwx - XT_Tm * t31 * OMEGAe * XTSIN(B0));
  smCnb->me[1][0] = t21;
  t22 = 1 / ( XT_Tm * OMEGAe * XTCOS(B0)) * ( dwy - XT_Tm * t32 * OMEGAe * XTSIN(B0));
  smCnb->me[1][1] = t22;
  t23 = 1 / ( XT_Tm * OMEGAe * XTCOS(B0)) * ( dwz - XT_Tm * t33 * OMEGAe * XTSIN(B0));
  smCnb->me[1][2] = t23;

  smCnb->me[0][0] = t22 * t33 - t23 * t32;
  smCnb->me[0][1] = t23 * t31 - t21 * t33;
  smCnb->me[0][2] = t21 * t32 - t22 * t31;

}

void init_naviProc()
{
  sm33I = m_get(3,3);
  m_ident(sm33I);
  smrI3 = m_get(3,3);
  m_zero(smrI3);
  sv1 = v_get(3);
  v_ones(sv1);
  svk2k3 = v_get(3);

  svGn = v_get(3);
  svGn1 = v_get(3);

  svVI = v_get(3);
  v_zero(svVI);

  svVT = v_get(3);
  v_zero(svVT);

  svVI1 = v_get(3);
  svMovementI = v_get(3);
  v_zero(svMovementI);
  svMovementI1 = v_get(3);
}

//TODO deltaVn should be m/s not m/s2
void update_Navi(VEC * deltaVb)
{
  //step 1
  v_copy(svGn,svGn1);
  Real k1 = - fM / (sr * sr);
  Real sinFaie = XTSIN(sFaie);
  Real k2 = 1 - 5 * sinFaie * sinFaie;
  Real k3 = 2 * sinFaie;
  sv_mlt(k2,svRnormal,svk2k3);
  v_mltadd(svk2k3,svOmegae,k3,svk2k3);
  smrI3->me[0][0] = svRnormal->ve[0];
  smrI3->me[1][1] = svRnormal->ve[1];
  smrI3->me[2][2] = svRnormal->ve[2];
  //vm_mlt(smrI3,svk2k3,svk2k3);
  mv_mltadd(sv1,svk2k3,smrI3,XT_J,svk2k3);
  vm_mlt(smrI3,svk2k3,svk2k3);
  sv_mlt(k1,svk2k3,svGn);

  //step 2
  //TODO first update_Quaternion
  mv_mlt(smCIb,deltaVb,deltaVb);
  VEC * deltaVI = deltaVb;

  //step 3
  v_copy(svVI,svVI1);
  v_add(svGn,svGn1,svVI);
  sv_mlt(XT_Tm / 2,svVI,svVI);
  v_add(svVI,svVI1,svVI);
  v_add(svVI,deltaVI,svVI);

  //step 4
  v_copy(svMovementI,svMovementI1);
  v_mltadd(deltaVI,svGn1,XT_Tm,svMovementI);
  v_mltadd(svVI1,svGn,0.5,svMovementI);
  v_mltadd(svMovementI1,svMovementI,XT_Tm,svMovementI);

  //step 5
  mv_mlt(smCgI,svVI,svVT);
  vm_mlt(smCgE,svVT,svVT);
  vm_mlt(smCTE,svVT,svVT);

  //step 6
  sH = sr - sR + H0;

  //step 7
  Real * vT = svVT->ve;
  sThetaT = XTARCSIN(vT[1]/ XTSQRT(vT[0] * vT[0] + vT[1] * vT[1] + vT[2] * vT[2]));
}
