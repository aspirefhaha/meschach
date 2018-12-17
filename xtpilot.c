#include <stdio.h>
#include "matrix2.h"
#include "xtpilot.h"
#include "naviguide.h"
#include "trajectory.h"
#include "ctlgain.h"


static Real sDeltaTime;
static Real sUpTime;
static int sIsSplited;  //Tfl related

// posture calc matrixes and vectors
static MAT * smRx ;  //Rotate Matrix for x
static MAT * smRy ;  //Rotate Matrix for y
static MAT * smRz ;  //Rotate Matrix for z
static VEC * svQn ;  //
static MAT * smQMCalc ;   //Quaternion Matrix for update

// matrixes and vectors for coordinate-transformation
static VEC * svOmegae;
static VEC * svOmegaI;
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
static VEC * svMovenmentg;

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
static Real sFaiI;
static Real sPsiI;
static Real sGammaI;

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

static VEC * svA0;
static VEC * svA1;
static VEC * svDeltaPostureI;

static VEC * svDelta1K;
static VEC * svCSMid;     //Control Signal Calc Mid Vector

static Real sRudder1;
static Real sRudder2;
static Real sRudder3;
static Real sRudder4;

static Real sbB0;
static Real sbAlpha11,sbAlpha12,sbAlpha21,sbAlpha22,sbAlpha31,sbAlpha32;
static Real sbBeta11,sbBeta12,sbBeta21,sbBeta22,sbBeta31,sbBeta32;



//TODO
//bind rudder control three parameters
void bind_sbB0AlphaBeta()
{

}

//TODO
Real get_beta()
{
  //TODO
  return 0;
}

void calc_ControlSignal()
{
  //step 1
  v_star(svA0,svDeltaPostureI,svDelta1K);
  vm_mlt(smCIb,svOmegab,svOmegaI);
  v_star(svA1,svOmegaI,svCSMid);
  v_add(svDelta1K,svCSMid,svDelta1K);

  //step 2
  static Real phiink1 = 0,phiink2 = 0,phiout1k1 = 0,phiout1k2=0,phiout2k1=0,phiout2k2=0;
  static Real phioutk1 = 0, phioutk2 = 0;
  static Real psiink1 = 0,psiink2 = 0,psiout1k1 = 0,psiout1k2=0,psiout2k1=0,psiout2k2=0;
  static Real psioutk1 = 0, psioutk2 = 0;
  Real phiink =svDelta1K->ve[0];
  Real phiout1k = sbB0 * ( phiink + sbBeta11* phiink1 + sbBeta12 * phiink2)
                  - sbAlpha11 * phiout1k1 - sbAlpha12 * phiout1k2;
  Real phiout2k = phiout1k + sbBeta21 * phiout1k1 + sbBeta22 * phiout1k2
                  - sbAlpha21 * phiout2k1 - sbAlpha22*phiout2k2;
  Real phioutk = phiout2k + sbBeta31 * phiout2k1 + sbBeta32 * phiout2k2
                  - sbAlpha31 * phioutk1 - sbAlpha32 * phioutk2;
  phiout1k2 = phiout1k1;
  phiout1k1 = phiout1k;

  phiout2k2 = phiout2k1;
  phiout2k1 = phiout2k;

  phiink2 = phiink1;
  phiink1 = phiink;

  phioutk2 = phioutk1;
  phioutk1 = phioutk;

  Real psiink = svDelta1K->ve[1];
  if(sH > 34000 && sH < 35000){
    svDelta1K->ve[1] + svA0->ve[1] * get_beta();
  }
  else if(sH <=34000 && sH > 33000){
    svDelta1K->ve[1] + svA0->ve[1] * get_beta() * (sH - 33000)/ 1000;
  }

  Real psiout1k = sbB0 * (psiink + sbBeta11 * psiink1 + sbBeta12 * psiink2)
                  - sbAlpha11 * psiout1k1 - sbAlpha12 * psiout1k2;
  Real psiout2k = psiout1k + sbBeta21 * psiout1k1 + sbBeta22 * psiout1k2
                  - sbAlpha21 * psiout2k1 - sbAlpha22 * psiout2k2;
  Real psioutk = psiout2k + sbBeta31 * psiout2k1 + sbBeta32 * psiout2k2
                  - sbAlpha31 * psioutk1 - sbAlpha32 * psioutk2;

  psiout1k2 = psiout1k1;
  psiout1k1 = psiout1k;

  psiout2k2 = psiout2k1;
  psiout2k1 = psiout2k;

  psioutk2 = psioutk1;
  psioutk1 = psioutk;

  psiink2 = psiink1;
  psiink1 = psiink;

  Real gammaoutk = svDelta1K->ve[2];

  //step 3
  if( XTABS(phioutk) >= RCDELTAFAIM ){
    phioutk = SIGNFUNC(phioutk) * RCDELTAFAIM;
  }
  if(XTABS(psioutk) >= RCDELTAPSIM){
    psioutk = SIGNFUNC(psioutk) * RCDELTAPSIM;
  }
  if(XTABS(gammaoutk) >= RCDELTAGAMMAM){
    gammaoutk = SIGNFUNC(gammaoutk) * RCDELTAGAMMAM;
  }

  //step 4
  sRudder1 = - phioutk - psioutk + gammaoutk;
  sRudder2 = - phioutk + psioutk + gammaoutk;
  sRudder3 = phioutk + psioutk + gammaoutk;
  sRudder4 = phioutk - psioutk + gammaoutk;

  //step 5
  if(XTABS(sRudder1) >= RCDELTAM){
    sRudder1 = SIGNFUNC(sRudder1) * RCDELTAM;
  }
  if(XTABS(sRudder2) >= RCDELTAM){
    sRudder2 = SIGNFUNC(sRudder2) * RCDELTAM;
  }
  if(XTABS(sRudder3) >= RCDELTAM){
    sRudder3 = SIGNFUNC(sRudder3) * RCDELTAM;
  }
  if(XTABS(sRudder4) >= RCDELTAM){
    sRudder4 = SIGNFUNC(sRudder4) * RCDELTAM;
  }
}

void init_postureControl()
{
  svA0 = v_get(3);
  v_zero(svA0);
  svA1 = v_get(3);
  v_zero(svA1);
  svDeltaPostureI = v_get(3);
  v_zero(svDeltaPostureI);

  svDelta1K = v_get(3);
  v_zero(svDelta1K);
  svCSMid = v_get(3);
  v_zero(svCSMid);
}

Real get_faic()
{
  int rindex = 0;
  if(sIsSplited){
    rindex = trj_faic_thetacx_height_count;
    while(--rindex>=0){
      if(Trj_faic_thetacx_height[rindex].height <= sH){
        return LINEAR_INTER(Trj_faic_thetacx_height,rindex,height,faic,sH);
      }
    }
  }
  else{
    rindex = trj_faic_thetacx_time_count;
    while(--rindex>=0){
      if(Trj_faic_thetacx_time[rindex].dtime <= sDeltaTime){
        return LINEAR_INTER(Trj_faic_thetacx_time,rindex,dtime,faic,sDeltaTime);
      }
    }
  }
}

void calc_deltaPosture()
{
  Real * dPos = svDeltaPostureI->ve;
  Real * a0 = svA0->ve;

  //TODO verify below 2 values are 0
  Real psic = 0;
  Real gammac = 0;
  dPos[0] = sFaiI - get_faic() + sGuideNormal / a0[0];
  dPos[1] = sPsiI - psic + sGuideLand / a0[1];
  dPos[2] = sGammaI - gammac;
}

int get_A01index()
{
  int rindex = ctlgain_count;
  while(--rindex>=0){
    if(ctlgain[rindex].dtime <= sDeltaTime){
      return rindex;
    }
  }
  return 0;
}

void calc_A01faipsi()
{
  Real * a0 = svA0->ve;
  Real * a1 = svA1->ve;
  int rindex = get_A01index();
  if(sIsSplited){
    a0[0] = LINEAR_INTER(ctlgain,rindex,dtime,a0,sDeltaTime);
    a0[1] = a0[0];//LINEAR_INTER(ctlgain,rindex,dtime,a0,sDeltaTime);


    a1[0] = LINEAR_INTER(ctlgain,rindex,dtime,a1,sDeltaTime);
    a1[1] = a1[0];//LINEAR_INTER(ctlgain,rindex,dtime,a1,sDeltaTime);
  }
  else{
    a0[0]= 15;
    a0[1] = 15;

    a1[0] = 2.5;
    a1[1] = 2.5;

  }
  a0[2] = LINEAR_INTER(ctlgain,rindex,dtime,a0gamma,sDeltaTime);
  a1[2] = LINEAR_INTER(ctlgain,rindex,dtime,a1gamma,sDeltaTime);
}

void init_Guidance()
{
  svVg = v_get(3);
  v_zero(svVg);
  svVb = v_get(3);
  v_zero(svVb);
}

void calc_GuidanceParameters()
{
  //Step 1
  mv_mlt(smCgI,svVI,svVg);
  Real * vg = svVg->ve;
  //Step 2
  mv_mlt(smCbI,svVI,svVb);
  Real * vb = svVb->ve;
  Real tVg = v_norm2(svVg);
  Real tVb = v_norm2(svVb);
  //Step 3
  if(vg[0]>=0){ //vgx >= 0
    sTheta = XTARCSIN(vg[1]/XTSQRT(vg[0] * vg[0] + vg[1] * vg[1]));
  }
  else{ //vgx<0
    sTheta = XT_PI - XTARCSIN(vg[1]/XTSQRT(vg[0] * vg[0] + vg[1] * vg[1]));
  }
  sEpsilon = XTARCSIN( - vg[2] / tVg);
  //Step 4
  if(vb[0]>=0){ //vbx >= 0
    sAlpha = XTARCSIN(vb[1] / XTSQRT(vb[0] * vb[0] + vb[1] * vb[1]));
    sBeta = XTARCSIN( vb[1] / tVb);
  }
  else {  //vbx < 0
    sAlpha = SIGNFUNC(vb[1]) * XT_PI - XTARCSIN( vb[1] / XTSQRT(vb[0] * vb[0] + vb[1] * vb[1]));
    sBeta = XT_PI * SIGNFUNC(vb[2]) - XTARCSIN(vb[2] / tVb);
  }
}



//TODO bind trajectory
Real get_trajectoryTheta()
{
  int rindex = 0;
  if(sIsSplited){
    rindex = trj_faic_thetacx_height_count;
    while(--rindex>=0){
      if(Trj_faic_thetacx_height[rindex].height <= sH){
        return LINEAR_INTER(Trj_faic_thetacx_height,rindex,height,thetacx,sH);
      }
    }
  }
  else{
    rindex = trj_faic_thetacx_time_count;
    while(--rindex>=0){
      if(Trj_faic_thetacx_time[rindex].dtime <= sDeltaTime){
        return LINEAR_INTER(Trj_faic_thetacx_time,rindex,dtime,thetacx,sDeltaTime);
      }
    }
  }
  return 0;
}

//TODO bind trajectory
Real get_trjactoryHeight()
{
  int rindex = trj_hcx_time_count;
  while(--rindex>=0 ){
    if( Trj_hcx_time[rindex].dtime <= sDeltaTime ){
      return LINEAR_INTER(Trj_hcx_time,rindex,dtime,height,sDeltaTime);
    }
  }
  return -1;
}

static int get_k12faipsiindex()
{
  int rindex = naviguidecount;
  while(--rindex>=0){
    if(naviguide[rindex].dtime<=sDeltaTime){
      return rindex;
    }
  }
}

void calc_GuidanceValues()
{
  Real deltaTheta = get_trajectoryTheta() - sTheta;
  Real deltaH = get_trjactoryHeight() - sH;
  int naviindex = get_k12faipsiindex();
  Real k1fai = LINEAR_INTER(naviguide,naviindex,dtime,k1fai,sDeltaTime);
  Real k2fai = LINEAR_INTER(naviguide,naviindex,dtime,k2fai,sDeltaTime);
  Real k1psi = LINEAR_INTER(naviguide,naviindex,dtime,k1psi,sDeltaTime);
  Real k2psi = LINEAR_INTER(naviguide,naviindex,dtime,k2psi,sDeltaTime);

  //TODO fhaha add not in formula
  mv_mlt(smCgI,svMovementI,svMovenmentg);
  Real z = svMovenmentg->ve[2];

  if(sDeltaTime <= 6){
    sGuideNormal = 0;
    sGuideLand = 0;
  }
  else if(sDeltaTime <= 8){
    sGuideNormal = (sDeltaTime - 6) / 2.0 * ( k1fai * deltaTheta + k2fai * deltaH );
    sGuideLand = (sDeltaTime - 6) / 2.0 * (k1psi * sEpsilon + k2psi * z);
  }
  else {
    if(sH <= 29500 /* TODO && flying up */){
      sGuideNormal = k1fai * deltaTheta + k2fai * deltaH;
      sGuideLand = k1psi * sEpsilon * k2psi * z;
    }
    else if(sH <= 30000 /* TODO && flying up */ ){
      sGuideNormal = (3000 - sH)/500 * (k1fai * deltaTheta + k2fai * deltaH);
      sGuideLand = (30000 - sH)/500 * (k1psi * sEpsilon + k2psi * z);
    }
    else if( sH <= 36000 /* TODO && flying up */ ){
      sGuideNormal = sGuideLand = 0;
    }
    else if( sH < 36500 /* TODO && flying up */ ){
      sGuideNormal = ( sH - 36000) / 500 * (k1fai * deltaTheta + k2fai * deltaH);
      sGuideLand = (sH - 36000) / 500 * (k1psi * sEpsilon + k2psi * z);
    }
    else{ ///* TODO && flying up  and sH > 36500*/ ){
      sGuideNormal = k1fai * deltaTheta + k2fai * deltaH;
      sGuideLand = k1psi * sEpsilon + k2psi * z;
    }
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
  m_mlt(Rx(sGammaI),Ry(sPsiI),smCbI);
  m_mlt(smCbI,Rz(sFaiI),smCbI);

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

  svOmegaI = v_get(3);
  //TODO verify omega I no need to init

  svr = v_get(3);

  svR0 = v_get(3);

  svRnormal = v_get(3);

  svMovementI = v_get(3);
  v_zero(svMovementI);

  svMovenmentg = v_get(3);
  v_zero(svMovenmentg);

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
  sFaiI = XT_PI /2;
  sPsiI = 0;
  sGammaI = 2 * XT_PI / 180;
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
  sPsiI =  XTARCSIN(-smCbI->me[2][0]);
  sFaiI = XTARCTAN2(smCbI->me[0][1],smCbI->me[0][0]); // -pi <= fai <= pi
  sGammaI =  XTARCTAN2(smCbI->me[1][2],smCbI->me[2][2]);
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
  Real cosg2 = XTCOS(sGammaI/2);
  Real cosp2 = XTCOS(sPsiI/2);
  Real cosf2 = XTCOS(sFaiI/2);
  Real sing2 = XTSIN(sGammaI/2);
  Real sinp2 = XTSIN(sPsiI/2);
  Real sinf2 = XTSIN(sFaiI/2);
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
  sIsSplited = 0;
  sUpTime = 0;
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

  init_postureControl();

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
