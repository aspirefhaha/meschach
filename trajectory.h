#pragma once
#include "machine.h"

typedef struct _trj_Hcx_Time_st {
  Real dtime;
  Real height;
}trj_Hcx_Time_st;

extern int trj_hcx_time_count ;

extern trj_Hcx_Time_st Trj_hcx_time[];

typedef struct _trj_Faic_Thetacx_Time_st {
  Real dtime;
  Real faic;
  Real thetacx;
  Real reserved;
}trj_Faic_Thetacx_Time_st;

extern int trj_faic_thetacx_time_count;

extern trj_Faic_Thetacx_Time_st Trj_faic_thetacx_time[];

typedef struct _trj_Faic_Thetacx_Height_st {
  Real height;
  Real faic;
  Real thetacx;
  Real reserved;
}trj_Faic_Thetacx_Height_st;

extern trj_Faic_Thetacx_Height_st Trj_faic_thetacx_height[];

extern int trj_faic_thetacx_height_count;
