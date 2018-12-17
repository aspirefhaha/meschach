#pragma once

typedef struct _trj_Hcx_Time_st {
  double dtime;
  double height;
}trj_Hcx_Time_st;

extern int trj_hcx_time_count ;

extern trj_Hcx_Time_st Trj_hcx_time[];

typedef struct _trj_Thetacx_Time_st {
  double dtime;
  double thetacx;
}trj_Thetacx_Time_st;

extern int trj_thetacx_time_count;

extern trj_Thetacx_Time_st Trj_thetacx_time[];

typedef struct _trj_Thetacx_Height_st {
  double height;
  double thetacx;
}trj_Thetacx_Height_st;

extern trj_Thetacx_Height_st Trj_thetacx_height[];

extern int trj_thetacx_height_count;
