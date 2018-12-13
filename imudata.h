#pragma once

typedef struct _imudata_t {
  double dtime;
  double dwx;
  double dwy;
  double dwz;
  double dvx;
  double dvy;
  double dvz;
}imudata_t;

extern imudata_t imudata[];
