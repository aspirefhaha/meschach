#pragma once

typedef struct _ctlgain_t{
  double dtime;
  double a0;
  double a1;
  double a0gamma;
  double a1gamma;
}ctlgain_t;

extern ctlgain_t ctlgain[];
