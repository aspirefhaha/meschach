#pragma once
#include "machine.h"

typedef struct _ctlgain_t{
  Real dtime;
  Real a0;
  Real a1;
  Real a0gamma;
  Real a1gamma;
}ctlgain_t;

extern ctlgain_t ctlgain[];

extern int ctlgain_count;
