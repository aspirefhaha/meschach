#pragma once
#include "machine.h"

typedef struct _naviguide_t {
	int dtime;
	Real k1fai;
	Real k2fai;
	Real k1psi;
	Real k2psi;
} naviguide_t;
extern naviguide_t naviguide[];
extern int naviguidecount;
