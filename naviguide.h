#pragma once

typedef struct _naviguide_t {
	int dtime;
	double k1fai;
	double k2fai;
	double k1psi;
	double k2psi;
} naviguide_t;
extern naviguide_t naviguide[];
extern int naviguidecount;
