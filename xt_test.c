#include <stdio.h>
#include "xtpilot.h"

void tk4(VEC * x)
{
  static VEC * v1 = VNULL, * v2 = VNULL, * v3 = VNULL, *v4 = VNULL, *temp = VNULL;

  v1 = v_resize(v1,x->dim);
  MEM_STAT_REG(v1,TYPE_VEC);

  v2 = v_resize(v2, x->dim+1);
  MEM_STAT_REG(v2,TYPE_VEC);

  v_output(v1);

  v_output(v2);

  v_output(v3);

  v_output(v4);

  v_output(temp);


  // V_FREE(v1);
  // V_FREE(v2);
  // V_FREE(v3);
}

void k()
{
  MAT * A, * QR;
  VEC *b,*x,*diag;
  printf(" Input A matrix\n");
  A = m_input(MNULL);
  if( A->m < A->n){
    printf("need m > n to obtain least squares fit\n");
    return;
  }
  printf("# A = ");
  m_output(A);
  diag = v_get(A->m);
  QR = m_copy(A,MNULL);
  QRfactor(QR,diag);
  printf("diag:");
  v_output(diag);
  printf("QR:");
  m_output(QR);
  printf("input b vector:\n");
  b = v_get(A->m);
  b = v_input(b);
  printf("b:");
  v_output(b);
  x = QRsolve(QR,diag,b,VNULL);
  printf("Vector of best fit parameters is:");
  v_output(x);
  printf(" || Ax-b || = %g\n", v_norm2(v_sub(mv_mlt(A,x,VNULL),b,VNULL)));
}

void j()
{
  VEC k;
  v_free(&k);
}

void test()
{
  MAT * A;
  VEC * x;
  PERM * p;
  A = m_get(3,4);
  x = v_get(10);
  p = px_get(10);
  printf(" A %d * %d\n",A->m,A->n);
  m_output(A);
  v_output(x);
  px_output(p);
  printf("Hello XinTu Pilot\n");

  mem_stat_mark(2);
  tk4(x);
  mem_stat_free(2);

  M_FREE(A);
  V_FREE(x);
  PX_FREE(p);
  //k();
  //j();
}

int main(int argc,char ** argv)
{
  //test();
  FILE * fp = fopen("./rx.dat","w");
  init_Rxyz();
  MAT * rx = Rx(M_PI / 3);
  m_output(rx);
  char pname[]="good";
  m_save(fp,rx,&pname);
  fclose(fp);
  VEC * tv = v_get(3);
  tv->ve[0]=1;
  tv->ve[1]=2;
  tv->ve[2]=3;
  Real nor = v_norm2(tv);
  v_output(tv);
  printf("norm result:%lf\n",nor);
  return 0;
}
