// agc_process.c - FIR-filterbank processing functions

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "chapro.h"
#include "cha_ff.h"

// Choose your math for going into and out of dB space
//#define db2(x)          ((20/logf(10))*logf(x))
//#define undb2(x)        expf((x)/(20/logf(10)))
//#define db2(x)          cha_db2(x)
//#define undb2(x)        cha_undb2(x)
#define undb2(x)    (expf(0.1151292546497023f*x))  //faster:  exp(log(10.0f)*(x/20));  this is exact
#define db2(x)      ((6.020599913279624f)*log2f_approx(x)) //faster: 20*log2_approx(x)/log2(10);  this is approximate

/* ----------------------------------------------------------------------
** Fast approximation to the log2() function.  It uses a two step
** process.  First, it decomposes the floating-point number into
** a fractional component F and an exponent E.  The fraction component
** is used in a polynomial approximation and then the exponent added
** to the result.  A 3rd order polynomial is used and the result
** when computing db20() is accurate to 7.984884e-003 dB.
** ------------------------------------------------------------------- */
//https://community.arm.com/tools/f/discussions/4292/cmsis-dsp-new-functionality-proposal/22621#22621
//float log2f_approx_coeff[4] = {1.23149591368684f, -4.11852516267426f, 6.02197014179219f, -3.13396450166353f};
static float log2f_approx(float X) {
  //float *C = &log2f_approx_coeff[0];
  float Y;
  float F;
  int E;

  // This is the approximation to log2()
  F = frexpf(fabsf(X), &E);
  //  Y = C[0]*F*F*F + C[1]*F*F + C[2]*F + C[3] + E;
  //Y = *C++;
  Y = 1.23149591368684f;
  Y *= F;
  //Y += (*C++);
  Y += -4.11852516267426f;
  Y *= F;
  //Y += (*C++);
  Y += 6.02197014179219f;
  Y *= F;
  //Y += (*C++);
  Y += -3.13396450166353f;
  Y += E;

  return(Y);
}

/***********************************************************/

static __inline void
smooth_env(float *x, float *y, int n, float *ppk, float alfa, float beta)
{
    float  xab, xpk;
    int k;

    // find envelope of x and return as y
    xpk = *ppk;                     // start with previous xpk
    for (k = 0; k < n; k++) {
	xab = (x[k] >= 0) ? x[k] : -x[k];
	if (xab >= xpk) {
            xpk = alfa * xpk + (1-alfa) * xab;
        } else {
            xpk = beta * xpk;
        }
	y[k] = xpk;
    }
    *ppk = xpk;                     // save xpk for next time
}

static __inline void
WDRC_circuit(float *x, float *y, float *pdb, int n, 
     float tkgn, float tk, float cr, float bolt)
{
    float gdb, tkgo, pblt;
    int k;

    if ((tk + tkgn) > bolt) {
        tk = bolt - tkgn;
    }
    tkgo = tkgn + tk * (1 - 1 / cr);
    pblt = cr * (bolt - tkgo);
    for (k = 0; k < n; k++) {
        if ((pdb[k] < tk) && (cr >= 1)) {
            gdb = tkgn;
        } else if (pdb[k] > pblt) {
            gdb = bolt + ((pdb[k] - pblt) / 10) - pdb[k];
        } else {
            gdb = ((1 / cr) - 1) * pdb[k] + tkgo;
        }
        y[k] = x[k] * undb2(gdb); 
    }
}

static __inline void
compress(CHA_PTR cp, float *x, float *y, int n, float *ppk,
    float alfa, float beta, float tkgn, float tk, float cr, float bolt)
{
    float mxdb, *xpk;
    int k;

    // find smoothed envelope
    xpk = (float *) cp[_xpk];
    smooth_env(x, xpk, n, ppk, alfa, beta);
    // convert envelope to dB
    mxdb = (float) CHA_DVAR[_mxdb];
    for (k = 0; k < n; k++) {
        xpk[k] = mxdb + db2(xpk[k]);
    }
    // apply wide-dynamic range compression
    WDRC_circuit(x, y, xpk, n, tkgn, tk, cr, bolt);
}


/***********************************************************/

FUNC(void)
cha_agc_input(CHA_PTR cp, float *x, float *y, int cs)
{
    float alfa, beta, tkgn, tk, cr, bolt, *ppk;

    // initialize WDRC variables
    alfa = (float) CHA_DVAR[_alfa];
    beta = (float) CHA_DVAR[_beta];
    tkgn = (float) CHA_DVAR[_tkgn];
    tk = (float) CHA_DVAR[_tk];
    cr = (float) CHA_DVAR[_cr];
    bolt = (float) CHA_DVAR[_bolt];
    ppk = (float *) cp[_ppk];  // first ppk for input
    compress(cp, x, y, cs, ppk, alfa, beta, tkgn, tk, cr, bolt);
}

FUNC(void)
cha_agc_channel(CHA_PTR cp, float *x, float *y, int cs)
{
    float alfa, beta, *tkgn, *tk, *cr, *bolt, *ppk;
    float *xk, *yk, *pk;
    int k, nc;

    // initialize WDRC variables
    alfa = (float) CHA_DVAR[_gcalfa];
    beta = (float) CHA_DVAR[_gcbeta];
    tkgn = (float *) cp[_gctkgn];
    tk = (float *) cp[_gctk];
    cr = (float *) cp[_gccr];
    bolt = (float *) cp[_gcbolt];
    ppk = (float *) cp[_gcppk];
    // loop over channels
    nc = CHA_IVAR[_nc];
    for (k = 0; k < nc; k++) {
        xk = x + k * cs;
        yk = y + k * cs;
        pk = ppk + k;
        compress(cp, xk, yk, cs, pk, alfa, beta, tkgn[k], tk[k], cr[k], bolt[k]);
    }
}

FUNC(void)
cha_agc_output(CHA_PTR cp, float *x, float *y, int cs)
{
    float alfa, beta, tkgn, tk, cr, bolt, *ppk;

    // initialize WDRC variables
    alfa = (float) CHA_DVAR[_alfa];
    beta = (float) CHA_DVAR[_beta];
    tkgn = (float) CHA_DVAR[_tkgn];
    tk = (float) CHA_DVAR[_tk];
    cr = (float) CHA_DVAR[_cr];
    bolt = (float) CHA_DVAR[_bolt];
    ppk = (float *) cp[_ppk] + 1;   // second ppk for output
    compress(cp, x, y, cs, ppk, alfa, beta, tkgn, tk, cr, bolt);
}
