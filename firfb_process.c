// firfb_process.c - FIRFB-processing functions

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "chapro.h"
#include "cha_ff.h"

/***********************************************************/

// FIR-filterbank analysis for short chunk (cs < nw)
static __inline void
firfb_analyze_sc(float *x, float *y, int cs,
    float *hh, float *xx, float *yy, float *zz, int nc, int nw)
{
    float   *hk, *yk, *zk;
    int      i, ir, ii, j, k, kk, nf, ns, nt, nk;

    nk = nw / cs;
    nt = cs * 2;
    nf = cs + 1;
    ns = nf * 2;
    // loop over channels
    for (k = 0; k < nc; k++) {
        fzero(xx, nt);
        fcopy(xx, x, cs);
        cha_fft_rc(xx, nt);
        // loop over sub-window segments
        for (j = 0; j < nk; j++) {
            kk = j + k * nk;
            hk = hh + kk * ns;
            for (i = 0; i < nf; i++) {
                ir = i * 2;
                ii = i * 2 + 1;
                yy[ir] = xx[ir] * hk[ir] - xx[ii] * hk[ii];
                yy[ii] = xx[ir] * hk[ii] + xx[ii] * hk[ir];
            }
            cha_fft_cr(yy, nt);
            yk = y + k * cs;
            zk = zz + k * nw;
            for (i = 0; i < nw; i++) {
                if (i < cs) {
                    yk[i] = yy[i] + zk[i];
                }
                if ((i + cs) < nw) {
                    zk[i] = yy[i + cs] + zk[i + cs];
                } else {
                    zk[i] = yy[i + cs];
                }
            }
        }
    }
}

// FIR-filterbank analysis for long chunk (cs >= nw)
static __inline void
firfb_analyze_lc(float *x, float *y, int cs,
    float *hh, float *xx, float *yy, float *zz, int nc, int nw)
{
    float   *hk, *yk, *zk;
    int      i, ir, ii, j, k, nf, nt, ni;

    nt = nw * 2;
    nf = nw + 1;
    // loop over sub-chunk segments
    for (j = 0; j < cs; j += nw) {
        ni = ((cs - j) < nw) ? (cs - j) : nw;
        fzero(xx, nt);
        fcopy(xx, x + j, ni);
        cha_fft_rc(xx, nt);
       // loop over channels
        for (k = 0; k < nc; k++) {
            hk = hh + k * nf * 2;
            for (i = 0; i < nf; i++) {
                ir = i * 2;
                ii = i * 2 + 1;
                yy[ir] = xx[ir] * hk[ir] - xx[ii] * hk[ii];
                yy[ii] = xx[ir] * hk[ii] + xx[ii] * hk[ir];
            }
            cha_fft_cr(yy, nt);
            yk = y + j + k * cs;
            zk = zz + k * nw;
            for (i = 0; i < nw; i++) {
                if (i < ni) {
                    yk[i] = yy[i] + zk[i];
                }
                zk[i] = yy[i + ni];
            }
        }
    }
}

// FIR-filterbank analysis
FUNC(void)
cha_firfb_analyze(CHA_PTR cp, float *x, float *y, int cs)
{
    float   *hh, *xx, *yy, *zz;
    int      nc, nw;

    nc = CHA_IVAR[_nc];
    nw = CHA_IVAR[_nw];
    hh = (float *) cp[_ffhh];
    xx = (float *) cp[_ffxx];
    yy = (float *) cp[_ffyy];
    zz = (float *) cp[_ffzz];
    if (cs < nw) {
        firfb_analyze_sc(x, y, cs, hh, xx, yy, zz, nc, nw);
    } else {
        firfb_analyze_lc(x, y, cs, hh, xx, yy, zz, nc, nw);
    }
}

// FIR-filterbank synthesis
FUNC(void)
cha_firfb_synthesize(CHA_PTR cp, float *x, float *y, int cs)
{
    float    xsum;
    int      i, k, nc;

    nc = CHA_IVAR[_nc];
    for (i = 0; i < cs; i++) {
        xsum = 0;
        for (k = 0; k < nc; k++) {
            xsum += x[i + k * cs];
        }
        y[i] = xsum;
    }
}
