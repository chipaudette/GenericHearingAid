// db.c - convert magnitudes to and from decibels
#include <stdlib.h>
#include <string.h>
#include <math.h>
//include <assert.h>
#include "chapro.h"
#include "cha_ff.h"


// METHOD: 0=exact, 1=polynomial_ratio, 2=lookup_table
#define METHOD 2

/***********************************************************/

#if METHOD == 1

static __inline float
pow_pr(float x) // approximate 2^x
{
    float p, q;
    int   n;
    static float a0 = -206059.514;
    static float a1 = -72102.2578;
    static float a2 = -11240.0288;
    static float a3 = -989.027847;
    static float b0 = -206059.514;
    static float b1 = 70727.3131;
    static float b2 = -10763.5093;
    static float b3 = -14.5169712;

    // assume: x > -0.5 and x < 0.5
    n = (x < 0);
    if (n) x = -x;
    p = (((a3 * x + a2) * x + a1) * x + a0);
    q = (((b3 * x + b2) * x + b1) * x + b0);
    if (n) return (q / p);
    return (p / q);
}

static __inline float
log_pr(float x) // approximate ln(x)
{
    float p, q, z, z2;
    static float a0 = 75.1518561;
    static float a1 = -134.730400;
    static float a2 = 74.2011014;
    static float b0 = 37.5759281;
    static float b1 = -79.8905092;
    static float b2 = 56.2155348;

    // assume: x > sqrt(1/2) and x < sqrt(2)
    z = (x - 1) / (x + 1);
    z2 = z * z;
    p = ((a2 * z2 + a1) * z2 + a0);
    q = ((b2 * z2 + b1) * z2 + b0);
    return (z * p / q);
}

#elif METHOD == 2

static __inline float
pow_lu(float x) // approximate 2^x
{
    float f, y;
    int i;
    static float xmn = -0.500000000;
    static float odx  = 1.270000000e+02;
    static float lu[] = { // pow
      7.071067691e-01,  7.109766006e-01,  7.148676515e-01,  7.187799215e-01,
      7.227136493e-01,  7.266688943e-01,  7.306457758e-01,  7.346444726e-01,
      7.386649847e-01,  7.427075505e-01,  7.467722297e-01,  7.508591413e-01,
      7.549684048e-01,  7.591001987e-01,  7.632545829e-01,  7.674316764e-01,
      7.716316581e-01,  7.758546472e-01,  7.801007032e-01,  7.843700051e-01,
      7.886626720e-01,  7.929788828e-01,  7.973186970e-01,  8.016822338e-01,
      8.060696125e-01,  8.104810715e-01,  8.149166703e-01,  8.193765283e-01,
      8.238607645e-01,  8.283695579e-01,  8.329030275e-01,  8.374613523e-01,
      8.420445919e-01,  8.466528654e-01,  8.512864113e-01,  8.559452891e-01,
      8.606297374e-01,  8.653397560e-01,  8.700755835e-01,  8.748372793e-01,
      8.796250820e-01,  8.844390512e-01,  8.892793655e-01,  8.941462040e-01,
      8.990396857e-01,  9.039599299e-01,  9.089071155e-01,  9.138813019e-01,
      9.188827872e-01,  9.239116311e-01,  9.289679527e-01,  9.340519905e-01,
      9.391638637e-01,  9.443036914e-01,  9.494716525e-01,  9.546679258e-01,
      9.598925710e-01,  9.651458859e-01,  9.704278708e-01,  9.757388234e-01,
      9.810788035e-01,  9.864480495e-01,  9.918466210e-01,  9.972748160e-01,
      1.002732635e+00,  1.008220434e+00,  1.013738155e+00,  1.019286036e+00,
      1.024864435e+00,  1.030473232e+00,  1.036112785e+00,  1.041783214e+00,
      1.047484636e+00,  1.053217292e+00,  1.058981299e+00,  1.064776897e+00,
      1.070604205e+00,  1.076463342e+00,  1.082354546e+00,  1.088278055e+00,
      1.094233990e+00,  1.100222468e+00,  1.106243730e+00,  1.112297893e+00,
      1.118385315e+00,  1.124505997e+00,  1.130660176e+00,  1.136847973e+00,
      1.143069744e+00,  1.149325490e+00,  1.155615449e+00,  1.161939859e+00,
      1.168298960e+00,  1.174692750e+00,  1.181121588e+00,  1.187585592e+00,
      1.194085002e+00,  1.200619936e+00,  1.207190633e+00,  1.213797331e+00,
      1.220440149e+00,  1.227119327e+00,  1.233835101e+00,  1.240587592e+00,
      1.247377038e+00,  1.254203677e+00,  1.261067629e+00,  1.267969251e+00,
      1.274908543e+00,  1.281885743e+00,  1.288901210e+00,  1.295955062e+00,
      1.303047538e+00,  1.310178876e+00,  1.317349195e+00,  1.324558735e+00,
      1.331807733e+00,  1.339096427e+00,  1.346424937e+00,  1.353793621e+00,
      1.361202717e+00,  1.368652225e+00,  1.376142621e+00,  1.383673906e+00,
      1.391246438e+00,  1.398860335e+00,  1.406516075e+00,  1.414213538e+00};
    y =  (x - xmn) * odx;
    i = (int) y;
    f = y - i;
    if (f == 0) return (lu[i]);
    return ((1 - f) * lu[i] + f * lu[i + 1]);
}

static __inline float
log_lu(float x) // approximate ln(x)
{
    float f, y;
    int i;
    static float xmn =  0.707106769;
    static float odx  = 1.796051178e+02;
    static float lu[] = { // log
     -3.465736210e-01, -3.387303948e-01, -3.309483230e-01, -3.232262433e-01,
     -3.155633509e-01, -3.079588115e-01, -3.004115820e-01, -2.929208875e-01,
     -2.854859531e-01, -2.781058252e-01, -2.707797587e-01, -2.635069788e-01,
     -2.562867701e-01, -2.491182536e-01, -2.420008332e-01, -2.349336445e-01,
     -2.279160470e-01, -2.209473550e-01, -2.140269727e-01, -2.071540654e-01,
     -2.003280669e-01, -1.935484409e-01, -1.868143827e-01, -1.801253706e-01,
     -1.734808683e-01, -1.668801606e-01, -1.603227407e-01, -1.538081169e-01,
     -1.473355740e-01, -1.409046650e-01, -1.345148385e-01, -1.281656623e-01,
     -1.218564659e-01, -1.155868992e-01, -1.093563288e-01, -1.031643376e-01,
     -9.701044858e-02, -9.089426696e-02, -8.481520414e-02, -7.877293229e-02,
     -7.276688516e-02, -6.679669768e-02, -6.086194515e-02, -5.496226251e-02,
     -4.909712449e-02, -4.326624796e-02, -3.746910766e-02, -3.170538321e-02,
     -2.597468905e-02, -2.027664892e-02, -1.461095270e-02, -8.977175690e-03,
     -3.374900669e-03,  2.196163638e-03,  7.736362983e-03,  1.324603800e-02,
      1.872540452e-02,  2.417502925e-02,  2.959511615e-02,  3.498598188e-02,
      4.034794495e-02,  4.568130895e-02,  5.098637938e-02,  5.626345426e-02,
      6.151271611e-02,  6.673467904e-02,  7.192951441e-02,  7.709749788e-02,
      8.223880827e-02,  8.735392243e-02,  9.244301170e-02,  9.750632942e-02,
      1.025441438e-01,  1.075567007e-01,  1.125442609e-01,  1.175070703e-01,
      1.224452630e-01,  1.273592860e-01,  1.322492957e-01,  1.371155083e-01,
      1.419581473e-01,  1.467773467e-01,  1.515735239e-01,  1.563468277e-01,
      1.610974520e-01,  1.658256054e-01,  1.705315113e-01,  1.752153784e-01,
      1.798774004e-01,  1.845176965e-01,  1.891366690e-01,  1.937343925e-01,
      1.983110756e-01,  2.028668076e-01,  2.074019760e-01,  2.119166702e-01,
      2.164110839e-01,  2.208853662e-01,  2.253397405e-01,  2.297743559e-01,
      2.341893911e-01,  2.385850102e-01,  2.429613024e-01,  2.473186255e-01,
      2.516570389e-01,  2.559766173e-01,  2.602777183e-01,  2.645604014e-01,
      2.688248158e-01,  2.730711102e-01,  2.772994637e-01,  2.815100253e-01,
      2.857029140e-01,  2.898783088e-01,  2.940362394e-01,  2.981770337e-01,
      3.023007810e-01,  3.064075708e-01,  3.104974627e-01,  3.145708144e-01,
      3.186276257e-01,  3.226680458e-01,  3.266922235e-01,  3.307002485e-01,
      3.346922994e-01,  3.386684656e-01,  3.426288664e-01,  3.465735614e-01};
    y =  (x - xmn) * odx;
    i = (int) y;
    f = y - i;
    if (f == 0) return (lu[i]);
    return ((1 - f) * lu[i] + f * lu[i + 1]);
}

#endif // METHOD

/***********************************************************/

FUNC(float)
cha_db1(float x) // 10 * log10(x)
{
    float m, ln;
    int e;
    static float c1 = 1e-38;
    static float c2 = -380;
    static float c3 = 1e38;
    static float c4 = 380;
    static float c5 = 0.707106769; // sqrt(0.5)
    static float c6 = 4.34294462;  // 10 / log(10)
    static float c7 = 0.693147182; // log(2);

    if (x < c1) return (c2);
    if (x > c3) return (c4);
    m = frexpf(x, &e);
    if (m < c5) {
        m *= 2;
  e--;
    }
#if METHOD == 0
    ln = logf(m); // exact
#elif METHOD == 1
    ln = log_pr(m);
#elif METHOD == 2
    ln = log_lu(m);
#endif
    return (c6 * (ln + c7 * e));
}

FUNC(float)
cha_undb1(float x) // 10 ^ (x / 10)
{
    float m, p2;
    int   e;
    static float c1 = 1e-38;
    static float c2 = -380;
    static float c3 = 1e38;
    static float c4 = 380;
    static float c5 = 0.166096404; // log(10) / (20 * log(2))

    if (x < c2) return (c1);
    if (x > c4) return (c3);
    x *= c5;
    e = (int) x;
    m = x - e;
    if (m < -0.5) {
        m++;
        e--;
    } else if (m > 0.5) {
        m--;
        e++;
    }
#if METHOD == 0
    p2 = powf(2, m); // exact
#elif METHOD == 1
    p2 = pow_pr(m); 
#elif METHOD == 2
    p2 = pow_lu(m); 
#endif
    return (ldexpf(p2, e));
}

/***********************************************************/

FUNC(float)
cha_db2(float x) // 20 * log10(x)
{
    float m, ln;
    int e;
    static float c1 = 1e-38;
    static float c2 = -760;
    static float c3 = 1e38;
    static float c4 = 760;
    static float c5 = 0.707106769; // sqrt(0.5)
    static float c6 = 8.68588924;  // 20 / log(10)
    static float c7 = 0.693147182; // log(2);

    if (x < c1) return (c2);
    if (x > c3) return (c4);
    m = frexpf(x, &e);
    if (m < c5) {
        m *= 2;
  e--;
    }
    // assume: x > sqrt(0.5) and x < sqrt(2)
#if METHOD == 0
    ln = logf(m); // exact
#elif METHOD == 1
    ln = log_pr(m);
#elif METHOD == 2
    ln = log_lu(m);
#endif
    return (c6 * (ln + c7 * e));
}

FUNC(float)
cha_undb2(float x) // 10 ^ (x / 20)
{
    float m, p2;
    int   e;
    static float c1 = 1e-38;
    static float c2 = -760;
    static float c3 = 1e38;
    static float c4 = 760;
    static float c5 = 0.166096404; // log(10) / (20 * log(2))

    if (x < c2) return (c1);
    if (x > c4) return (c3);
    x *= c5;
    e = (int) ((x < 0) ? (x - 0.5) : (x + 0.5));
    m = x - e;
    // assume: x > -0.5 and x < 0.5
#if METHOD == 0
    p2 = powf(2, m); // exact
#elif METHOD == 1
    p2 = pow_pr(m); 
#elif METHOD == 2
    p2 = pow_lu(m); 
#endif
    return (ldexpf(p2, e));
}

