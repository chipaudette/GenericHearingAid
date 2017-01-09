// cha_ff.h - FIR-filterbank & AGC
#ifdef __cplusplus
extern "C" {
#endif
    
#ifndef CHA_FF_H
#define CHA_FF_H

/*****************************************************/

// DSL prescription

#define DSL_MXCH 32              // maximum number of channels

typedef struct {
    double attack;               // attack time (ms)
    double release;              // release time (ms)
    double maxdB;                // maximum signal (dB SPL)
    int ear;                     // 0=left, 1=right
    int nchannel;                // number of channels
    double cross_freq[DSL_MXCH]; // cross frequencies (Hz)
    double tkgain[DSL_MXCH];     // compression-start gain
    double cr[DSL_MXCH];         // compression ratio
    double tk[DSL_MXCH];         // compression-start kneepoint
    double bolt[DSL_MXCH];       // broadband output limiting threshold
} CHA_DSL;

typedef struct {
    double attack;               // attack time (ms)
    double release;              // release time (ms)
    double fs;                   // sampling rate (Hz)
    double maxdB;                // maximum signal (dB SPL)
    double tkgain;               // compression-start gain
    double tk;                   // compression-start kneepoint
    double cr;                   // compression ratio
    double bolt;                 // broadband output limiting threshold
} CHA_WDRC;

typedef struct {
    char *ifn, *ofn, mat;
    double rate;
    float *iwav, *owav;
    long *siz;
    long iod, nwav, nsmp, mseg, nseg, oseg, pseg;
    void **out;
} I_O;
    
/*****************************************************/

// firfb module

FUNC(int) cha_firfb_prepare(CHA_PTR, double *, int, double, int, int, int);
FUNC(void) cha_firfb_analyze(CHA_PTR, float *, float *, int);
FUNC(void) cha_firfb_synthesize(CHA_PTR, float *, float *, int);

// compressor module

FUNC(int) cha_agc_prepare(CHA_PTR, CHA_DSL *, CHA_WDRC *, double);
FUNC(void) cha_agc_input(CHA_PTR, float *, float *, int);
FUNC(void) cha_agc_channel(CHA_PTR, float *, float *, int);
FUNC(void) cha_agc_output(CHA_PTR, float *, float *, int);

// call firfb and compressor modules
//FUNC(static void) gha_process(I_O, CHA_PTR);

/*****************************************************/

#define _offset   _reserve

// pointer indices

#define _cc       _offset+0
#define _ffhh     _offset+2
#define _ffxx     _offset+4
#define _ffyy     _offset+5
#define _ffzz     _offset+6
#define _gctk     _offset+7
#define _gccr     _offset+8
#define _gctkgn   _offset+9
#define _gcbolt   _offset+10
#define _gcppk    _offset+11
#define _xsc      _offset+12
#define _xpk      _offset+13
#define _ppk      _offset+14

// integer variable indices

#define _cs       0 
#define _nw       1
#define _nc       3

// double variable indices

#define _alfa     0
#define _beta     1
#define _fs       2
#define _mxdb     3
#define _tkgn     4
#define _tk       5
#define _cr       6
#define _bolt     7
#define _scl      8
#define _gcalfa   9
#define _gcbeta   10

#endif /* CHA_FF_H */
    
#ifdef __cplusplus
}
#endif
