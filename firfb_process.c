// firfb_process.c - FIR-filterbank processing functions

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "chapro.h"
#include "cha_ff.h"


//Added for ARM FFT/IFFT processing
#define USE_ARM_MATH 1
#if USE_ARM_MATH == 1
  #include <arm_math.h>
  #define ARM_NFFT (128*2)   //CHUNK_SIZE * 2...YOU MUST SET THIS VALUE YOURSELF!
  //#define ARM_NFFT (64*2)   //CHUNK_SIZE * 2...YOU MUST SET THIS VALUE YOURSELF!
  //#define ARM_NFFT (32*2)   //CHUNK_SIZE * 2...YOU MUST SET THIS VALUE YOURSELF!

  #if (ARM_NFFT == 64) || (ARM_NFFT == 256)
    //use radix4, which is faster than radix2.
    #define ARM_FFT_INST_TYPE arm_cfft_radix4_instance_f32  //radix 4 is for NFFT=64 and NFFT=256
    #define ARM_FFT_INIT_FUNC arm_cfft_radix4_init_f32
    #define ARM_FFT_FUNC arm_cfft_radix4_f32
//  #elif (ARM_NFFT == 128)
//    //use rfft
//    #define USE_ARM_RFFT
//    #define ARM_FFT_INST_TYPE arm_rfft_instance_f32
//    #define ARM_FFT_RAD4_INST_TYPE arm_cfft_radix4_instance_f32
//    #define ARM_FFT_INIT_FUNC arm_rfft_init_f32
//    #define ARM_FFT_FUNC arm_rfft_f32   
  #else
    //use radix2, which is slower than radix4.  Doing an RFFT is also possible at NFFT=128 or NFFT=512
    #define ARM_FFT_INST_TYPE arm_cfft_radix2_instance_f32  //radix 2 is for NFFT=32 and NFFT=128
    #define ARM_FFT_INIT_FUNC arm_cfft_radix2_init_f32
    #define ARM_FFT_FUNC arm_cfft_radix2_f32
  #endif

  //create ARM Math FFT instances
  ARM_FFT_INST_TYPE fft_inst1, ifft_inst1; 
  #ifdef USE_ARM_RFFT
    ARM_FFT_RAD4_INST_TYPE fft_rad4_inst, ifft_rad4_inst;
  #endif

  //create temporary memory
  float xx_temp[2*ARM_NFFT], yy_temp[2*ARM_NFFT];
  float xx_temp2[2*ARM_NFFT], yy_temp2[2*ARM_NFFT];

  //define initialization functions
  static void initialize_ARM_FFT(void) {
    uint8_t ifftFlag; // 0 is FFT, 1 is IFFT
    uint8_t doBitReverse = 1;
    int FFT_allocation_status=-9, IFFT_allocation_status=-9;
    
    ifftFlag = 0; //zero says to setup as FFT
    #ifdef USE_ARM_RFFT
      FFT_allocation_status = ARM_FFT_INIT_FUNC(&fft_inst1, &fft_rad4_inst, ARM_NFFT, ifftFlag, doBitReverse); //init FFT
    #else
      FFT_allocation_status = ARM_FFT_INIT_FUNC(&fft_inst1, ARM_NFFT, ifftFlag, doBitReverse); //init FFT
    #endif
    
    ifftFlag = 1; //one says to setup as IFFT
    #ifdef USE_ARM_RFFT
      IFFT_allocation_status = ARM_FFT_INIT_FUNC(&ifft_inst1, &ifft_rad4_inst, ARM_NFFT, ifftFlag, doBitReverse); //init FFT
    #else
      IFFT_allocation_status = ARM_FFT_INIT_FUNC(&ifft_inst1, ARM_NFFT, ifftFlag, doBitReverse); //init IFFT    
    #endif
  }

  static void rebuildNegFreqBins(float data[], int n_fft) {
      //create the negative frequency space via complex conjugate of the positive frequency space
      const int N_POS_BINS = n_fft / 2 + 1;
      const int ind_nyquist_bin = N_POS_BINS-1;
      int targ_ind = ind_nyquist_bin+1; //start targeting one above nyquest
      int source_ind;
      for (source_ind = (ind_nyquist_bin-1); source_ind > 0; source_ind--) {  //start sourcing one below nyquist.  End before DC.
        data[2*targ_ind] = data[2*source_ind]; //real
        data[2*targ_ind+1] = -data[2*source_ind+1]; //imaginary.  negative sign makes it the complex conjugate, which is what we want for the neg freq space
        targ_ind++;
      }
    }
  
#endif

/***********************************************************/

// complex multiply: z = x * y
static __inline void
cmul(float *z, float *x, float *y, int n)
{
    int      i, ir, ii;

    for (i = 0; i < n; i++) {
        ir = i * 2;
        ii = i * 2 + 1;
        z[ir] = x[ir] * y[ir] - x[ii] * y[ii];
        z[ii] = x[ir] * y[ii] + x[ii] * y[ir];
    }
}

// FIR-filterbank analysis for short chunk (cs < nw)
static __inline void
firfb_analyze_sc_orig(float *x, float *y, int cs,
    float *hh, float *xx, float *yy, float *zz, int nc, int nw)
{
    float   *hk, *yk, *zk;
    int      i, j, k, nf, ns, nt, nk;

    nk = nw / cs;
    nt = cs * 2;
    nf = cs + 1;
    ns = nf * 2;
    // loop over channels
    //for (k = 0; k < nc; k++) {
    for (k=0; k < 6; k++) {
        fzero(xx, nt);
        fcopy(xx, x, cs);
        cha_fft_rc(xx, nt);
        // loop over sub-window segments
        yk = y + k * cs;
        zk = zz + k * (nw + cs);
        for (j = 0; j < nk; j++) {
            hk = hh + (k * nk + j) * ns;
            #if USE_ARM_MATH
              arm_cmplx_mult_cmplx_f32(xx, hk, yy, nf);
            #else
              cmul(yy, xx, hk, nf);
            #endif
            cha_fft_cr(yy, nt);
            for (i = 0; i < nt; i++) {
                zk[i + j * cs] += yy[i];
            }
        }
        fcopy(yk, zk, cs);
        fmove(zk, zk + cs, nw);
        fzero(zk + nw, cs);
    }
}

static __inline void
firfb_analyze_sc_wARM(float *x, float *y, int cs,
    float *hh, float *xx, float *yy, float *zz, int nc, int nw)
{
    float   *hk, *yk, *zk;
    int      i, j, k, nf, ns, nt, nk;

    nk = nw / cs;
    nt = cs * 2;
    nf = cs + 1;
    ns = nf * 2;

    #if USE_ARM_MATH
      //use ARM-specific FFT acceleration
      #ifdef USE_ARM_RFFT
        //fzero(xx, nt);
        fzero(xx_temp, 2*nt);
        fzero(xx_temp2, 2*nt);
        fcopy(xx_temp2, x, cs); //just to make sure that we don't damage the input data
        ARM_FFT_FUNC(&fft_inst1, xx_temp2, xx_temp);
      #else   
        for (k = 0; k < cs; k++) { xx_temp[2*k]=x[k+j]; xx_temp[2*k+1]=0.0f;} //ni = nw = 128
        for (k=cs; k < nt; k++) { xx_temp[2*k]=0.0f; xx_temp[2*k+1] = 0.0f; } ///zero pad the rest of the buffer
        ARM_FFT_FUNC(&fft_inst1, xx_temp); //DSP accelerated       
      #endif    
    #else
      //original
      fzero(xx, nt); //original, but moved up here
      fcopy(xx, x, cs); //original, but moved up here
      cha_fft_rc(xx, nt);    //original, but moved up here 
    #endif

    // loop over channels
    //for (k = 0; k < nc; k++) {
    for (k = 0; k < 4; k++) {
        //fzero(xx, nt); //original location
        //fcopy(xx, x, cs); //original location
        //cha_fft_rc(xx, nt); //original location
        
        // loop over sub-window segments
        yk = y + k * cs;
        zk = zz + k * (nw + cs);
        for (j = 0; j < nk; j++) {
            hk = hh + (k * nk + j) * ns;
            #if USE_ARM_MATH
              fzero(yy_temp, 2*nt);
              #ifdef USE_ARM_RFFT
                fzero(yy_temp2, 2*nt);
                arm_cmplx_mult_cmplx_f32(xx_temp, hk, yy_temp2, nf);
              #else
                arm_cmplx_mult_cmplx_f32(xx_temp, hk, yy_temp, nf);
              #endif
            #else
              cmul(yy, xx, hk, nf);
            #endif

            //now do inverse FFT
            #if USE_ARM_MATH
              #define ORIG_YY 0
              #ifdef USE_ARM_RFFT
                ARM_FFT_FUNC(&ifft_inst1, yy_temp2, yy_temp);
              #else
                #define ORIG_YY 0
                rebuildNegFreqBins(yy_temp, nt); //recreate the negative freqs
                ARM_FFT_FUNC(&ifft_inst1, yy_temp);
              #endif
            #else
              #define ORIG_YY 1
              //original
              cha_fft_cr(yy, nt);
            #endif
   
            for (i = 0; i < nt; i++) {
              #if ORIG_YY
                zk[i + j * cs] += yy[i];
              #else
                #ifdef USE_ARM_RFFT
                  zk[i + j * cs] += yy_temp[i];
                #else
                  zk[i + j * cs] += yy_temp[2*i];
                #endif
              #endif
            }
        }
        fcopy(yk, zk, cs);
        fmove(zk, zk + cs, nw);
        fzero(zk + nw, cs);
    }
}

// FIR-filterbank analysis for long chunk (cs >= nw)
static __inline void
firfb_analyze_lc(float *x, float *y, int cs,
    float *hh, float *xx, float *yy, float *zz, int nc, int nw)
{
    float   *hk, *yk, *zk;
    int      i, j, k, nf, nt, ni;
    
    //nw = 128;  //length of window of new data
    nt = nw * 2; //length of FFT transform is 256 points. (will zero pad the last half)
    nf = nw + 1; //length of positive frequeny space of FFT data (ie, DC through Nyquist..128+1 = 129
        
    // loop over sub-chunk segments
    for (j = 0; j < cs; j += nw) {
        ni = ((cs - j) < nw) ? (cs - j) : nw;
        
        #if USE_ARM_MATH
          #ifdef USE_ARM_RFFT
            //fzero(xx, nt);
            //fcopy(xx, x + j, ni); 
            fzero(xx_temp, nt); 
            fcopy(xx_temp, x+j, ni);  //just real, not interleaved complex
            ARM_FFT_FUNC(&fft_inst1, xx_temp, xx_temp);
          #else
            for (k = 0; k < ni; k++) { xx_temp[2*k]=x[k+j]; xx_temp[2*k+1]=0.0f;} //ni = nw = 128
            for (k=ni; k < nt; k++) { xx_temp[2*k]=0.0f; xx_temp[2*k+1] = 0.0f; } ///zero pad the rest of the buffer
            ARM_FFT_FUNC(&fft_inst1, xx_temp); //DSP accelerated
          #endif
        #else
          fzero(xx, nt);
          fcopy(xx, x + j, ni);      
          cha_fft_rc(xx, nt); //FFT
        #endif
          
        // loop over channels
        for (k = 0; k < nc; k++) {     
            hk = hh + k * nf * 2;
            #if USE_ARM_MATH
              arm_cmplx_mult_cmplx_f32(xx_temp, hk, yy_temp, nf); //complex multiply (ie, create the current channel)
              #ifdef USE_ARM_RFFT
                ARM_FFT_FUNC(&ifft_inst1, yy_temp, yy_temp); //yy_temp is now real, not interleaved complex
              #else
                rebuildNegFreqBins(yy_temp, nt); //nt is 256
                ARM_FFT_FUNC(&ifft_inst1, yy_temp); //yy_temp is interleaved complex (thought imaginary should be zero)
              #endif
            #else
              cmul(yy, xx, hk, nf); //complex multiply (ie, create the current channel)
              cha_fft_cr(yy, nt);   //IFFT.  yy should be real, not interleaved comples
            #endif
            
            yk = y + k * cs;
            zk = zz + k * nw;

            #if USE_ARM_MATH
              #ifdef USE_ARM_RFFT
                //yy_temp should just be real, not interleaved complex
                for (i = 0; i < ni; i++) {  yk[i + j] = yy_temp[i] + zk[i]; }
                fcopy(zk, yy_temp + ni, nw);
              #else
                //yy is interleaved real-complex-real-complex...just get the real part.
                for (i = 0; i < ni; i++)  yk[i + j] = yy_temp[2*i] + zk[i];
  
                //copy out the state of the system (the output that had been the zero pad...again, just the real part)
                float *yy_foo = yy_temp + 2*ni;  
                for (i = 0; i < nw; i++)  zk[i] = yy_foo[2*i];  //WEA MODDED...replaces fcopy(zk, yy + ni, nw);
              #endif
            #else
              //original code..yy should just be real, not interleaved complex
              for (i = 0; i < ni; i++) {  yk[i + j] = yy[i] + zk[i]; }
              fcopy(zk, yy + ni, nw);
            #endif
        }
    }
}

// FIR-filterbank analysis
FUNC(void)
cha_firfb_analyze(CHA_PTR cp, float *x, float *y, int cs)
{
    float   *hh, *xx, *yy, *zz;
    int      nc, nw;
    #if USE_ARM_MATH
      static int firstTime = 1;
      //This will get checked every time.  But, without making this a class, I don't know how to do this.
      if (firstTime) {  initialize_ARM_FFT();  firstTime = 0; }
    #endif


    nc = CHA_IVAR[_nc];
    nw = CHA_IVAR[_nw];
    hh = (float *) cp[_ffhh];
    xx = (float *) cp[_ffxx];
    yy = (float *) cp[_ffyy];
    zz = (float *) cp[_ffzz];
    if (cs < nw) {
        //firfb_analyze_sc_orig(x, y, cs, hh, xx, yy, zz, nc, nw);
        firfb_analyze_sc_wARM(x, y, cs, hh, xx, yy, zz, nc, nw);
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
