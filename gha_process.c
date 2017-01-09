//  gha_process.c
//  
//
//  Created by Rasetshwane, Daniel M. on 1/8/17.
//
//
#ifdef __cplusplus
extern "C" {
#endif
    
    
#include <stdio.h>
#include "chapro.h"
#include "cha_ff.h"

static void
gha_process(float *x, float *y, CHA_PTR cp)
{
    int cs;
    
    // initialize i/o pointers
    //x = io->iwav;
    //y = io->owav;
    cs = CHA_IVAR[_cs];
    // analyze input waveform
    cha_firfb_analyze(cp, x, y, cs);
}

#ifdef __cplusplus
}
#endif
