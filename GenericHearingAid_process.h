extern "C" {
#include "chapro.h"
#include "cha_ff.h"
#include "cha_ff_data.h"
}
/*
   GenericHearingAid_process

   Created: Chip Audette, December 2016
   Purpose; Here is the skeleton of a audio processing algorithm that will
       (hopefully) make it easier for people to start making their own 
       algorithm.
   This processes a single stream of audio data (ie, it is mono)

   Modified by: Daniel Rasetshwane, January 2017
   Purpose: Implements Generic Hearing Aid signal processing   
  

   MIT License.  use at your own risk.
*/

#include <arm_math.h> //ARM DSP extensions.  https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
#include <AudioStream_F32.h>


class AudioEffectMine_F32 : public AudioStream_F32
{
   public:
    //constructor
    AudioEffectMine_F32(void) : AudioStream_F32(1, inputQueueArray_f32) {
      //do any setup activities here
    };

    //here's the method that is called automatically by the Teensy Audio Library
    void update(void) {
      //Serial.println("AudioEffectMine_F32: doing update()");  //for debugging.
      audio_block_f32_t *audio_block;
      audio_block = AudioStream_F32::receiveWritable_f32();
      if (!audio_block) return;

      //do your work
      CHA_PTR cp;
      cp = (CHA_PTR) cha_data; 

      int n = 128;  // chunck size
      int nc = 8;   // number of channels

      float *x; // hold filterbank data

      x = (float *) calloc(n * nc * 2, sizeof(float)); // better, initializes memory
      //x = (float *) malloc(n * nc * 2 * sizeof(float)); // faster, does not initialize memory

      applyMyAlgorithm(audio_block,cp,x,n);
      

      ///transmit the block and release memory
      AudioStream_F32::transmit(audio_block);
      AudioStream_F32::release(audio_block);
      free(x);
    }
     
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    // Here is where you can add your algorithm.
    // This function gets called block-wise...which is usually hard-coded to every 128 samples
    void applyMyAlgorithm(audio_block_f32_t *audio_block,CHA_PTR cp,float *x,int n) {
            
      cha_agc_input(cp, audio_block->data, audio_block->data, n);
      //cha_firfb_analyze(cp, audio_block->data, x, n);
      //cha_agc_channel(cp, x, x, n);
      //cha_firfb_synthesize(cp, x, audio_block->data, n);
      cha_agc_output(cp, audio_block->data, audio_block->data, n);
      
    } //end of applyMyAlgorithms
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    // Call this method in your main program if you want to set the value of your user parameter. 
    // The user parameter can be used in your algorithm above.  The user_parameter variable was
    // created in the "private:" section of this class, which appears a little later in this file.
    // Feel free to create more user parameters (and to use better names for your variables)
    // for use in this class.
    float32_t setUserParameter(float val) {
      return user_parameter = val;
    }
 
  private:
    //state-related variables
    audio_block_f32_t *inputQueueArray_f32[1]; //memory pointer for the input to this module

    //this value can be set from the outside (such as from the potentiometer) to control
    //a parameter within your algorithm
    float32_t user_parameter = 0.0;   

};  //end class definition for AudioEffectMine_F32
