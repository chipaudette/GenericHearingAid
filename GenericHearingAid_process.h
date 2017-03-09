extern "C" {
#include "chapro.h"
#include "cha_ff.h"
#include "cha_ff_data128.h"
}

#define CHUNK_SIZE 128
#define NUM_FREQ_CHAN 8

/*
   GenericHearingAid_process

   Created: Chip Audette, December 2016

   Modified by: Daniel Rasetshwane, January 2017
   Purpose: Implements Generic Hearing Aid signal processing   
   
   This processes a single stream of audio data (ie, it is mono) 

   MIT License.  use at your own risk.
*/

#include <arm_math.h> //ARM DSP extensions.  https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
#include <AudioStream_F32.h>

class AudioGenericHearingAid_F32 : public AudioStream_F32
{
   public:
    //constructor
    AudioGenericHearingAid_F32(void) : AudioStream_F32(1, inputQueueArray_f32) {
      //do any setup activities here
    };

    //here's the method that is called automatically by the Teensy Audio Library
    void update(void) {
      //Serial.println("AudioEffectMine_F32: doing update()");  //for debugging.
      audio_block_f32_t *audio_block;
      audio_block = AudioStream_F32::receiveWritable_f32();
      if (!audio_block) return;

      //users could choose to put all of their processing in this method
      applyMyAlgorithm(audio_block);
      

      ///transmit the block and release memory
      AudioStream_F32::transmit(audio_block);
      AudioStream_F32::release(audio_block);

    }
     
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    // Here is where you can add your algorithm.
    // This function gets called block-wise...which is usually hard-coded to every 128 samples
    void applyMyAlgorithm(audio_block_f32_t *audio_block) {
      //get and set CHA-specific parameters
      CHA_PTR cp;
      cp = (CHA_PTR) cha_data; 
      int n = CHUNK_SIZE;  // chunck size
      int nc = NUM_FREQ_CHAN;   // number of channels

      //allocate some memory for CHA processing
      //float *x; // hold filterbank data
      //x = (float *) calloc(n * nc * 2, sizeof(float)); // better, initializes memory
      //x = (float *) malloc(n * nc * 2 * sizeof(float)); // faster, does not initialize memory
      memset(x,0.0f,n*nc*2);  //clear this working memory

      //do CHA processing
      cha_agc_input(cp, audio_block->data, audio_block->data, n);
      cha_firfb_analyze(cp, audio_block->data, x, n);
      cha_agc_channel(cp, x, x, n);
      cha_firfb_synthesize(cp, x, audio_block->data, n);
      cha_agc_output(cp, audio_block->data, audio_block->data, n);

      //free up memory
      //free(x);

      //processed audio is returned back through audio_block
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

    //memory for CHA processing
    float32_t x[CHUNK_SIZE * NUM_FREQ_CHAN * 2];

};  //end class definition for AudioEffectMine_F32

