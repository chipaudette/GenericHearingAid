extern "C" {
#include "chapro.h"
#include "cha_ff.h"
#if AUDIO_BLOCK_SAMPLES == 32
  #include "cha_ff_data32.h"
#elif AUDIO_BLOCK_SAMPLES == 64
  #include "cha_ff_data64.h"
#elif AUDIO_BLOCK_SAMPLES == 128
  #include "cha_ff_data128.h"
#endif
}

//#define CHUNK_SIZE 128
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
#include "AudioStream_Mod.h"
#include <AudioStream_F32.h>

#if USE_F32_AUDIO_BLOCKS == 1
#define INHERIT_CLASS AudioStream_F32
#define AUDIO_TYPE audio_block_f32_t
class AudioEffectMine : public INHERIT_CLASS
#else
#define INHERIT_CLASS AudioStream
#define AUDIO_TYPE audio_block_t
class AudioEffectMine : public INHERIT_CLASS
#endif
{
   public:
    //constructor
    AudioEffectMine(void) : INHERIT_CLASS(1, inputQueueArray) {
      //do any setup activities here
    };

    //here's the method that is called automatically by the Teensy Audio Library
    void update(void) {
      //Serial.println("AudioEffectMine: doing update()");  //for debugging.
      AUDIO_TYPE *audio_block;
      #if USE_F32_AUDIO_BLOCKS
        audio_block = INHERIT_CLASS::receiveWritable_f32();
      #else
        audio_block = INHERIT_CLASS::receiveWritable();
      #endif
      if (!audio_block) return;

      //convert to floating point data type
      audio_block_f32_t *audio_block_f32;
      #if (USE_F32_AUDIO_BLOCKS==1)
        //audio is laready in F32 format
        audio_block_f32 = audio_block; //simply copy the pointer
      #else
        //convert integer to F32 format
        audio_block_f32_t audio_block_f32_data;
        audio_block_f32 = &audio_block_f32_data;
        //AudioConvert_I16toF32::convertAudio_I16toF32(audio_block,audio_block_f32,audio_block_f32->length);
        //const float MAX_INT = 32678.0;
        const int len = audio_block_f32->length;
        for (int i = 0; i < len; i++) audio_block_f32->data[i] = (float)(audio_block->data[i]);
        arm_scale_f32(audio_block_f32->data, 1.0f/32678.0f, audio_block_f32->data, audio_block_f32->length); //divide by 32678 to get -1.0 to +1.0        
      #endif

      //users could choose to put all of their processing in this method
      applyMyAlgorithm(audio_block_f32);
      
      //convert back to Int16 data type, if needed
      #if (USE_F32_AUDIO_BLOCKS==1)
        //don't need to convert, we just send the F32 data
        audio_block = audio_block_f32;//simply copy the pointer
      #else
        //convert F32 to integer
        //AudioConvert_F32toI16::convertAudio_F32toI16(audio_block_f32,audio_block,audio_block_f32->length);
        //const float MAX_INT = 32678.0;
        arm_scale_f32(audio_block_f32->data,32768.0f,audio_block_f32->data,len);
        for (int i = 0; i < len; i++) {
          //audio_block->data[i] = (int16_t)(max(min( (audio_block_f32->data[i] * 32678.0f), 32678.0f), -32678.0f));
          audio_block->data[i] = (int16_t)(max(min(audio_block_f32->data[i], 32678.0f), -32678.0f));
        }        
      #endif

      ///transmit the block and release memory
      INHERIT_CLASS::transmit(audio_block);
      INHERIT_CLASS::release(audio_block);

    }
     
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    // Here is where you can add your algorithm.
    // This function gets called block-wise...which is usually hard-coded to every 128 samples
    void applyMyAlgorithm(audio_block_f32_t *audio_block) {
      //get and set CHA-specific parameters
      CHA_PTR cp;
      cp = (CHA_PTR) cha_data; 
      int n = AUDIO_BLOCK_SAMPLES;  // chunck size
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
    AUDIO_TYPE *inputQueueArray[1]; //memory pointer for the input to this module

    //this value can be set from the outside (such as from the potentiometer) to control
    //a parameter within your algorithm
    float32_t user_parameter = 0.0;  

    //memory for CHA processing
    float32_t x[AUDIO_BLOCK_SAMPLES * NUM_FREQ_CHAN * 2];

};  //end class definition for AudioEffectMine_F32

