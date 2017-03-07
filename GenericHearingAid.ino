/*
   GenericHearingAid

   Created: Chip Audette, Dec 2016
   Modified by: Daniel Rasetshwane, January 2017
   Purpose: Implements Generic Hearing Aid signal processing

   Uses Teensy Audio Adapter.
   Assumes microphones (or whatever) are attached to the LINE IN (stereo)
   Listens  potentiometer mounted to Audio Board to provde a control signal.

   MIT License.  use at your own risk.
*/

#define CUSTOM_SAMPLE_RATE 24000     //See local AudioStream_Mod.h.  Only a limitted number supported
#define CUSTOM_BLOCK_SAMPLES 128     //See local AudioStream_Mod.h.  Do not change.  Doesn't work yet.

//Use test tone as input (set to 1)?  Or, use live audio (set to zero)
#define USE_TEST_TONE_INPUT 1

//include my custom AudioStream.h...this prevents the default one from being used
#include "AudioStream_Mod.h"

//These are the includes from the Teensy Audio Library
#include <Audio.h>      //Teensy Audio Library
#include <Wire.h>
#include <SPI.h>
//include <SD.h>
//include <SerialFlash.h>

#include <OpenAudio_ArduinoLibrary.h> //for AudioConvert_I16toF32, AudioConvert_F32toI16, and AudioEffectGain_F32
#include "GenericHearingAid_process.h"


//create audio library objects for handling the audio
AudioControlSGTL5000    audioHardware;     //controller for the Teensy Audio Board
AudioSynthWaveformSine  testSignal;          //use to generate test tone as input
AudioInputI2S           i2s_in;          //Digital audio *from* the Teensy Audio Board ADC.  Sends Int16.  Stereo.
AudioOutputI2S          i2s_out;        //Digital audio *to* the Teensy Audio Board DAC.  Expects Int16.  Stereo
AudioConvert_I16toF32   int2Float1;     //Converts Int16 to Float.  See class in AudioStream_F32.h
AudioEffectMine_F32     effect1;        //This is your own algorithms
AudioConvert_F32toI16   float2Int1;     //Converts Float to Int16.  See class in AudioStream_F32.h


//Make all of the audio connections
#if (USE_TEST_TONE_INPUT == 1)
  //use test tone as audio input
  AudioConnection         patchCord1(testSignal, 0, int2Float1, 0);    //connect the Left input to the Left Int->Float converter
#else
  //use real audio input (microphones or line-in)
  AudioConnection         patchCord1(i2s_in, 0, int2Float1, 0);    //connect the Left input to the Left Int->Float converter
#endif
AudioConnection_F32     patchCord10(int2Float1, 0, effect1, 0);    //Left.  makes Float connections between objects
AudioConnection_F32     patchCord12(effect1, 0, float2Int1, 0);    //Left.  makes Float connections between objects
AudioConnection         patchCord20(float2Int1, 0, i2s_out, 0);  //connect the Left float processor to the Left output
AudioConnection         patchCord21(float2Int1, 0, i2s_out, 1);  //connect the Right float processor to the Right output

// which input on the audio shield will be used?
//const int myInput = AUDIO_INPUT_LINEIN;   //or, do AUDIO_INPUT_MIC
const int myInput = AUDIO_INPUT_MIC;   //or, do AUDIO_INPUT_MIC

//I have a potentiometer on the Teensy Audio Board
#define POT_PIN A1  //potentiometer is tied to this pin

// define the setup() function, the function that is called once when the device is booting
void setup() {
  Serial.begin(115200);   //open the USB serial link to enable debugging messages
  delay(500);             //give the computer's USB serial system a moment to catch up.
  Serial.println("GenericHearingAid: setup()...");
  Serial.print("Global: F_CPU: "); Serial.println(F_CPU); 
  Serial.print("Global: F_PLL: "); Serial.println(F_PLL);
  Serial.print("Global: AUDIO_SAMPLE_RATE: "); Serial.println(AUDIO_SAMPLE_RATE);
  Serial.print("Global: AUDIO_BLOCK_SAMPLES: "); Serial.println(AUDIO_BLOCK_SAMPLES);

  // Audio connections require memory
  AudioMemory(10);      //allocate Int16 audio data blocks
  AudioMemory_F32(10);  //allocate Float32 audio data blocks

  //change the sample rate...this is required for any sample rate other than 44100...WEA to fix.
  setI2SFreq((int)AUDIO_SAMPLE_RATE); //set the sample rate for the Audio Card (the rest of the library doesn't know, though)
 
  // Enable the audio shield, select input, and enable output
  audioHardware.enable();                   //start the audio board
  audioHardware.inputSelect(myInput);       //choose line-in or mic-in
  audioHardware.volume(0.8);                //volume can be 0.0 to 1.0.  0.5 seems to be the usual default.
  audioHardware.lineInLevel(10, 10);        //level can be 0 to 15.  5 is the Teensy Audio Library's default
  audioHardware.adcHighPassFilterDisable(); //reduces noise.  https://forum.pjrc.com/threads/27215-24-bit-audio-boards?p=78831&viewfull=1#post78831

  // setup any other other features
  pinMode(POT_PIN, INPUT); //set the potentiometer's input pin as an INPUT

  //setup sine wave as test signal
  testSignal.amplitude(0.01);
  testSignal.frequency(500.0f);
  Serial.println("setup() complete");
} //end setup()


// define the loop() function, the function that is repeated over and over for the life of the device
void loop() {
  //choose to sleep ("wait for interrupt") instead of spinning our wheels doing nothing but consuming power
  asm(" WFI");  //ARM-specific.  Will wake on next interrupt.  The audio library issues tons of interrupts, so we wake up often.

  //service the potentiometer...if enough time has passed
  servicePotentiometer(millis());

  //update the memory and CPU usage...if enough time has passed
  printMemoryAndCPU(millis());
} //end loop()


//servicePotentiometer: listens to the blue potentiometer and sends the new pot value
//  to the audio processing algorithm as a control parameter
void servicePotentiometer(unsigned long curTime_millis) {
  static unsigned long updatePeriod_millis = 100; //how many milliseconds between updating the potentiometer reading?
  static unsigned long lastUpdate_millis = 0;
  static float prev_val = 0;

   //has enough time passed to update everything?
  if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0; //handle wrap-around of the clock
  if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) { //is it time to update the user interface?

    //read potentiometer
    float val = float(analogRead(POT_PIN)) / 1024.0; //0.0 to 1.0
    val = 0.1 * (float)((int)(10.0 * val + 0.5)); //quantize so that it doesn't chatter...0 to 1.0

    //send the potentiometer value to your algorithm as a control parameter
    //float scaled_val = val / 3.0; scaled_val = scaled_val * scaled_val;
    if (abs(val - prev_val) > 0.05) { //is it different than befor?
      //Serial.print("Sending new value to my algorithms: ");
      //Serial.println(effect1.setUserParameter(val));   //effect2.setUserParameter(val);

      #if USE_TEST_TONE_INPUT==1
        float freq = 700.f+200.f*((val - 0.5)*2.0);  //change tone 700Hz +/- 200 Hz
        Serial.print("Changing tone frequency to = "); Serial.println(freq);
        testSignal.frequency(freq);
      #else
        float vol = 0.70f + 0.15f*((val-0.5)*2.0);  //set volume as 0.70 +/- 0.15
        Serial.print("Setting output volume control to = "); Serial.println(vol);
        audioHardware.volume(vol);
      #endif
    }
    prev_val = val;  //use the value the next time around
    lastUpdate_millis = curTime_millis;
  } // end if
} //end servicePotentiometer();


void printMemoryAndCPU(unsigned long curTime_millis) {
  static unsigned long updatePeriod_millis = 2000; //how many milliseconds between updating gain reading?
  static unsigned long lastUpdate_millis = 0;

  //has enough time passed to update everything?
  if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0; //handle wrap-around of the clock
  if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) { //is it time to update the user interface?
    Serial.print("CPU Cur/Peak: ");
    Serial.print(AudioProcessorUsage());
    Serial.print("%/");
    Serial.print(AudioProcessorUsageMax());
    Serial.print("%,   ");
    Serial.print("MEMORY Int16 Cur/Peak: ");
    Serial.print(AudioMemoryUsage());
    Serial.print("/");
    Serial.print(AudioMemoryUsageMax());
    Serial.print(",   ");
    Serial.print("MEMORY Float32 Cur/Peak: ");
    Serial.print(AudioMemoryUsage_F32());
    Serial.print("/");
    Serial.print(AudioMemoryUsageMax_F32());
    Serial.println();
    lastUpdate_millis = curTime_millis; //we will use this value the next time around.
  }
}

//Here's the function to change the sample rate of the system (via changing the clocking of the I2S bus)
//https://forum.pjrc.com/threads/38753-Discussion-about-a-simple-way-to-change-the-sample-rate?p=121365&viewfull=1#post121365
float setI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } __attribute__((__packed__)) tmclk;

  const int numfreqs = 15;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 24000, 32000, 44100, 44117.64706 , 48000, 88200, 44117.64706 * 2, 96000, 176400, 44117.64706 * 4, 192000};

  #if (F_PLL==16000000)
    const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {48, 125}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
  #elif (F_PLL==72000000)
    const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {32, 375}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
  #elif (F_PLL==96000000)
    const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {8, 125},  {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
  #elif (F_PLL==120000000)
    const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {32, 625},  {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
  #elif (F_PLL==144000000)
    const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {16, 375},  {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
  #elif (F_PLL==180000000)
    const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {37, 1084},  {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
  #elif (F_PLL==192000000)
    const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {4, 125}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
  #elif (F_PLL==216000000)
    const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {32, 1125},  {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
  #elif (F_PLL==240000000)
    const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {16, 625}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
  #endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return (float)(F_PLL / 256 * clkArr[f].mult / clkArr[f].div);
    }
  }
  return 0.0f;
}
