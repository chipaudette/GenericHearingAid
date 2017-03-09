/*
   GenericHearingAid

   Created: Chip Audette, Dec 2016
   Modified by: Daniel Rasetshwane, January 2017
   Modified by: Chip Audette March 2017
   Purpose: Implements Generic Hearing Aid signal processing

   Uses Teensy Audio Adapter.
   Assumes microphones (or whatever) are attached to the LINE IN (stereo)
   Listens  potentiometer mounted to Audio Board to provde a control signal.

   MIT License.  use at your own risk.
*/

//Use test tone as input (set to 1)?  Or, use live audio (set to zero)
#define USE_TEST_TONE_INPUT 0

//These are the includes from the Teensy Audio Library
//#include <Audio.h>      //Teensy Audio Library
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Tympan_Library.h> 

#include "GenericHearingAid_process.h"


const float sample_rate_Hz = 24000.0f ; //24000 or 44117.64706f (or other frequencies in the table in AudioOutputI2S_F32
const int audio_block_samples = 128;  //for this version of GenericHearingAid, do not change from 128
AudioSettings_F32   audio_settings(sample_rate_Hz, audio_block_samples);

//create audio library objects for handling the audio
AudioControlTLV320AIC3206   audioHardware;    //controller for the Teensy Audio Board
AudioSynthWaveformSine_F32    testSignal;          //use to generate test tone as input
AudioInputI2S_F32             i2s_in;          //Digital audio *from* the Audio Board ADC. 
AudioOutputI2S_F32            i2s_out;        //Digital audio *to* the  Audio Board DAC.  
AudioGenericHearingAid_F32    effect1;        //This is your own algorithms

//make the Audio Connections
#if (USE_TEST_TONE_INPUT == 1)
  //use test tone as audio input
  AudioConnection         patchCord1(testSignal, 0, effect1, 0);  //connect the test signal to the hearing aid processing
#else
  //use real audio input (microphones or line-in)
  AudioConnection         patchCord1(i2s_in, 0, effect1, 0);      //connect the Left input to the hearing aid processing
#endif
AudioConnection_F32       patchCord20(effect1, 0, i2s_out, 0);  //connect the hearing aid output to the left audio output
AudioConnection_F32       patchCord21(effect1, 0, i2s_out, 1);  //connect the hearing aid output to the right audio output


//I have a potentiometer on the Teensy Audio Board
#define POT_PIN A1  //potentiometer is tied to this pin

void setupTympanHardware(void) {
  Serial.println("Setting up Tympan Audio Board...");
  audioHardware.enable(); // activate AIC
  
  //choose input
  switch (1) {
    case 1: 
      //choose on-board mics
      audioHardware.inputSelect(TYMPAN_INPUT_ON_BOARD_MIC); // use the on board microphones
      break;
    case 2:
      //choose external input, as a line in
      audioHardware.inputSelect(TYMPAN_INPUT_JACK_AS_LINEIN); //
      break;
    case 3:
      //choose external mic plus the desired bias level
      audioHardware.inputSelect(TYMPAN_INPUT_JACK_AS_MIC); // use the microphone jack
      int myBiasLevel = TYMPAN_MIC_BIAS_2_5;  //choices: TYMPAN_MIC_BIAS_2_5, TYMPAN_MIC_BIAS_1_7, TYMPAN_MIC_BIAS_1_25, TYMPAN_MIC_BIAS_VSUPPLY
      audioHardware.setMicBias(myBiasLevel); // set mic bias to 2.5 // default
      break;
  }
  
  //set volumes
  audioHardware.volume_dB(0.f);  // -63.6 to +24 dB in 0.5dB steps.  uses signed 8-bit
  audioHardware.setInputGain_dB(10.f); // set MICPGA volume, 0-47.5dB in 0.5dB setps

  //setup the potentiometer.  same for Teensy Audio Board as for Tympan
  pinMode(POT_PIN, INPUT); //set the potentiometer's input pin as an INPUT
}


// define the setup() function, the function that is called once when the device is booting
void setup() {
  Serial.begin(115200);   //open the USB serial link to enable debugging messages
  delay(500);             //give the computer's USB serial system a moment to catch up.
  Serial.println("GenericHearingAid: setup()...");

  // Audio connections require memory
  AudioMemory(10);      //allocate Int16 audio data blocks
  AudioMemory_F32(10);  //allocate Float32 audio data blocks

  // Setup the Audio Hardware
  setI2SFreq((int)AUDIO_SAMPLE_RATE); //set the sample rate for the Audio Card (the rest of the library doesn't know, though)
  setupTympanHardware();
  Serial.println("Audio Hardware Setup Complete.");
  
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
      prev_val = val;  //save the value for comparison for the next time around
      val = 1.0 - val;  //reverse direction of potentiometer (error with Tympan PCB)

      #if USE_TEST_TONE_INPUT==1
        float freq = 700.f+200.f*((val - 0.5)*2.0);  //change tone 700Hz +/- 200 Hz
        Serial.print("Changing tone frequency to = "); Serial.println(freq);
        testSignal.frequency(freq);
      #else
        float vol_dB = 0.f + 15.0f*((val-0.5)*2.0);  //set volume as 0dB +/- 15 dB
        Serial.print("Changing output volume frequency to = "); Serial.print(vol_dB);Serial.println(" dB");
        audioHardware.volume_dB(vol_dB);
      #endif
    }
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
