#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

#include "Filter.h"
#include "ADSR.h"
#include "Oscillator.h"
#include "WaveTableOsc.h"
#include "Button.h"

#define overSamp (2)
#define constantRatioLimit (99999)

#define SWITCHCOUNT 40 // how many switches are we reading
#define KEYCOUNT 37   // how many buttons are we reading

#define PROPAGATIONDELAY 20 // standard delay after we send a signal down the line
#define CHANGEAFTERREADS 8 // how many times we need to read the same value before we recognise a state change
#define MIDIOVERUSB (true) // if this is true then we output midi commands over usb instead of debug over usb and midi over USART1


double sound = 0.0;
double a = 440; // a is 440 hz...

double lfoPhase;
double inverseSampleRate;
int depth;
double frequency;
float ph;
float lfoMod;

int gPotInput;
int gAudioFramesPerAnalogFrame;
float fc;

// pin layout
int latchPin = P8_07;
int dataPin = P8_09;
int clockPin = P8_11;

bool states[SWITCHCOUNT]; // the current recognised state
int diffCount[SWITCHCOUNT]; // count the number of times we see a state different to what we recognise
int nowPlaying[SWITCHCOUNT]; // the note we are playing right now - when the button is released this is what we have to switch off in midi
int usingMap = 0; // button map index, 0 to 2
int upBy = 0; // up by 0 to 11

int stateIndex=0;

Button* buttons[SWITCHCOUNT]; //Create SWITCHCOUNT no. of button class

// Different button map layouts
// button map
unsigned int buttonNoteMapping[3][KEYCOUNT] = {
  {

    //Park Layout
    
         30,35,40,45,
       27,32,37,42,47,
      24,29,34,39,44,49,
    21,26,31,36,41,46,51,
      23,28,33,38,43,48,
       25,30,35,40,45,
         27,32,37,42
  },
  {

    //Harmonic Table Layout
    
         45,49,53,57,
       48,42,46,50,54,
      31,35,39,43,47,51,
    24,28,32,36,40,44,48,
      21,25,29,33,37,41,
        18,22,26,30,34,
          15,19,23,27
  },
  {      

    // Gerhard
    
        27,31,35,39,
       26,30,34,38,42,
      25,29,33,37,41,45,
    24,28,32,36,40,44,48,
      27,31,35,39,43,47,
       30,34,38,42,46,
         33,37,41,45
  }
};

#define SWITCHINDEX_HARMONICTABLE 33
#define SWITCHINDEX_UPBYONE 34

// each button in this array order returns this number
int buttonMap[] = {
       3, 2, 1, 0,
     5, 6, 7, 8, 4,
    12,13,14,15,9,10,
  11,16,20,21, 22,23,17,
    30,29,28,24,19,18, 
      25,26,27,32,31,
        39,38,37,36,

        33, 34, 35 // extra buttons
};

// create array for reverse lookup
int buttonIndexMap[SWITCHCOUNT];

//Function Prototypes
float sawTooth(int samplePosition);
float makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq);
float lfo(float phase, int waveform);
void fft(int N, myFloat *ar, myFloat *ai);
void setSawtoothOsc(WaveTableOsc *osc, float baseFreq, int sampleRate);
void defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai);
void readButtons(BeagleRTContext *context);
void changeDetected(int switchIndex, bool newState, int sampleRate);
void raiseByOne();
void changeMode();


// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool setup(BeagleRTContext *context, void *userData)
{

int sampleRate = context->audioSampleRate;

  pinModeFrame(context, 0, P9_12, INPUT);
  pinModeFrame(context, 0, P9_14, OUTPUT);

  gPotInput=1;
  gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
  

  lfoPhase = 0.0;
    inverseSampleRate = 1.0/context->audioSampleRate;
    depth = 1;
    frequency = 200;
    lfoMod=0;

  // shiftin lines
        pinModeFrame(context, 0, latchPin, OUTPUT);
        pinModeFrame(context, 0, clockPin, OUTPUT);
        pinModeFrame(context, 0, dataPin, INPUT);

   // set everything to neutral
        for(int i=0; i<SWITCHCOUNT; i++) {
          states[i] = false;
          diffCount[i] = 0;
          nowPlaying[i] = 0;
          buttonIndexMap[i] = -1;

          buttons[i] = new Button(context->audioSampleRate);
        }

      // create reverse button lookup
        for (int j=0; j<KEYCOUNT; j++) {
          buttonIndexMap[buttonMap[j]] = j;
        }

        return true;
      }

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numAnalogFrames
// will be 0.

/* basic_blink
* Connect an LED in series with a 470ohm resistor between P8_07 and ground.
* The LED will blink every @interval seconds.
*/

void render(BeagleRTContext *context, void *userData)
{

readButtons(context);

  
  // for each analogue frame, make some sound
  for (unsigned int m = 0; m < context->audioFrames; m++) {

      // if(!(m % gAudioFramesPerAnalogFrame)) {
      //   fc = map(analogReadFrame(context, m/gAudioFramesPerAnalogFrame, gPotInput), 0, 0.82, 0, 0.8);
      //   }

    ph = lfoPhase;
    lfoMod=depth*lfo(ph, 3);


    sound=0.0;
    
    // filter1.setCutoff(filterCuttoff*lfoMod);
    // filter1.setCutoffMod(filterEnv->process());
    // sound = filter1.process(sound);

    ph += frequency*inverseSampleRate;
        if(ph >= 1.0)
            ph -= 1.0;

    // write the sound value to left and right
    for (unsigned int channel = 0; channel < context->audioChannels; channel++)
      context->audioOut[m * context->audioChannels + channel] = sound;
  }

  lfoPhase = ph;

}

void readButtons(BeagleRTContext *context){

  int indexClock = stateIndex - 1;
  int indexRead = stateIndex - 3;

  if (stateIndex==0){
   //set it to 1 to collect parallel data
    digitalWriteFrame(context, 0, latchPin, GPIO_HIGH);
    digitalWriteFrame(context, 2, latchPin, GPIO_LOW); 
  }

  if ((indexClock>=0) && (indexClock<SWITCHCOUNT)) { 
      digitalWriteFrame(context, 0, clockPin, GPIO_LOW);
      digitalWriteFrame(context, 10, clockPin, GPIO_HIGH);
  }

  if((indexRead>=0) && (indexRead<SWITCHCOUNT)) {
      int buttonReadIndex = indexRead;
      bool newRead = digitalReadFrame(context, 5, dataPin);

      if (states[buttonReadIndex] != newRead) {
      // count the number of times we read a different state
        diffCount[buttonReadIndex]++;
      // if we have seen enough, switch the state
        if (diffCount[buttonReadIndex] >= CHANGEAFTERREADS) {
          rt_printf("%d \n", buttonReadIndex);
          diffCount[buttonReadIndex]=0;
          states[buttonReadIndex] = newRead;
          changeDetected(buttonReadIndex, newRead, context->audioSampleRate);
        }
      } else {
        diffCount[buttonReadIndex] = 0;
      }
    }
  
  stateIndex++;
  if (indexRead==SWITCHCOUNT) {
    stateIndex = 0;
  }
}

// switch button mode
 void changeMode() {
  usingMap = (usingMap + 1) % 3;
    rt_printf("Using Map %d \n", usingMap);
}

// increase the 'up by one' count
void raiseByOne() {
  upBy = (upBy + 1) % 12;
    rt_printf("Up By One: %d \n", upBy);
}

// when a button changes state
void changeDetected(int switchIndex, bool newState, int sampleRate) {

    // if it is  a button press
  if (newState == true) {
    switch(switchIndex) {
      case SWITCHINDEX_HARMONICTABLE:
      changeMode();
      break;
      case SWITCHINDEX_UPBYONE:
      raiseByOne();
      break;
      default:
          // midi out
      int buttonIndex = buttonIndexMap[switchIndex];
          // if we have a mapping for this key
      if (buttonIndex>=0) {
        unsigned int note = buttonNoteMapping[usingMap][buttonIndex];
        note += upBy;
            // store what we played so we know what to switch off
        nowPlaying[switchIndex] = note;
        buttons[switchIndex]->buttonPressed(note, sampleRate);
          rt_printf("Switch Index %d \n", switchIndex);

      }
    }
  } else {
      // button release
    switch(switchIndex) {
      case SWITCHINDEX_HARMONICTABLE:
      case SWITCHINDEX_UPBYONE:
      break;
      default:
          // midi out
          // whatever we played we now release
      unsigned int note= nowPlaying[switchIndex];
      nowPlaying[switchIndex] = 0;
      buttons[switchIndex]->buttonReleased();
    }      
  }
}

float lfo(float phase, int waveform)
{
    switch(waveform)
    {
        case 1:
            if(phase < 0.25f)
                return 0.5f + 2.0f*phase;
            else if(phase < 0.75f)
                return 1.0f - 2.0f*(phase - 0.25f);
            else
                return 2.0f*(phase-0.75f);
        case 2:
            if(phase < 0.5f)
                return 1.0f;
            else
                return 0.0f;
        case 3:
            if(phase < 0.48f)
                return 1.0f;
            else if(phase < 0.5f)
                return 1.0f - 50.0f*(phase - 0.48f);
            else if(phase < 0.98f)
                return 0.0f;
            else
                return 50.0f*(phase - 0.98f);
        case 4:
        default:
            return 0.5f + 0.5f*sinf(2.0 * M_PI * phase);
    }
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}



