#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

#include "Filter.h"
#include "ADSR.h"
#include "WaveTableOsc.h"
#include "Button.h"

#define overSamp (2)
#define constantRatioLimit (99999)

#define SWITCHCOUNT 40 // how many switches are we reading
#define KEYCOUNT 37   // how many buttons are we reading

#define CHANGEAFTERREADS 2 // how many times we need to read the same value before we recognise a state change

double sound = 0.0; //Initialise the audio output to 0

//Variables for the first bank of 4 potentiometers
//(analog inputs) used for the amplitude envelope
int gPot1Input; 
int gPot2Input;
int gPot3Input;
int gPot4Input;

//Variables for the first bank of 4 potentiometers
//(analog inputs) used for the filter envelope
int gPot5Input;
int gPot6Input;
int gPot7Input;
int gPot8Input;

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// pin layout
int latchPin = P8_09;
int dataPin = P8_07;
int clockPin = P8_11;

bool states[SWITCHCOUNT]; // the current recognised state
int diffCount[SWITCHCOUNT]; // count the number of times we see a state different to what we recognise
int nowPlaying[SWITCHCOUNT]; // the note we are playing right now - when the button is released this is what we have to switch off in midi
int usingMap = 0; // button map index, 0 to 2
int upBy = 0; // up by 0 to 11

//State index used keep track of the state at which
//The shift register button scanning process is
int stateIndex=0;

Button* buttons[SWITCHCOUNT]; //Create SWITCHCOUNT no. of button class

// Different button map layouts for 
// different hermonic table options
unsigned int buttonNoteMapping[3][KEYCOUNT] = {
  {

    //Park Table Layout

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

    // Gerhard Table Layout

        27,31,35,39,
      26,30,34,38,42,
     25,29,33,37,41,45,
    24,28,32,36,40,44,48,
     27,31,35,39,43,47,
      30,34,38,42,46,
        33,37,41,45
}
};

//Defines for  the button numbers assigned to the
// upbyone and harmonictable functions
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

//Assignment of the analog inputs for the
//four potentiometers used for the amplitude ADSR
       gPot1Input = 0;
       gPot2Input = 1;
       gPot3Input = 2;
       gPot4Input = 3;

//Assignment of the analog inputs for the
//four potentiometers used for the filter ADSR
       gPot5Input = 4;
       gPot6Input = 5;
       gPot7Input = 6;
       gPot8Input = 7;

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
      rt_printf("Wavetables Created \n");

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

// Parameter bank select state machine
// to be implemented to allow multiple parameters ot be controlled
// using the 8 available potentiometers

// buttonState=digitalReadFrame(context, 0, P9_12); //read the value of the button

//   if (buttonState != lastButtonState) {
//     if(buttonState==1){

//       if(buttonPushCounter == 0)
//       {
//       rt_printf("Oscillators %d \n",buttonPushCounter);
//       buttonPushCounter++;
//       }
//       else if(buttonPushCounter == 1)
//       {
//       rt_printf("Envelopes %d \n",buttonPushCounter);
//       buttonPushCounter++;
//       }
//       else if(buttonPushCounter == 2)
//       {
//       rt_printf("Filter %d \n",buttonPushCounter);
//       buttonPushCounter = 0;
//       }
//     } 
//     lastButtonState=buttonState;
//   }

  //Once per render read in the values of the 8 potentiometers
  //Remap the values to ranges appropriate for the assigned parameter
  double a1 = map(analogReadFrame(context, 0, gPot1Input), 0.0, 0.82, 0.0001, 1.0);
  double d1 = map(analogReadFrame(context, 0, gPot2Input), 0.0, 0.82, 0.0001, 1.0);
  double s1 = map(analogReadFrame(context, 0, gPot3Input), 0.0, 0.82, 0.0, 1.0);
  double r1 = map(analogReadFrame(context, 0, gPot4Input), 0.0, 0.82, 0.0001, 1.0);

  double a2 = map(analogReadFrame(context, 0, gPot5Input), 0.0, 1.0, 0.0001, 1.0);
  double d2 = map(analogReadFrame(context, 0, gPot6Input), 0.0, 0.82, 0.0001, 1.0);
  double s2 = map(analogReadFrame(context, 0, gPot7Input), 0.0, 0.82, 1.0, 1.0);
  double r2 = map(analogReadFrame(context, 0, gPot8Input), 0.0 , 0.82, 0.0001, 1.0);

  // Assign the read values of the potiometers to each
  // amplitude envelpe and filter envelope
  // assigned to each button
  for(int j=0; j<SWITCHCOUNT; j++){
    buttons[j]->setAmpEnv(a1, d1, s1, r1);
    buttons[j]->setFilterEnv(a2, d2, s2, r2);
  }

  // Calls the read button function to determine
  // which buttons are currently being pressed
  readButtons(context);

  // for each analogue frame, make some sound
  for (unsigned int m = 0; m < context->audioFrames; m++) {
    sound=0.0;

    for(int j=0; j<SWITCHCOUNT; j++){
      sound+=buttons[j]->getOutput();
    }

    // write the sound value to left and right
    for (unsigned int channel = 0; channel < context->audioChannels; channel++)
      context->audioOut[m * context->audioChannels + channel] = sound;
  }
}

void readButtons(BeagleRTContext *context){

  int indexClock = stateIndex - 1;
  int indexRead = stateIndex - 3;

  if (stateIndex==0){
   //set it to 1 to collect parallel data
    digitalWriteFrameOnce(context, 0, latchPin, GPIO_HIGH);
    digitalWriteFrameOnce(context, 2, latchPin, GPIO_LOW); 
  }

  //set it to 1 to collect parallel data, wait
  if ((indexClock>=0) && (indexClock<SWITCHCOUNT)) { 
    digitalWriteFrameOnce(context, 0, clockPin, GPIO_LOW);
    digitalWriteFrameOnce(context, 2, clockPin, GPIO_HIGH);
  }

  if((indexRead>=0) && (indexRead<SWITCHCOUNT)) {
    int buttonReadIndex = indexRead;
    bool newRead = digitalReadFrame(context, 1, dataPin);

      // if the state is different to what we have stored
    if (states[buttonReadIndex] != newRead) {
      // count the number of times we read a different state
      diffCount[buttonReadIndex]++;
      // if we have seen enough, switch the state
      if (diffCount[buttonReadIndex] >= CHANGEAFTERREADS) {
        diffCount[buttonReadIndex]=0;
        states[buttonReadIndex] = newRead;
        changeDetected(buttonReadIndex, newRead, context->audioSampleRate);
      }
    } else {
      diffCount[buttonReadIndex] = 0;
    }
  }
  
  //move on to the next state of the shift register scanning process
  stateIndex++;
  //if all the buttons have been scanned return to the initial state of the shift register scanning process
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
          // rt_printf("Switch Index %d \n", switchIndex);

      }
    }
  } else {
      // button release
    switch(switchIndex) {
      case SWITCHINDEX_HARMONICTABLE:
      case SWITCHINDEX_UPBYONE:
      break;
      default:
          // whatever we played we now release
      unsigned int note= nowPlaying[switchIndex];
      nowPlaying[switchIndex] = 0;
      buttons[switchIndex]->buttonReleased();
    }      
  }
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}



