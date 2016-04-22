#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

#define SWITCHCOUNT 40 // how many switches are we reading
#define KEYCOUNT 37   // how many buttons are we reading

#define PROPAGATIONDELAY 20 // standard delay after we send a signal down the line
#define CHANGEAFTERREADS 8 // how many times we need to read the same value before we recognise a state change
#define MIDIOVERUSB (true) // if this is true then we output midi commands over usb instead of debug over usb and midi over USART1

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
     8, 4, 5, 6, 7,
    9,10,11,14,13,12,
  15,17,18,19,20,21,22,
    27,26,25,24,23,16, 
      28,29,30,31,32,
        39,38,37,36,

        33, 34, 35 // extra buttons
};

// create array for reverse lookup
      int buttonIndexMap[SWITCHCOUNT];


void readButtons(BeagleRTContext *context);


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
        }

      // create reverse button lookup
        for (int j=0; j<KEYCOUNT; j++) {
          buttonIndexMap[buttonMap[j]] = j;
        }

        return true;
      }


// switch button mode
 void changeMode() {
  usingMap = (usingMap + 1) % 3;
  if (!MIDIOVERUSB) {
    rt_printf("Using Map %d \n", usingMap);
  }
}

// increase the 'up by one' count
void raiseByOne() {
  upBy = (upBy + 1) % 12;
  if (!MIDIOVERUSB) {
    rt_printf("Up By One: %d \n", upBy);
  }
}

// when a button changes state
void changeDetected(int switchIndex, bool newState) {

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
        rt_printf("Switch %d Note %d :ON \n", switchIndex, note);

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
      rt_printf("Switch %d Note %d :OFF \n", switchIndex, note);

      
    }      
  }
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

}

void readButtons(BeagleRTContext *context){

  if (stateIndex==0){
   //set it to 1 to collect parallel data
    digitalWriteFrame(context, 0, latchPin, GPIO_HIGH);
    digitalWriteFrame(context, 1, latchPin, GPIO_LOW); 
  }

  else {
    int digitalFrames=context->digitalFrames;
    int clocksPerRender=digitalFrames/4;

    for(int i=0; i<clocksPerRender; i++){
      digitalWriteFrame(context, i*4, clockPin, GPIO_HIGH);
      digitalWriteFrame(context, (i*4)+1, clockPin, GPIO_LOW);
    }

    if(stateIndex>1){
      for(int i=0; i<clocksPerRender;i++){
        int buttonReadIndex = ((stateIndex-2)*clocksPerRender)+i;

        bool newRead = digitalReadFrame(context, (i*4)+2, dataPin);

        if (states[buttonReadIndex] != newRead) {
        // count the number of times we read a different state
          diffCount[buttonReadIndex]++;
        // if we have seen enough, switch the state
          if (diffCount[buttonReadIndex] >= CHANGEAFTERREADS) {
            diffCount[buttonReadIndex]=0;
            states[buttonReadIndex] = newRead;
            changeDetected(buttonReadIndex, newRead);
          }
        } else {
          diffCount[buttonReadIndex] = 0;
        }

        if(buttonReadIndex>=SWITCHCOUNT-1){
          stateIndex=0;
          return;
        }
      }
    }
  }

  stateIndex++;
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}



