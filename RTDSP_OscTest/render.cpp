    /*
 *
 * Andrew McPherson and Victor Zappi
 * Queen Mary, University of London
 */

#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

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
		pinModeFrame(context, 0, P9_12, INPUT);
		pinModeFrame(context, 0, P9_14, OUTPUT);
	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numAnalogFrames
// will be 0.

/* basic_button
* - connect an LED in series with a 470ohm resistor between P8_07 and ground.
* - connect a 1k resistor to P9_03 (+3.3V), 
* - connect the other end of the resistor to both a button and P8_08
* - connect the other end of the button to ground.
* The program will read the button and make the LED blink when the button is pressed.
*/

void render(BeagleRTContext *context, void *userData)
{
	// read the button - only once for each render (buttons don't change very often)
	int buttonStatus=digitalReadFrame(context, 0, P9_12); //read the value of the button
	buttonStatus != buttonStatus;
	
	// for each digital frame, copy the button status to the LED
	for(unsigned int n=0; n<context->digitalFrames; n++) {		
		digitalWriteFrame(context, n, P9_14, buttonStatus); //write the status to the LED
		    rt_printf("On/Off %d \n", buttonStatus);
	}
	
	// for each analogue frame, amke some sound
	for(unsigned int m=0; m<context->audioFrames; m++) {
		
		// super-simple audio maker
		// sawtooth. The value starts at zero and goes up with each sample in the render
		float sound = ((float)m) / (float)context->audioFrames;		
		
		// if the button is not pressed, mute the sound
		if (0==buttonStatus) {
			sound = 0.0f;
		}
		// write the sound value to left and right

		for(unsigned int channel = 0; channel < context->audioChannels; channel++)
      	context->audioOut[m * context->audioChannels + channel] = sound;
	}
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}
