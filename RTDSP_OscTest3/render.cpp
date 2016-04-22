    /*
 *
 * Andrew McPherson and Victor Zappi
 * Queen Mary, University of London
 */

#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

#include "Filter.h"

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

Filter filter1;


bool setup(BeagleRTContext *context, void *userData)
{
		pinModeFrame(context, 0, P9_12, INPUT);
		pinModeFrame(context, 0, P9_14, OUTPUT);

		filter1.calculateCoefficients(0, 200, context->audioSampleRate, 1, 0, 0);

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


float sawTooth(int samplePosition) {
	// 5khz tone = 5000 sawteeth in a second of 44100 samples
	// this will be approximate because it is rounded to the nearest integer
	int perSecond = 44100 / 440;
	// for this sample position, how far up the sawn tooth are we?
	int toothPosition = samplePosition % perSecond;
	// map that into the space 0.00 to 1.00
	float sound = (float)toothPosition / (float)perSecond;
	return sound;	
}

void render(BeagleRTContext *context, void *userData)
{
	// read the button - only once for each render (buttons don't change very often)
	int buttonStatus=digitalReadFrame(context, 0, P9_12); //read the value of the button
	
	// for each digital frame, copy the button status to the LED
	for(unsigned int n=0; n<context->digitalFrames; n++) {		
		digitalWriteFrame(context, n, P9_14, buttonStatus); //write the status to the LED
		    rt_printf("On/Off %d \n", buttonStatus);
	}
	
	// for each analogue frame, amke some sound
	for(unsigned int m=0; m<context->audioFrames; m++) {
		
		// get a sound value for this sample position.
		// in reality, rather than use context->audioSampleCount you would count the samples since the user 
		// pressed the button! (but we're being lazy here)
		float sound = sawTooth(context->audioSampleCount + m);
		filter1.calculateCoefficients(0, 200, context->audioSampleRate, 1, 0, 0);
    	sound=filter1.processSamples(sound);
		
    	//rt_printf("coeffs: %f %f %f\n",filter1.b0a0,filter1.b1a0,filter1.b2a0);

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
