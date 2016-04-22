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
#include "ADSR.h"
#include "Oscillator.h"
// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

float sound = 0.0;
double filterCuttoff = 200;

double oscFreq = 440;

double lfoPhase;
double inverseSampleRate;
int depth;
double frequency;
float ph;
float lfoMod;



Filter filter1;
ADSR *env;
ADSR *filterEnv;

int lastButtonStatus = 0;

bool setup(BeagleRTContext *context, void *userData)
{
	env = new ADSR();
	filterEnv = new ADSR();

	pinModeFrame(context, 0, P9_12, INPUT);
	pinModeFrame(context, 0, P9_14, OUTPUT);

	filter1.calculateCoefficients(0, 200, context->audioSampleRate, 1, 0, 0);

	filterEnv->setTargetRatioA(0.1);
	filterEnv->setTargetRatioDR(0.1);

	filterEnv->setAttackRate(.1 * context->audioSampleRate);  // .1 second
	filterEnv->setDecayRate(.3 * context->audioSampleRate);
	filterEnv->setReleaseRate(.5 * context->audioSampleRate);
	filterEnv->setSustainLevel(.8);

	env->setTargetRatioA(0.1);
	env->setTargetRatioDR(0.1);

	env->setAttackRate(.01 * context->audioSampleRate);  // .1 second
	env->setDecayRate(.3 * context->audioSampleRate);
	env->setReleaseRate(.5 * context->audioSampleRate);
	env->setSustainLevel(.8);

	lfoPhase = 0.0;
    inverseSampleRate = 1.0/context->audioSampleRate;
    depth = 1;
    frequency = 200;
    lfoMod=0;

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
	int perSecond = 44100 / oscFreq;
	// for this sample position, how far up the sawn tooth are we?
	int toothPosition = samplePosition % perSecond;
	// map that into the space 0.00 to 1.00
	float sound = (float)toothPosition / (float)perSecond;
	return sound;
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

void render(BeagleRTContext *context, void *userData)
{
	// read the button - only once for each render (buttons don't change very often)
	// read the button status in the most recent digital sample, which is the last one of course
	int buttonStatus = digitalReadFrame(context, context->digitalFrames-1, P9_12); 

	// for each digital frame, copy the button status to the LED
	for (unsigned int n = 0; n < context->digitalFrames; n++) {
		digitalWriteFrame(context, n, P9_14, buttonStatus); //write the status to the LED
		//rt_printf("On/Off %d \n", buttonStatus);														
	}

	// if the button state changed since the last render
	if (lastButtonStatus != buttonStatus) {
		// set gate to false if buttonState is now 0, to true otherwise
		env->gate(buttonStatus != 0);
		filterEnv->gate(buttonStatus != 0);

		// remember what we set
		lastButtonStatus = buttonStatus;
	}

	// for each analogue frame, make some sound
	for (unsigned int m = 0; m < context->audioFrames; m++) {

		ph = lfoPhase;
		lfoMod=depth*lfo(ph, 3);
		// get a sound value for this sample position.
		// in reality, rather than use context->audioSampleCount you would count the samples since the user 
		// pressed the button! (but we're being lazy here)			
		sound = sawTooth(context->audioSampleCount + m) * env->process();
		filter1.calculateCoefficients(0, filterCuttoff * (filterEnv->process() * 10)/* *lfoMod Modulation using the LFO, filter unstable at high freqs*/, context->audioSampleRate, 1, 0, 0); //Calculate the coefficients
		sound = filter1.processSamples(sound);
		//rt_printf("Sound %d \n", sound);

		ph += frequency*inverseSampleRate;
        if(ph >= 1.0)
            ph -= 1.0;

		// write the sound value to left and right
		for (unsigned int channel = 0; channel < context->audioChannels; channel++)
			context->audioOut[m * context->audioChannels + channel] = sound;
	}

	lfoPhase = ph;

}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}
