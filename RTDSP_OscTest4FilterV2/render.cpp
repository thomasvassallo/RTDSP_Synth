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
#include "WaveTableOsc.h"

#define myFloat double
#define overSamp (2)
#define constantRatioLimit (99999)


float sound = 0.0;
double filterCuttoff = 200;

double oscFreq = 110;

double lfoPhase;
double inverseSampleRate;
int depth;
double frequency;
float ph;
float lfoMod;

int gPotInput;
int gAudioFramesPerAnalogFrame;
float fc;


Filter filter1;
ADSR *env;
ADSR *filterEnv;
WaveTableOsc osc1;

int lastButtonStatus = 0;

//Function Prototypes
float sawTooth(int samplePosition);
float makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq);
float lfo(float phase, int waveform);
void fft(int N, myFloat *ar, myFloat *ai);
void setSawtoothOsc(WaveTableOsc *osc, float baseFreq, int sampleRate);
void defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai);



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

	env = new ADSR();
	filterEnv = new ADSR();

	pinModeFrame(context, 0, P9_12, INPUT);
	pinModeFrame(context, 0, P9_14, OUTPUT);

	gPotInput=1;
	gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;

	setSawtoothOsc(&osc1, 20, context->audioSampleRate);

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
		if(buttonStatus){
		osc1.setFrequency(oscFreq/context->audioSampleRate);
		}

		env->gate(buttonStatus != 0);
		filterEnv->gate(buttonStatus != 0);

		// remember what we set
		lastButtonStatus = buttonStatus;
	}

	// for each analogue frame, make some sound
	for (unsigned int m = 0; m < context->audioFrames; m++) {

			if(!(m % gAudioFramesPerAnalogFrame)) {
	  		fc = map(analogReadFrame(context, m/gAudioFramesPerAnalogFrame, gPotInput), 0, 0.82, 0, 0.8);
	  		}

		ph = lfoPhase;
		lfoMod=depth*lfo(ph, 3);


		sound=osc1.getOutput() * env->process();
		osc1.updatePhase();		

		// filter1.setCutoff(filterCuttoff*lfoMod);
		// filter1.setCutoffMod(filterEnv->process());
		// // filter1.setResonance(fc);
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

float makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq) {
    fft(len, ar, ai);
    
    if (scale == 0.0) {
        // calc normal
        myFloat max = 0;
        for (int idx = 0; idx < len; idx++) {
            myFloat temp = fabs(ai[idx]);
            if (max < temp)
                max = temp;
        }
        scale = 1.0 / max * .999;        
    }
    
    // normalize
    float wave[len];
    for (int idx = 0; idx < len; idx++)
        wave[idx] = ai[idx] * scale;
        
    if (osc->addWaveTable(len, wave, topFreq))
        scale = 0.0;
    
    return scale;
}

void fft(int N, myFloat *ar, myFloat *ai)
/*
 in-place complex fft
 
 After Cooley, Lewis, and Welch; from Rabiner & Gold (1975)
 
 program adapted from FORTRAN 
 by K. Steiglitz  (ken@princeton.edu)
 Computer Science Dept. 
 Princeton University 08544          */
{    
    int i, j, k, L;            /* indexes */
    int M, TEMP, LE, LE1, ip;  /* M = log N */
    int NV2, NM1;
    myFloat t;               /* temp */
    myFloat Ur, Ui, Wr, Wi, Tr, Ti;
    myFloat Ur_old;
    
    // if ((N > 1) && !(N & (N - 1)))   // make sure we have a power of 2
    
    NV2 = N >> 1;
    NM1 = N - 1;
    TEMP = N; /* get M = log N */
    M = 0;
    while (TEMP >>= 1) ++M;
    
    /* shuffle */
    j = 1;
    for (i = 1; i <= NM1; i++) {
        if(i<j) {             /* swap a[i] and a[j] */
            t = ar[j-1];     
            ar[j-1] = ar[i-1];
            ar[i-1] = t;
            t = ai[j-1];
            ai[j-1] = ai[i-1];
            ai[i-1] = t;
        }
        
        k = NV2;             /* bit-reversed counter */
        while(k < j) {
            j -= k;
            k /= 2;
        }
        
        j += k;
    }
    
    LE = 1.;
    for (L = 1; L <= M; L++) {            // stage L
        LE1 = LE;                         // (LE1 = LE/2) 
        LE *= 2;                          // (LE = 2^L)
        Ur = 1.0;
        Ui = 0.; 
        Wr = cos(M_PI/(float)LE1);
        Wi = -sin(M_PI/(float)LE1); // Cooley, Lewis, and Welch have "+" here
        for (j = 1; j <= LE1; j++) {
            for (i = j; i <= N; i += LE) { // butterfly
                ip = i+LE1;
                Tr = ar[ip-1] * Ur - ai[ip-1] * Ui;
                Ti = ar[ip-1] * Ui + ai[ip-1] * Ur;
                ar[ip-1] = ar[i-1] - Tr;
                ai[ip-1] = ai[i-1] - Ti;
                ar[i-1]  = ar[i-1] + Tr;
                ai[i-1]  = ai[i-1] + Ti;
            }
            Ur_old = Ur;
            Ur = Ur_old * Wr - Ui * Wi;
            Ui = Ur_old * Wi + Ui * Wr;
        }
    }
}

void setSawtoothOsc(WaveTableOsc *osc, float baseFreq, int sampleRate) {    
    // calc number of harmonics where the highest harmonic baseFreq and lowest alias an octave higher would meet
    int maxHarms = sampleRate / (3.0 * baseFreq) + 0.5;

    // round up to nearest power of two
    unsigned int v = maxHarms;
    v--;            // so we don't go up if already a power of 2
    v |= v >> 1;    // roll the highest bit into all lower bits...
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;            // and increment to power of 2
    int tableLen = v * 2 * overSamp;  // double for the sample rate, then oversampling

    myFloat ar[tableLen], ai[tableLen];   // for ifft

    double topFreq = baseFreq * 2.0 / sampleRate;
    myFloat scale = 0.0;
    for (; maxHarms >= 1; maxHarms >>= 1) {
        defineSawtooth(tableLen, maxHarms, ar, ai);
        scale = makeWaveTable(osc, tableLen, ar, ai, scale, topFreq);
        topFreq *= 2;
        if (tableLen > constantRatioLimit) // variable table size (constant oversampling but with minimum table size)
            tableLen >>= 1;
    }
}

//
// defineSawtooth
//
// prepares sawtooth harmonics for ifft
//
void defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai) {
    if (numHarmonics > (len >> 1))
        numHarmonics = (len >> 1);
    
    // clear
    for (int idx = 0; idx < len; idx++) {
        ai[idx] = 0;
        ar[idx] = 0;
    }

    // sawtooth
    for (int idx = 1, jdx = len - 1; idx <= numHarmonics; idx++, jdx--) {
        myFloat temp = -1.0 / idx;
        ar[idx] = -temp;
        ar[jdx] = temp;
    }
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BeagleRTContext *context, void *userData)
{
	// Nothing to do here
}
