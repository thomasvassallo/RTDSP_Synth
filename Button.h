#ifndef Button_h
#define Button_h

#include <BeagleRT.h>
#include <Utilities.h>
#include <cmath>
#include <rtdk.h>

#include "ADSR.h"
#include "Filter.h"
#include "WaveTableOsc.h"

#define myFloat double

#define overSamp (2)
#define constantRatioLimit (99999)


class Button {
public:
    Button(int);
    ~Button(void);

    double getOutput();
    void setNoteOn(int);
    int isNoteOn();
    void buttonReleased();
    void buttonPressed(int , int);
    void setFilterEnv(float, float, float, float);
    void setAmpEnv(float, float, float, float);


private:
WaveTableOsc osc1;
WaveTableOsc osc2;
WaveTableOsc osc3;

Filter filter;

ADSR env;
ADSR filterEnv;

int sampleRate;

int filterCuttoff;
int noteOn;
float attack, decay, sustain, release;
float fAttack, fDecay, fSustain, fRelease;

double detuneOsc1, detuneOsc2, detuneOsc3;


    void defineSawtooth(int , int numHarmonics, myFloat *ar, myFloat *ai);
    void setSawtoothOsc(WaveTableOsc *osc, float baseFreq, int sampleRate);
    void fft(int N, myFloat *ar, myFloat *ai);
    float makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq);

};



#endif
