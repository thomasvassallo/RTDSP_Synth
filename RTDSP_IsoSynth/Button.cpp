
#include "Button.h"
#include <math.h>


Button::Button(int sampleRate_) {

    rt_printf("Button Class \n");

    sampleRate=sampleRate_;

    filterCuttoff = 50;

    osc1.setFrequency(440/sampleRate);
    osc2.setFrequency(440/sampleRate);
    osc3.setFrequency(440/sampleRate);  

    filterEnv.setAttackRate(.1 * sampleRate);  // .1 second
    filterEnv.setDecayRate(.3 * sampleRate);
    filterEnv.setReleaseRate(.5 * sampleRate);
    filterEnv.setSustainLevel(.8);

    env.setAttackRate(.01 * sampleRate);  // .1 second
    env.setDecayRate(.3 * sampleRate);
    env.setReleaseRate(.5 * sampleRate);
    env.setSustainLevel(.8);

    setSawtoothOsc(&osc1, 20, sampleRate);
    setSawtoothOsc(&osc2, 20, sampleRate);
    setSawtoothOsc(&osc3, 20, sampleRate);

}

void Button::setAmpEnv(float a1, float d1, float s1, float r1){

    attack=a1;
    decay=d1;
    sustain=s1;
    release=r1;

     if(env.getState()!=0){    

    env.setAttackRate(attack * sampleRate);  // .1 second
    env.setDecayRate(decay * sampleRate);
    env.setReleaseRate(release * sampleRate);
    env.setSustainLevel(sustain);
    }
}

void Button::setFilterEnv(float a2, float d2, float s2, float r2){

    filterEnv.setAttackRate(a2 * sampleRate);  // .1 second
    filterEnv.setDecayRate(d2 * sampleRate);
    filterEnv.setReleaseRate(r2 * sampleRate);
    filterEnv.setSustainLevel(s2);
}

 double Button::getOutput(){

    if(env.getState()==0){

        return 0.0;
    }

    double sound=env.process()*(osc1.getOutput() + osc2.getOutput() + osc3.getOutput()/5);

    osc1.updatePhase();
    osc2.updatePhase(); 
    osc3.updatePhase();    

    filter.setCutoff(filterCuttoff);
    filter.setCutoffMod(filterEnv.process());
    sound=filter.process(sound);
    return sound;
     }

    void Button::setNoteOn(int pressed){
        noteOn=pressed;
    }
    int Button::isNoteOn(){
        return noteOn;
    }

void Button::buttonPressed(int note, int sampleRate) {

  double oscillatorFrequency=(440.0 / 32.0) * pow(2.0,(((note+12) - 9.0) / 12.0));;

    osc1.setFrequency((oscillatorFrequency+0.20)/sampleRate);
    osc2.setFrequency((oscillatorFrequency+0.30)/sampleRate);
    osc3.setFrequency(oscillatorFrequency/sampleRate);  

    env.setAttackRate(attack * sampleRate);  // .1 second
    env.setDecayRate(decay * sampleRate);
    env.setReleaseRate(release * sampleRate);
    env.setSustainLevel(sustain);

    env.gate(true);
    filterEnv.gate(true);
    
    rt_printf("Frequency %f Note %d :ON \n", oscillatorFrequency, note);
  
}

void Button::buttonReleased() {

  env.gate(false);
  filterEnv.gate(false);
  
}

float Button::makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq) {
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

void Button::fft(int N, myFloat *ar, myFloat *ai)
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

void Button::setSawtoothOsc(WaveTableOsc *osc, float baseFreq, int sampleRate) {    
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
void Button::defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai) {
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




Button::~Button(void) {
}


