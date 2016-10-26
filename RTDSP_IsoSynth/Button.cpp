
#include "Button.h"
#include <math.h>


//Initialiser of the button class. This acts as a voice for each button available
Button::Button(int sampleRate_) {

    // rt_printf("Button Class \n");

    sampleRate=sampleRate_;

    osc1.setFrequency(440/sampleRate); //Set an initial frequency for Oscillator 1
    osc2.setFrequency(440/sampleRate); //Set an initial frequency for Oscillator 2
    osc3.setFrequency(440/sampleRate); //Set an initial frequency for Oscillator 3

    filterEnv.setAttackRate(.1 * sampleRate);  // Set an initial attack time  for the filter Envelope (.1 seconds)
    filterEnv.setDecayRate(.3 * sampleRate); // Set an initial decay time  for the filter Envelope (.3 seconds)
    filterEnv.setReleaseRate(.5 * sampleRate); // Set an initial release time  for the filter Envelope (.5 seconds)
    filterEnv.setSustainLevel(.8); // Set an initial sustain level  for the filter Envelope (.8 amp)

    env.setAttackRate(.01 * sampleRate);  // Set an initial attack time  for the amplitude Envelope (.1 seconds)
    env.setDecayRate(.3 * sampleRate); // Set an initial decay time  for the amplitude Envelope (.3 seconds)
    env.setReleaseRate(.5 * sampleRate); // Set an initial release time  for the amplitude Envelope (.5 seconds)
    env.setSustainLevel(.8); // Set an initial sustain level  for the amplitude Envelope (.8 amp)

    setSawtoothOsc(&osc1, 20, sampleRate); //Call the setSawtoothOsc function to create an 1st oscillator for each button
    setSawtoothOsc(&osc2, 20, sampleRate); //Call the setSawtoothOsc function to create an 2nd oscillator for each button
    setSawtoothOsc(&osc3, 20, sampleRate); //Call the setSawtoothOsc function to create an 3rd oscillator for each button

}


//Function used to set the amplitude envelope values
void Button::setAmpEnv(float a1, float d1, float s1, float r1){

    //Define the parameters as the input values
    attack=a1;
    decay=d1;
    sustain=s1;
    release=r1;

    //Only recalculate the envelope values if the envelope is in use (button has been pressed)
    if(env.getState()!=0){    

    //Recalculate all envelope values as defined.
        env.setAttackRate(attack * sampleRate); 
        env.setDecayRate(decay * sampleRate);
        env.setReleaseRate(release * sampleRate);
        env.setSustainLevel(sustain);
    }
}

//Function used to set the filter envelope values
void Button::setFilterEnv(float a2, float d2, float s2, float r2){

    //Define the parameters as the input values
    fAttack=a2;
    fDecay=d2;
    fSustain=s2;
    fRelease=r2;

    //Only recalculate the envelope values if the envelope is in use (button has been pressed)
    if(filterEnv.getState()!=0){  
    filterEnv.setAttackRate(fAttack * sampleRate);
    filterEnv.setDecayRate(fDecay * sampleRate);
    filterEnv.setReleaseRate(fRelease * sampleRate);
    filterEnv.setSustainLevel(fSustain);
}
}

//Get the final output
double Button::getOutput(){

    //If the envelope state is FALSE then return silence (0.0)
    if(env.getState()==0){
        return 0.0;
    }

    //Output sound is the sum of all three oscillators
    //Each oscillator is reduced to 1/6th of its original value originally in order to  prevent clipping
    //This attenuation is to be added as a user controllable parameter
    //The sum of the oscillators is then multiplied by the amplitude envelope in order to allow the timbre to be user controlled
    double sound=env.process()*((osc1.getOutput()/6) + (osc2.getOutput()/6) + (osc3.getOutput()/6));

    //Temporary fixed values for the filter
    //Also aim to have these as user controllable 
    double resonance = 0.6;
    double cutoff =0.3;

    //Temporary fixed value for the amount that the envelope will modulate the filter
    //Should range between -1 & 1
    double filterEnvelopeAmount =1;

    //Set the resonance, cutoff and cutoff modulation of the filter
    filter.setCutoffMod(filterEnvelopeAmount*filterEnv.process());
    filter.setCutoff(cutoff);
    filter.setResonance(resonance);

    //Process the output with the filter with its not updated parameters
    sound = filter.process(sound);

    //Update the phase for each oscillator
    osc1.updatePhase();
    osc2.updatePhase(); 
    osc3.updatePhase();    

    //Output the final audio signal
    return sound;
}


//Functions not currently in use
//To be used to check whether or not a voice is currently in use
void Button::setNoteOn(int pressed){
    noteOn=pressed;
}
int Button::isNoteOn(){
    return noteOn;
}


//Function is called when a button is pressed
void Button::buttonPressed(int note, int sampleRate) {


    //Temporary fixed values for oscillator detune amount
    //Detune ranged from -1.0 to 1.0
    //This equates to 0-100 cent in either direction of the centre frequency.
   detuneOsc1 = 0.0;
   detuneOsc2 = 0.12;
   detuneOsc3 = 0.25;


   //This equatio concerts the midi note value of each button pressed 
   //to the frequency to be used by the oscillator
   //The notes are pitched up by 24 semitones as the initial values were deemed too low. 
   //THis semitone shift is another parameter that could be implemented as user selectable
   double oscillatorFrequency1=(440.0 / 32.0) * pow(2.0,(((note+24) - 9.0) / 12.0));
   double oscillatorFrequency2=(440.0 / 32.0) * pow(2.0,(((note+24) - 9.0) / 12.0));
   double oscillatorFrequency3=(440.0 / 32.0) * pow(2.0,(((note+24) - 9.0) / 12.0));

   //The calculated frequency is then assigned to the individual oscillator 
   //(the detune amount is added ot this vaule)
   osc1.setFrequency((oscillatorFrequency1+detuneOsc1)/sampleRate);
   osc2.setFrequency((oscillatorFrequency2+detuneOsc2)/sampleRate);
   osc3.setFrequency((oscillatorFrequency3+detuneOsc3)/sampleRate);  

   //As the button is pressed the assigned amplitude envelope has its parameters
   //recalculated at the amount that is currently being read in by the potentiometers.
    env.setAttackRate(attack * sampleRate); 
    env.setDecayRate(decay * sampleRate);
    env.setReleaseRate(release * sampleRate);
    env.setSustainLevel(sustain);

   //As the button is pressed the assigned amplitude envelope has its parameters
   //recalculated at the amount that is currently being read in by the potentiometers.
    filterEnv.setAttackRate(fAttack * sampleRate);
    filterEnv.setDecayRate(fDecay * sampleRate);
    filterEnv.setReleaseRate(fRelease * sampleRate);
    filterEnv.setSustainLevel(fSustain);

    //The statuses of both the amplitude and filter envelopes 
    //assigned to button that is being pressed is set to true
    env.gate(true);
    filterEnv.gate(true);
    
    // rt_printf("Frequency %f Note %d :ON \n", oscillatorFrequency, note);

}

//Function is called when a button is released
void Button::buttonReleased() {

    // State of both the amplitude envelope and the 
    // filter envelope are set to false
  env.gate(false);
  filterEnv.gate(false);
  
}

//Create a wavetable per octave
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

// defineSawtooth
// prepares sawtooth (and other wave types) harmonics for ifft
void Button::defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai) {
    if (numHarmonics > (len >> 1))
        numHarmonics = (len >> 1);
    
    // clear
    for (int idx = 0; idx < len; idx++) {
        ai[idx] = 0;
        ar[idx] = 0;
    }

    // // sawtooth
    // for (int idx = 1, jdx = len - 1; idx <= numHarmonics; idx++, jdx--) {
    //     myFloat temp = -1.0 / idx;
    //     ar[idx] = -temp;
    //     ar[jdx] = temp;
    // }
    
     // square
    for (int idx = 1, jdx = len - 1; idx <= numHarmonics; idx++, jdx--) {
     myFloat temp = idx & 0x01 ? 1.0 / idx : 0.0;
     ar[idx] = -temp;
     ar[jdx] = temp;
 }


     // // triangle
     // float sign = 1;
     // for (int idx = 1, jdx = len - 1; idx <= numHarmonics; idx++, jdx--) {
     // myFloat temp = idx & 0x01 ? 1.0 / (idx * idx) * (sign = -sign) : 0.0;
     // ar[idx] = -temp;
     // ar[jdx] = temp;
     // }

}




Button::~Button(void) {
}


