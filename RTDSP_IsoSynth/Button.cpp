
#include "Button.h"
#include <math.h>


Button::Button(double f1, double f2, double f3, int sampleRate) {


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

 double Button::getOutput(){

    double sound=env->process()*(osc1.getOutput() + osc2.getOutput() + osc3.getOutput());
    osc1.updatePhase();
    osc2.updatePhase(); 
    osc3.updatePhase();    

    filter.setCutoff(filterCuttoff);
    filter.setCutoffMod(filterEnv->process());
    return filter.process(sound);
     }

    void Button::setNoteOn(int pressed){
        noteOn=pressed;
    }
    int Button::isNoteOn(){
        return noteOn;
    }

void buttonPressed(int note, int sampleRate) {

  double oscillatorFrequency=(a / 32.0) * pow(2.0,((note - 9.0) / 12.0));;

    osc1.setFrequency(note/sampleRate);
    osc2.setFrequency(note/sampleRate);
    osc3.setFrequency(note/sampleRate);  

    env->gate(true);
    filterEnv->gate(true);
    
    rt_printf("Frequency %f Note %d :ON \n", oscillatorFrequency, note);
  
}

void buttonReleased(int note) {

  env->gate(false);
  filterEnv->gate(false);
  
}




Button::~Button(void) {
}


