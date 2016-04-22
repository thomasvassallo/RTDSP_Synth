#ifndef Button_h
#define Button_h

#include "ADSR.h"
#include "Filter.h"
#include "WaveTableOsc.h"


class Button {
public:
    Button(double, double, double);
    ~Button(void);

    double getOutput();
    void setNoteOn(int);
    int isNoteOn();
    void buttonReleased(int);
    void buttonPressed(int , int);


private:
WaveTableOsc osc1;
WaveTableOsc osc2;
WaveTableOsc osc3;

Filter filter;

ADSR env;
ADSR filterEnv;

int filterCuttoff = 200;
int noteOn;
};



#endif
