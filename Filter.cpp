#include "Filter.h"

// By Paul Kellett
// http://www.musicdsp.org/showone.php?id=29


// 4 first order cascaded low pass filters
// results in a 24dB/Octave reduction im amplitude
double Filter::process(double inputValue) {
    if (inputValue == 0.0) return inputValue;
    double calculatedCutoff = getCalculatedCutoff();
    buf0 += calculatedCutoff * (inputValue - buf0 + feedbackAmount * (buf0 - buf1));
    buf1 += calculatedCutoff * (buf0 - buf1);
    buf2 += calculatedCutoff * (buf1 - buf2);
    buf3 += calculatedCutoff * (buf2 - buf3);

    //depending on the filter mode the output is calculated
    switch (mode) {
        case FILTER_MODE_LOWPASS:
            return buf3;
        case FILTER_MODE_HIGHPASS:
            return inputValue - buf3;
        case FILTER_MODE_BANDPASS:
            return buf0 - buf3;
        default:
            return 0.0;
    }
}