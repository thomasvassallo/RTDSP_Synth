#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

class Filter
{
public:
    Filter();
    
    void calculateCoefficients(int const type,double const frequency,double const sample_rate,double const q,double const db_gain,bool q_is_bandwidth);
    float processSamples(float sound);

    
private:
    
    float b0a0,b1a0,b2a0,a1a0,a2a0;
    float ou1,ou2,in1,in2;
    double alpha,a0,a1,a2,b0,b1,b2;
    double in0;
    float  yn;    
    
};



#endif  // FILTER_H_INCLUDED
