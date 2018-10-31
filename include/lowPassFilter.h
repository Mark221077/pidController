#ifndef H_LOWPASSFILTER_INCLUDED
#define H_LOWPASSFILTER_INCLUDED 1

//simple first order low pass filter
class LowPassFilter {

private:
    double uPrev = 20, yPrev = 20;

public:
    double nextStep(double u);

};


#endif