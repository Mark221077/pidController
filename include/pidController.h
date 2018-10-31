#ifndef H_PIDCONTROLLER_INCLUDED
#define H_PIDCONTROLLER_INCLUDED 1


//anti-windup PID controller, calculated and simulated in MatLab
class PIDController {
private:
    double uPrev = 0, aPrev = 0, derPrev = 0, intPrev = 0;

    double calcProportional(double u);
    double calcDerivative(double u);
    double calcIntegral(double u);
    double saturateAndCalcA(double u);

public:
    double integ = 0, prop = 0, der = 0;
    double nextStep(double u);          //should be called with a frequency of 1kHz
    void reset();

};


#endif