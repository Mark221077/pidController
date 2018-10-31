
#include "pidController.h"
#include <Arduino.h>

double PIDController::calcProportional(double u)
{
    return 2 * u;
}

double PIDController::calcDerivative(double u)
{
    derPrev = constrain(40 * u - 40 * uPrev + 0.99005 * derPrev, -80, 80);
    return derPrev;
}

double PIDController::calcIntegral(double u)
{
    intPrev = uPrev * 0.02470 * 0.00100 + 5 * 0.00100 * aPrev + intPrev;
    return intPrev;
}

double PIDController::saturateAndCalcA(double u)
{

    double sat = constrain(u, 0, 325);

    aPrev = sat - u;

    return sat;
}

double PIDController::nextStep(double u)
{

    //anti-windup PID controller, simulated as continuous system in MatLab
    //recalculated in the z domain with T = 1/1000

    //there is around 10% overshoot, and oscillation on the output
    //I think that its acceptable
    //To reduce it, we would need to slow the system down
    //The iron has an InputDelay between 15-20 seconds, and its really hard to have good regulation

    prop = calcProportional(u);
    integ = calcIntegral(u);
    der = calcDerivative(u);
    der = der < 0 ? 0 : der;        //the signal is too noisy, even after the low pass filter
                                    //the derivative is negative, when the difference between e and w is being reduced
                                    //ignore negative values, so we dont slow down the regulation
                                    //this adds overshoot, but its still acceptable
                                    //keep the positive values for faster reaction to changes
      
    uPrev = u;

    return saturateAndCalcA(prop + integ + der);    //saturation and the calculation of a
}


void PIDController::reset() {           //resets the controller
    uPrev = 0;
    intPrev = 0;
    derPrev = 0;
    aPrev = 0;
}