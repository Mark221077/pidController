#include "lowPassFilter.h"


double LowPassFilter::nextStep(double u) {
    yPrev = 0.0002 * uPrev + 0.9998* yPrev;

    uPrev = u;
    return yPrev;
}