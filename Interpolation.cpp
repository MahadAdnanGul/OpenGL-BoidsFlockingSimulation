#include <cmath>
#include "Interpolation.h"
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif


    double Interpolation::CatmullRomInterpolation(double t, double p0, double p1, double p2, double p3, double tension)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        return 0.5 * ((2 * p1) +
                      (-tension * p0 + p2) * t +
                      (2 * tension * p0 - 5 * p1 + 4 * p2 - tension * p3) * t2 +
                      (-tension * p0 + 3 * p1 - 3 * p2 + tension * p3) * t3);
    }

// Catmull-Rom interpolation function for rotation (angles in radians)
double Interpolation::CatmullRomAngleInterpolation(double t, double a0, double a1, double a2, double a3, double tension)
    {
        // Ensure the angles are in the range [-pi, pi] for interpolation
        a0 = fmod(a0 + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
        a1 = fmod(a1 + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
        a2 = fmod(a2 + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
        a3 = fmod(a3 + 3.0 * M_PI, 2.0 * M_PI) - M_PI;

        double t2 = t * t;
        double t3 = t2 * t;

        // Perform Catmull-Rom interpolation for angles with tension
        double s0 = 0.5 * (a2 - a0);
        double s1 = 0.5 * (a3 - a1);
        double result = (2 * a1 - 2 * a2 + s0 + s1) * t3 +
                        (-3 * a1 + 3 * a2 - 2 * s0 - s1) * t2 +
                        s0 * (1.0 - tension) * t +
                        a1;

        return result;
    }

double Interpolation::BSplinePositionInterpolation(double t, double p0, double p1, double p2, double p3) {
    double t2 = t * t;
    double t3 = t2 * t;

    double b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6.0;
    double b1 = (3 * t3 - 6 * t2 + 4) / 6.0;
    double b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0;
    double b3 = t3 / 6.0;

    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

double Interpolation::BSplineAngleInterpolation(double t, double a0, double a1, double a2, double a3) {
    double t2 = t * t;
    double t3 = t2 * t;

    double b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6.0;
    double b1 = (3 * t3 - 6 * t2 + 4) / 6.0;
    double b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0;
    double b3 = t3 / 6.0;

    // Perform B-spline interpolation for angles
    double result = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;

    // Ensure the result is in the range [-pi, pi]
    result = fmod(result + 3.0 * M_PI, 2.0 * M_PI) - M_PI;

    return result;
}


