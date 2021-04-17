#include <cmath>
#include "kinematics.h"


double calc_cosbeta( double x, double y )
{
    //          x^2 + y^2 - A^2 - B^2
    // cos(b) = ---------------------
    //                   2AB
    
    double n = (x * x) + (y * y) - (LEN_A * LEN_A) - (LEN_B * LEN_B);
    return n / (2.0 * LEN_A * LEN_B);
}

double calc_alpha( double x, double y, double beta, double cosbeta )
{
    //                               sin(b)
    // a = atan(y/x) - atan( B * -------------- )
    //                           A + B * cos(b)
    //
    // sin(b) = sqrt(1 - cosb^2)

    double a1 = atan(y / x);
    double a2 = atan((LEN_B * sin(beta)) / (LEN_A + (LEN_B * cosbeta)));

    return a1 - a2;
}

struct arm_angles calculate_angles( float x, float y )
{
    struct arm_angles ang;

    double cosb = calc_cosbeta(x, y);
    double b = -acos(cosb);
    
    ang.shoulder = (float)calc_alpha(-y, x, b, cosb) + M_PI_2;
    if( y < 0 ) ang.shoulder = ang.shoulder + M_PI;

    ang.elbow = b;
    return ang;
}

