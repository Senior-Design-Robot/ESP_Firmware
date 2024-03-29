#pragma once

#define LEN_A 20.0
#define LEN_B 20.0
#define ARM_REACH (LEN_A + LEN_B)

#include <math.h>

struct arm_angles
{
    double shoulder;
    double elbow;
};

double calc_cosbeta( double x, double y );
double calc_alpha( double x, double y, double beta, double cosbeta );
struct arm_angles calculate_angles( float x, float y );

inline double rad_to_deg( double radians )
{
    return 180.0 * M_1_PI * radians;
}

inline double deg_to_rad( double degrees )
{
    return degrees * (M_PI / 180.0);
}
