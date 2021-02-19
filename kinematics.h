#pragma once

#define LEN_A 20.0
#define LEN_B 20.0

#include <cmath>

struct arm_angles
{
    float shoulder;
    float elbow;
};

double calc_cosbeta( double x, double y );
double calc_alpha( double x, double y, double beta, double cosbeta );
struct arm_angles calculate_angles( float x, float y );

inline float rad_to_deg( float radians )
{
    return 180.0 * M_1_PI * radians;
}

inline float deg_to_rad( float degrees )
{
    return degrees * (M_PI / 180.0);
}
