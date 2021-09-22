#ifndef _ARC_INT_HPP_
#define _ARC_INT_HPP_
#include <cmath>

struct arc_length
{
    arc_length(float r, float phi, float offs)
    {
        r2          = r*r;
        s2_phi      = std::sin(phi);
        s2_phi      = s2_phi*s2_phi;
        sqrt_coeff  = 2.0f*offs*r*std::cos(phi);
        num_end     = r2 + offs*offs*(1.0f - s2_phi);
    }

    float operator()(const float theta) const
    {
        const float c_theta   = std::cos(theta);
        const float subexpr   = s2_phi*c_theta*c_theta;
        const float subexpr1m = 1.0f - subexpr;

        const float numerator = std::sqrt(r2*subexpr*(subexpr-2.0f)
                                          + sqrt_coeff*std::sqrt(subexpr1m)*subexpr1m
                                          + num_end);
        return numerator/subexpr1m;
    }

    float s2_phi;
    float r2;
    float sqrt_coeff;
    float num_end;
};
#endif
