#include <cmath>
#include <gsl/gsl_integration.h>
#include <iostream>
#include "timer.hpp"

struct arc
{
    float pi_range[2];
    float radius;
    float phi;
};

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

double arc_func(double t, void *v)
{
    const arc_length &al = *static_cast<const arc_length*>(v);
    return al(t);
}

float qng_int(const arc &a, const float offs, size_t &neval, float &err)
{
    arc_length al(a.radius, a.phi, offs);

    gsl_function f_x = {&arc_func, &al};

    double x;
    double abserr;
    gsl_integration_qng(&f_x, a.pi_range[0], a.pi_range[1], 1e-4, 1e-3, &x, &abserr, &neval);

    err = abserr;

    return x;
}

gsl_integration_workspace *qag_ws = gsl_integration_workspace_alloc (100);

float qag_int(const arc &a, const float offs, float &err)
{
    arc_length al(a.radius, a.phi, offs);

    gsl_function f_x = {&arc_func, &al};

    double x;
    double abserr;
    gsl_integration_qag(&f_x, a.pi_range[0], a.pi_range[1], 1e-1, 1e-1, 100, GSL_INTEG_GAUSS61, qag_ws, &x, &abserr);

    err = abserr;

    return x;
}

float trap_int(const arc &a, const float offs, size_t steps)
{
    const arc_length al(a.radius, a.phi, offs);
    const float      dx = (a.pi_range[1]-a.pi_range[0])/(steps-1);

    float res = al(a.pi_range[0]);
    for(size_t i = 1; i < steps-1; ++i)
    {
        float x = a.pi_range[0] + i*dx;
        res += 2.0*al(x);
    }
    res += al(a.pi_range[1]);
    res *= dx*0.5f;

    return res;
}

float simp_int(const arc &a, const float offs, size_t steps)
{
    const arc_length al(a.radius, a.phi, offs);
    const float      dx = (a.pi_range[1]-a.pi_range[0])/(steps-1);

    float res = al(a.pi_range[0]);
    for(size_t i = 1; i < steps-3; i+=2)
    {
        res += 4.0*al(a.pi_range[0] +    i *dx);
        res += 2.0*al(a.pi_range[0] + (i+1)*dx);
    }
    res += 4.0*al(a.pi_range[1] - dx);
    res += al(a.pi_range[1]);
    res *= dx/3.0f;

    return res;
}

int main(int argc, char *arg[])
{
    arc a;
    a.pi_range[0] = 0;
    a.pi_range[1] = 3*M_PI/4;
    a.radius = 1.0;
    a.phi = M_PI*3/16.0;

    float offs = 2.0;
    size_t N = 1000;

    float qng_e;
    {
        timer t;
        t.start();
        size_t qng_eval;
        float qng_err;

        for(size_t i = 0; i < N; ++i)
        {
            qng_e = qng_int(a, offs, qng_eval, qng_err);
        }
        t.stop();

        std::cout << "int: " << qng_e << " abs_err: " << qng_err << " neval: " << qng_eval << " time: " << t.interval_S()/static_cast<float>(N) << std::endl;
    }

    float qag_e;
    {
        timer t;
        t.start();
        float qag_err;

        for(size_t i = 0; i < N; ++i)
        {
            qag_e = qag_int(a, offs, qag_err);
        }
        t.stop();

        std::cout << "int: " << qag_e << " abs_err: " << qag_err  << " time: " << t.interval_S()/static_cast<float>(N) << std::endl;
    }

    float trap_e;
    float m;
    {
        timer t;
        t.start();
        for(size_t i = 0; i < N; ++i)
        {
            trap_e = trap_int(a, offs, 12);
            m+=trap_e;
        }
        t.stop();

        std::cout << "int: " << trap_e << " err: " << trap_e - qag_e <<  " time: " <<  t.interval_S()/static_cast<float>(N) << std::endl;
    }

    float simp_e;
    {
        timer t;
        t.start();
        for(size_t i = 0; i < N; ++i)
        {
            simp_e = simp_int(a, offs, 11);
            m+= simp_e;
        }
        t.stop();

        std::cout << "int: " << simp_e << " err: " << simp_e - qag_e <<  " time: " <<  t.interval_S()/static_cast<float>(N) << std::endl;

    }
    std::cout << (a.pi_range[1]-a.pi_range[0])*(offs+a.radius) << std::endl;
    std::cout << m << std::endl;

    return 0;
}
