//
// Created by Арсений Плахотнюк on 16.06.2024.
//
/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#include <iostream>
#include <vector>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>


namespace HarmonicOscillator {

//[ rhs_function
/* The type of container used to hold the state vector */
    typedef std::vector<double> state_type;

    const double gam = 0.15;

/* The rhs of x' = f(x) */
    void harmonic_oscillator(const state_type &x, state_type &dxdt, const double /* t */) {
        dxdt[0] = x[1];
        dxdt[1] = -x[0] - gam * x[1];
    }
//]





//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
    class harm_osc {

        double m_gam;

    public:
        harm_osc(double gam) : m_gam(gam) {}

        void operator()(const state_type &x, state_type &dxdt, const double /* t */) {
            dxdt[0] = x[1];
            dxdt[1] = -x[0] - m_gam * x[1];
        }
    };
//]





//[ integrate_observer
    struct push_back_state_and_time {
        std::vector<state_type> &m_states;
        std::vector<double> &m_times;

        push_back_state_and_time(std::vector<state_type> &states, std::vector<double> &times)
                : m_states(states), m_times(times) {}

        void operator()(const state_type &x, double t) {
            m_states.push_back(x);
            m_times.push_back(t);
        }
    };
//]

    struct write_state {
        void operator()(const state_type &x) const {
            std::cout << x[0] << "\t" << x[1] << "\n";
        }
    };
}


TEST(INTEGRATE_RK, TEST1) {
//    using namespace boost::numeric::odeint;
    using namespace HarmonicOscillator;


    //[ state_initialization
    state_type x(2);
    x[0] = 1.0; // start at x=1.0, p=0.0
    x[1] = 0.0;
    //]


    std::cout<< "Init: " << x[0] << ", " << x[1] << std::endl;
    //[ integration
    size_t steps = boost::numeric::odeint::integrate(harmonic_oscillator,
                                                     x, 0.0, 10.0, 0.1);
    //]

    std::cout << "End: " << x[0] << ", " << x[1] << std::endl;



    //[ integration_class
    harm_osc ho(0.15);
    steps = boost::numeric::odeint::integrate(ho,
                                              x, 0.0, 10.0, 0.1);
    //]





    //[ integrate_observ
    std::vector<state_type> x_vec;
    std::vector<double> times;

    steps = boost::numeric::odeint::integrate(harmonic_oscillator,
                                              x, 0.0, 10.0, 0.1,
                                              push_back_state_and_time(x_vec, times));

    /* output */
    for (size_t i = 0; i <= steps; i++) {
        std::cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
    }
    //]







    //[ define_const_stepper
    boost::numeric::odeint::runge_kutta4<state_type> stepper;
    integrate_const(stepper, harmonic_oscillator, x, 0.0, 10.0, 0.01);
    //]




    //[ integrate_const_loop
    const double dt = 0.01;
    for (double t = 0.0; t < 10.0; t += dt)
        stepper.do_step(harmonic_oscillator, x, t, dt);
    //]




    //[ define_adapt_stepper
    typedef boost::numeric::odeint::runge_kutta_cash_karp54<state_type> error_stepper_type;
    //]



    //[ integrate_adapt
    typedef boost::numeric::odeint::controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
    controlled_stepper_type controlled_stepper;
    integrate_adaptive(controlled_stepper, harmonic_oscillator, x, 0.0, 10.0, 0.01);
    //]

    {
        //[integrate_adapt_full
        double abs_err = 1.0e-10, rel_err = 1.0e-6, a_x = 1.0, a_dxdt = 1.0;
        controlled_stepper_type controlled_stepper(
                boost::numeric::odeint::default_error_checker<double, boost::numeric::odeint::range_algebra, boost::numeric::odeint::default_operations>(
                        abs_err, rel_err, a_x, a_dxdt));
        integrate_adaptive(controlled_stepper, harmonic_oscillator, x, 0.0, 10.0, 0.01);
        //]
    }


    //[integrate_adapt_make_controlled
    integrate_adaptive(boost::numeric::odeint::make_controlled<error_stepper_type>(1.0e-10, 1.0e-6),
                       harmonic_oscillator, x, 0.0, 10.0, 0.01);
    //]




    //[integrate_adapt_make_controlled_alternative
    integrate_adaptive(make_controlled(1.0e-10, 1.0e-6, error_stepper_type()),
                       harmonic_oscillator, x, 0.0, 10.0, 0.01);
    //]

#ifdef BOOST_NUMERIC_ODEINT_CXX11
    //[ define_const_stepper_cpp11
    {
        boost::numeric::odeint::runge_kutta4 <state_type> stepper;
        integrate_const(stepper, [](const state_type &x, state_type &dxdt, double t) {
            dxdt[0] = x[1];
            dxdt[1] = -x[0] - gam * x[1];
        }, x, 0.0, 10.0, 0.01);
    }
    //]



    //[ harm_iterator_const_step]
    std::for_each(make_const_step_time_iterator_begin(stepper, harmonic_oscillator, x, 0.0, 0.1, 10.0),
                  make_const_step_time_iterator_end(stepper, harmonic_oscillator, x),
                  [](std::pair<const state_type &, const double &> x) {
                      std::cout << x.second << " " << x.first[0] << " " << x.first[1] << "\n";
                  });
    //]
#endif

}