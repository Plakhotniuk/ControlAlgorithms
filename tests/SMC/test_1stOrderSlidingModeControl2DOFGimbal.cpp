//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/FOSMTDC.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью скользящего режима.
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;

class ControlTwoAxisGimbalSMCData : public ::testing::Test {
protected:
    // Время моделирования
    const double timeStartModeling = 0.;  //!< время начала моделирования
    const double timeEndModeling = 10.;   //!< время конца моделирования
    const double checkTime = 3.;  //!< момент времени, начиная с которого происходит проверка состояния системы
    const double angleTolerance = 2.e-3;  //!< угловая точность наведения

    // параметры интегрирования
    const double integrationStep = 0.001;
    const double integrTol = 1e-6;

    // System params
    State state{0, 0, 0, 0};
    std::mt19937 randomEngine;

    DirectFormParams params{
            .J1_ = 5.72 * 1e1,
            .J2_ = 5.79 * 1e1,
            .J3_ = 7.04 * 1e1,
            .J4_ = 6.22 * 1e1,
            .Kg_ = 0.1, .Fs_ = 0.1,
            .gConstDiag_ = Matrix2d{{1, 0},
                                    {0, 1}},
            .disturbanceSigma_ = Matrix2d{{0.0, 0},
                                          {0,   0.0}},
            .randomEngine_ = randomEngine};

    ComputeRHS::Trajectory desiredTraj{.omega_ = 2., .a1_ = 0.5, .a2_ = -0.5, .b1_ = 0.05, .b2_ = -0.05};

    //Control params
    // Chattering avoidance
    const double phi = 1;
    Vector2d uControl = {0., 0.};                // control input, u

    Matrix2d kMatrix = Matrix2d{{95, 0},
                                {0,  150}};
    Matrix2d lambdaMatrix = Matrix2d{{5, 0},
                                     {0, 5}};
};

TEST_F(ControlTwoAxisGimbalSMCData, TEST_FIXED_STEP) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/SMC/data/TwoAxisGimbalSMC.txt", std::ios::out);

    file << std::setprecision(10) << state.transpose() << " " << desiredTraj.getState(0).transpose() << " "
         << timeStartModeling << "\n";

    FirstOrderSMCTimeDelay controller(lambdaMatrix, kMatrix, uControl, phi);

    ComputeRHS::GimbalFOSMTDC::TwoDOFGimbalRHS rhs(state, controller, params, integrationStep, desiredTraj);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        file << state.transpose() << " " << desiredTraj.getState(t).transpose() << " ";
        file << t << "\n";
    }
    file.close();
}

TEST_F(ControlTwoAxisGimbalSMCData, TEST_FIXED_STEP_WITH_DISTURBANCE) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/SMC/data/TwoAxisGimbalSMCdist.txt", std::ios::out);

    params.disturbanceSigma_ = Matrix2d{{1, 0},
                                        {0,   1}};

    file << std::setprecision(10) << state.transpose() << " " << desiredTraj.getState(0).transpose() << " "
         << timeStartModeling << "\n";

    FirstOrderSMCTimeDelay controller(lambdaMatrix, kMatrix, uControl, phi);

    ComputeRHS::GimbalFOSMTDC::TwoDOFGimbalRHS rhs(state, controller, params, integrationStep, desiredTraj);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        file << state.transpose() << " " << desiredTraj.getState(t).transpose() << " ";
        file << t << "\n";
    }
    file.close();
}


TEST_F(ControlTwoAxisGimbalSMCData, TEST_ADAPTIVE_STEP) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/SMC/data/TwoAxisGimbalSMC2.txt", std::ios::out);

    file << std::setprecision(10) << state.transpose() << " " << desiredTraj.getState(0).transpose() << " "
         << timeStartModeling << "\n";

    FirstOrderSMCTimeDelay controller(lambdaMatrix, kMatrix, uControl, phi);

    ComputeRHS::GimbalFOSMTDC::TwoDOFGimbalRHS rhs(state, controller, params, integrationStep, desiredTraj);

    std::vector<State> x_vec;
    std::vector<double> times;

    typedef boost::numeric::odeint::runge_kutta_cash_karp54<State> error_stepper_type;
    typedef boost::numeric::odeint::controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
    controlled_stepper_type controlled_stepper;

    boost::numeric::odeint::integrate_adaptive(
            boost::numeric::odeint::make_controlled<error_stepper_type>(integrTol, integrTol),
            rhs, state, timeStartModeling, timeEndModeling, integrationStep,
            push_back_state_and_time(x_vec, times));

    for (unsigned i = 0; i < x_vec.size(); ++i) {
        file << x_vec[i].transpose() << " " << desiredTraj.getState(times[i]).transpose() << " ";
        file << times[i] << "\n";
    }
    file.close();
}
