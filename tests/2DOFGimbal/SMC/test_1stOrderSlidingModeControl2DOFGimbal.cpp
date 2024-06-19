//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include "tests/2DOFGimbal/SystemConfig.hpp"
#include "ControlAlgorithms/Controllers/FirstOrderSMC/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/FOSMTDC.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью скользящего режима.
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;


class ControlTwoAxisGimbalSMCData : public ::tests::TwoDOFGimbal::ControlTwoAxisGimbalSMCData {
protected:
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
