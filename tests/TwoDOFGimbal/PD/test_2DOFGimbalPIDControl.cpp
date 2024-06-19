//
// Created by Арсений Плахотнюк on 18.06.2024.
//
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/rhsPID.hpp"
#include "tests/TwoDOFGimbal/SystemConfig.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью ПИД регулятора
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;
using namespace tests;

class PidData : public ::tests::TwoDOFGimbal::ControlTwoAxisGimbalData, public ::testing::Test {
protected:

    //Control params
    const double kP1 = 4500;
    const double kI1 = 0;
    const double kD1 = 5000;

    const double kP2 = 4500;
    const double kI2 = 0;
    const double kD2 = 5000;
};

TEST_F(PidData, TEST1) {
    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/PID/data/TwoAxisGimbalPID.txt", std::ios::out);

    PID controller1(kP1, kI1, kD1, maxControlValue);
    PID controller2(kP2, kI2, kD2, maxControlValue);

    Utils::fileDrop(file, state, desiredTraj, timeStartModeling);

    ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep, desiredTraj);

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        Utils::fileDrop(file, state, desiredTraj, t);
    }
    file.close();

}

TEST_F(PidData, TEST_TUNING) {

    const double kp1 = 4500;
    const double ki1 = 0;
    const double kd1 = 5000;

    const double kp2 = 4500;
    const double ki2 = 0;
    const double kd2 = 5000;

    PID controller1(kp1, ki1, kd1, maxControlValue);
    PID controller2(kp2, ki2, kd2, maxControlValue);
    ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep, desiredTraj);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    double stateError = 0;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        stateError += std::sqrt(
                (state(0) - state(4)) * (state(0) - state(4)) + (state(1) - state(5)) * (state(1) - state(5)));
    }

    std::cout << stateError << std::endl;
}

