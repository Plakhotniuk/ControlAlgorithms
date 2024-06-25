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
    const double kP1 = 2000;
    const double kI1 = 0;
    const double kD1 = 2500;

    const double kP2 = 2000;
    const double kI2 = 0;
    const double kD2 = 2500;
};

TEST_F(PidData, TEST1) {
    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/PD/data/TwoAxisGimbalPDRealDynamics.txt", std::ios::out);

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

TEST_F(PidData, TEST_STATIC_ADD_NOISE) {

    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/PD/data/TwoAxisGimbalPDNoiseMetrics.txt", std::ios::out);

//    tests::Utils::fileDrop(file, state, desiredTraj, timeStartModeling);
    const double ang_sec = 0.000005;
    const double maxSigma = ang_sec * 20;
    for(double sigma_noise = 0; sigma_noise < maxSigma; sigma_noise+=0.5*ang_sec){
        params.disturbanceSigma_ =  Matrix4d::Identity() * sigma_noise;

        PID controller1(kP1, kI1, kD1, maxControlValue);
        PID controller2(kP2, kI2, kD2, maxControlValue);

        state = {0, 0, 0, 0};

        ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep, desiredTraj);

        boost::numeric::odeint::runge_kutta4<State> stepper;
        double ise = 0;
        double iae = 0;
        double itae = 0;
        double error = 0;
        Vector2d disturbanceVector;

        for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
            stepper.do_step(rhs, state, t, integrationStep);
            error = (state.segment<2>(0) - desiredTraj.getPosition(t)).norm();
            ise += error * error * integrationStep;
            iae += error * integrationStep;
            itae += t * error * integrationStep;
            Vector4d disturbanceVector = Random::getVectorNoise<double, 4>(params.randomEngine_,
                                                                           params.disturbanceSigma_);
            state += disturbanceVector;
        }
        file << std::setprecision(10) << sigma_noise <<" "<< ise << " " << iae << " " << itae << "\n";
    }

    file.close();
}

TEST_F(PidData, TEST_TUNING) {

    const double kp1 = 4500;
    const double ki1 = 0;
    const double kd1 = 2000;

    const double kp2 = 4500;
    const double ki2 = 0;
    const double kd2 = 2000;

    PID controller1(kp1, ki1, kd1, maxControlValue);
    PID controller2(kp2, ki2, kd2, maxControlValue);
    ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep, desiredTraj);

    boost::numeric::odeint::runge_kutta4<State> stepper;

    double stateError = 0;
    Vector4d desiredState = desiredTraj.getState(timeStartModeling);

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        desiredState = desiredTraj.getState(t);
        stateError += std::sqrt(
                (state(0) - desiredState(0)) * (state(0) - desiredState(0)) +
                (state(1) - desiredState(1)) * (state(1) - desiredState(1)));
    }

    std::cout << stateError << std::endl;
}

