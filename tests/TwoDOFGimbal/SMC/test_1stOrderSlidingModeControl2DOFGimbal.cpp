//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include "tests/TwoDOFGimbal/SystemConfig.hpp"
#include "ControlAlgorithms/Controllers/FirstOrderSMC/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/FOSMTDC.hpp"
#include "tests/Utils.hpp"
#include "gtest/gtest.h"

/**
 *  Тест управления двух осевым поворотным устройством с помощью скользящего режима.
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;
using namespace tests;


class SMCData : public ::tests::TwoDOFGimbal::ControlTwoAxisGimbalData, public ::testing::Test{
protected:
    //Control params
    // Chattering avoidance
    const double phi = 1;

    Matrix2d kMatrix = Matrix2d{{95, 0},
                                {0,  150}};
    Matrix2d lambdaMatrix = Matrix2d{{5, 0},
                                     {0, 5}};
};

TEST_F(SMCData, TEST_FIXED_STEP) {

    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/SMC/data/TwoAxisGimbalSMCNoised.txt", std::ios::out);

    tests::Utils::fileDrop(file, state, desiredTraj, timeStartModeling);

    FirstOrderSMCTimeDelay controller(lambdaMatrix, kMatrix, phi, maxControlValue);

    ComputeRHS::GimbalFOSMTDC::TwoDOFGimbalRHS rhs(state, controller, params, integrationStep, desiredTraj);


    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        tests::Utils::fileDrop(file, state, desiredTraj, t);
    }
    file.close();
}

TEST_F(SMCData, TEST_STATIC_ADD_NOISE) {

    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/SMC/data/TwoAxisGimbalSMCNoiseMetrics.txt", std::ios::out);

//    tests::Utils::fileDrop(file, state, desiredTraj, timeStartModeling);
    const double maxSigma = 1000;
    for(double sigma_noise = 0; sigma_noise < maxSigma; sigma_noise+=100){
        params.disturbanceSigma_ =  Matrix2d{{sigma_noise, 0}, {0, sigma_noise}};
        FirstOrderSMCTimeDelay controller(lambdaMatrix, kMatrix, phi, maxControlValue);

        state = {0, 0, 0, 0};
        ComputeRHS::GimbalFOSMTDC::TwoDOFGimbalRHS rhs(state, controller, params, integrationStep, desiredTraj);

        boost::numeric::odeint::runge_kutta4<State> stepper;
        double ise = 0;
        double iae = 0;
        double itae = 0;
        double error = 0;

        for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
            stepper.do_step(rhs, state, t, integrationStep);
            error = (state.segment<2>(0) - desiredTraj.getPosition(t)).norm();
            ise += error * error * integrationStep;
            iae += error * integrationStep;
            itae += t * error * integrationStep;
//        tests::Utils::fileDrop(file, state, desiredTraj, t);
        }
        file << std::setprecision(10) << sigma_noise <<" "<< ise << " " << iae << " " << itae << "\n";
    }

    file.close();
}
