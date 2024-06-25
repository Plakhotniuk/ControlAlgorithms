//
// Created by Арсений Плахотнюк on 19.06.2024.
//
#pragma once
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/CascadePI.hpp"
#include "tests/TwoDOFGimbal/SystemConfig.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью ПИД регулятора
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;

class CascadePIData : public ::tests::TwoDOFGimbal::ControlTwoAxisGimbalData, public ::testing::Test {
protected:

    /// Внутренний кардан
    // Параметры контроллеров
    // Каскадное включение двух ПИ регуляторов
    // внутренний (inner) Position control
    const double positionKp1 = 10.0;
    const double positionKi1 = 0.5;
    // внешний (outer) Rate control
    const double rateKp1 = 1000.;
    const double rateKi1 = 100.;

    /// Внешний кардан
    // Параметры контроллеров
    // Каскадное включение двух ПИ регуляторов
    // внутренний (inner) Position control
    const double positionKp2 = 15;
    const double positionKi2 = 1;

    // внешний (outer) Rate control
    const double rateKp2 = 1000.;
    const double rateKi2 = 100.;

};

TEST_F(CascadePIData, TEST1) {
    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/CascadePI/data/TwoAxisGimbalCascadePIRealDynamics.txt", std::ios::out);

    PID positionController1(positionKp1, positionKi1, 0, maxControlValue);
    PID rateController1(rateKp1, rateKi1, 0, maxControlValue);

    PID positionController2(positionKp2, positionKi2, 0, maxControlValue);
    PID rateController2(rateKp2, rateKi2, 0, maxControlValue);

    tests::Utils::fileDrop(file, state, desiredTraj, timeStartModeling);

    ComputeRHS::GimbalCascadePI::TwoDOFGimbalRHS rhs(state, positionController1, rateController1, positionController2,
                                                     rateController2, params, integrationStep, desiredTraj);

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        tests::Utils::fileDrop(file, state, desiredTraj, t);
    }
    file.close();
}

TEST_F(CascadePIData, TEST_STATIC_ADD_NOISE) {

    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/CascadePI/data/TwoAxisGimbalCascadePINoiseMetrics.txt", std::ios::out);

//    tests::Utils::fileDrop(file, state, desiredTraj, timeStartModeling);
    const double ang_sec = 0.000005;
    const double maxSigma = ang_sec * 20;
    for(double sigma_noise = 0; sigma_noise < maxSigma; sigma_noise+=0.5*ang_sec){
        params.disturbanceSigma_ =  Matrix4d::Identity() * sigma_noise;

        PID positionController1(positionKp1, positionKi1, 0, maxControlValue);
        PID rateController1(rateKp1, rateKi1, 0, maxControlValue);

        PID positionController2(positionKp2, positionKi2, 0, maxControlValue);
        PID rateController2(rateKp2, rateKi2, 0, maxControlValue);

        state = {0, 0, 0, 0};

        ComputeRHS::GimbalCascadePI::TwoDOFGimbalRHS rhs(state, positionController1, rateController1, positionController2,
                                                         rateController2, params, integrationStep, desiredTraj);

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
            Vector4d disturbanceVector = Random::getVectorNoise<double, 4>(params.randomEngine_,
                                                                           params.disturbanceSigma_);
            state += disturbanceVector;
        }
        file << std::setprecision(10) << sigma_noise <<" "<< ise << " " << iae << " " << itae << "\n";
    }

    file.close();
}
