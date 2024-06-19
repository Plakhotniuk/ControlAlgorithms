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
    const double positionKp1 = 8.0;
    const double positionKi1 = 0.1;
    // внешний (outer) Rate control
    const double rateKp1 = 750.;
    const double rateKi1 = 50.;

    /// Внешний кардан
    // Параметры контроллеров
    // Каскадное включение двух ПИ регуляторов
    // внутренний (inner) Position control
    const double positionKp2 = 5.0;
    const double positionKi2 = 0.05;

    // внешний (outer) Rate control
    const double rateKp2 = 300.;
    const double rateKi2 = 150.;
};

TEST_F(CascadePIData, TEST1) {
    file.open(PROJECT_DIR + "/tests/TwoDOFGimbal/CascadePI/data/TwoAxisGimbalCascadePI.txt", std::ios::out);

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
