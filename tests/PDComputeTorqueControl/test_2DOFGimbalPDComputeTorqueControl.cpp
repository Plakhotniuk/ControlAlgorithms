//
// Created by Арсений Плахотнюк on 19.06.2024.
//
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbalPDComputeTorqueControl.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью вычисленного управления крутящим моментом PD
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;

class ControlTwoAxisGimbalPDComputedTorqueControl : public ::testing::Test {
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
    State state{0, 0, 0, 0, 0.5, 0.5, 0, 0};

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

    //Control params
    const double kP1 = 100;
    const double kD1 = 20;

    const double kP2 = 100;
    const double kD2 = 20;
};

TEST_F(ControlTwoAxisGimbalPDComputedTorqueControl, TEST1){
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/PDComputeTorqueControl/data/TwoAxisGimbalPD.txt", std::ios::out);

    PDComputedTorqueRegulator controller1(kP1, kD1);
    PDComputedTorqueRegulator controller2(kP2, kD2);

    file << std::setprecision(10) << state.transpose() << " " << timeStartModeling << "\n";

    ComputeRHS::GimbalPDComputedControl::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        file << state.transpose() << " ";
        file << t << "\n";
    }
    file.close();

}

TEST_F(ControlTwoAxisGimbalPDComputedTorqueControl, TEST_DISTURBANCE){
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/PDComputeTorqueControl/data/TwoAxisGimbalPDdist.txt", std::ios::out);

    PDComputedTorqueRegulator controller1(kP1, kD1);
    PDComputedTorqueRegulator controller2(kP2, kD2);

    params.disturbanceSigma_ = Matrix2d{{0.5, 0}, {0, 0.5}};

    file << std::setprecision(10) << state.transpose() << " " << timeStartModeling << "\n";

    ComputeRHS::GimbalPDComputedControl::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        file << state.transpose() << " ";
        file << t << "\n";
    }
    file.close();
}