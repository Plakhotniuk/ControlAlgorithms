//
// Created by Арсений Плахотнюк on 18.06.2024.
//
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbalPID.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью ПИД регулятора
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;

class ControlTwoAxisGimbalPidData : public ::testing::Test {
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

    const DirectFormParams params{
            .J1_ = 5.72 * 1e1,
            .J2_ = 5.79 * 1e1,
            .J3_ = 7.04 * 1e1,
            .J4_ = 6.22 * 1e1,
            .Kg_ = 0.1, .Fs_ = 0.1,
            .gConstDiag_ = Matrix2d{{1, 0},
                                    {0, 1}}};

    //Control params
    const double kP1 = 4500;
    const double kI1 = 0;
    const double kD1 = 3500;

    const double kP2 = 4500;
    const double kI2 = 0;
    const double kD2 = 2500;
};

TEST_F(ControlTwoAxisGimbalPidData, TEST1){
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/PID/data/TwoAxisGimbalPID.txt", std::ios::out);

    PID controller1(kP1, kI1, kD1);
    PID controller2(kP2, kI2, kD2);

    file << std::setprecision(10) << state.transpose() << " " << timeStartModeling << "\n";

    ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep);

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

TEST_F(ControlTwoAxisGimbalPidData, TEST_TUNING){

    PID controller1(kP1, kI1, kD1);
    PID controller2(kP2, kI2, kD2);
    ComputeRHS::GimbalPID::TwoDOFGimbalRHS rhs(state, controller1, controller2, params, integrationStep);

    std::vector<State> x_vec;
    std::vector<double> times;

    boost::numeric::odeint::runge_kutta4<State> stepper;

    double stateError = 0;

    for (double t = timeStartModeling; t < timeEndModeling; t += integrationStep) {
        stepper.do_step(rhs, state, t, integrationStep);
        stateError += std::sqrt((state(0) - state(4))*(state(0) - state(4)) + (state(1) - state(5))*(state(1) - state(5)));
    }

    std::cout << stateError << std::endl;
}

