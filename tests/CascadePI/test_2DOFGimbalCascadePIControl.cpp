//
// Created by Арсений Плахотнюк on 19.06.2024.
//
#include <fstream>
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbalCascadePI.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью ПИД регулятора
 */

using namespace ControlAlgorithms;
using namespace ControlObjects::TwoDOFGimbal;
using namespace Controllers;

class ControlTwoAxisGimbalCascadePIData : public ::testing::Test {
protected:

    // Время моделирования
    const double timeStartModeling = 0.;  //!< время начала моделирования
    const double timeEndModeling = 10.;   //!< время конца моделирования
    const double checkTime = 3.;  //!< момент времени, начиная с которого происходит проверка состояния системы
    const double angleTolerance = 2.e-3;  //!< угловая точность наведения


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
};

TEST_F(ControlTwoAxisGimbalCascadePIData, TEST1) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/CascadePI/data/TwoAxisGimbalCascadePI.txt", std::ios::out);

    PID positionController1(positionKp1, positionKi1, 0);
    PID rateController1(rateKp1, rateKi1, 0);

    PID positionController2(positionKp2, positionKi2, 0);
    PID rateController2(rateKp2, rateKi2, 0);

    file << std::setprecision(10) << state.transpose() << " " << timeStartModeling << "\n";

    ComputeRHS::GimbalCascadePI::TwoDOFGimbalRHS rhs(state, positionController1, rateController1, positionController2,
                                                     rateController2, params, integrationStep);

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