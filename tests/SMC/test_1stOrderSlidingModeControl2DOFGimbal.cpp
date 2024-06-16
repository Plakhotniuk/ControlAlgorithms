//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include <fstream>
#include "ControlAlgorithms/Controllers/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ControlObjects/2DOFGimbal/FeedbackForm.hpp"
#include "gtest/gtest.h"
#include <boost/numeric/odeint.hpp>
#include "tests/Utils.hpp"

/**
 *  Тест управления двух осевым поворотным устройством с помощью sliding mode control
 */

using namespace ControlAlgorithms;
using namespace ControlObjects;
using namespace Controllers;

class ControlTwoAxisGimbalSMCData : public ::testing::Test {
protected:
    // Время моделирования
    const double timeStartModeling = 0.;  //!< время начала моделирования
    const double timeEndModeling = 10.;   //!< время конца моделирования
    const double checkTime = 3.;  //!< момент времени, начиная с которого происходит проверка состояния системы
    const double angleTolerance = 2.e-3;  //!< угловая точность наведения

    // параметры интегрирования
    const double integrationStep = 0.0001;
    const double controlClock = 0.001;
    const double numOfIter = static_cast<uint>(controlClock / integrationStep);


    // Chattering avoidance
    const double phi = 0.1;

    // System params
    Params::State state{.x = {0, 0, 0, 0}, .t = 0};

    const Params::DirectFormParams params{.J1_ = 5.72 * 1e1,
            .J2_ = 5.79 * 1e1,
            .J3_ = 7.04 * 1e1,
            .J4_ = 6.22 * 1e1,
            .Kg_ = 0.1,
            .Fs_ = 0.,
            .gConstDiag_ = Matrix2d{{1, 0},
                                    {0, 1}}};

    Vector2d prevStepVelocity = state.x.segment<2>(2);
    Vector2d desiredState = {1., 1.};            // целевое состояние
    Vector2d desiredStateDerivative = {0., 0.};  // целевая скорость

    // Control
    Vector2d uControl = {0., 0.};                // control input, u

    Matrix2d kMatrix = Matrix2d{{18, 0},
                          {0,  18}};
    Matrix2d lambdaMatrix = Matrix2d{{5, 0},
                               {0, 5}};

    void SetUp() override {}
};

TEST_F(ControlTwoAxisGimbalSMCData, TEST1) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/SMC/data/TwoAxisGimbalSMC.txt", std::ios::out);

    TwoDOFGimbalFeedbackForm gimbal(params);  //!< модель системы

    FirstOrderSMCTimeDelay<2> controller(lambdaMatrix, kMatrix, phi);

    auto desiredTrajectory = [](const double t) -> Vector2d {
        return {1, 1};
    };



}
