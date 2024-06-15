//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include <fstream>
//#include "LaserP/Control/CalculateControl.hpp"
#include "ControlAlgorithms/Controllers/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ControlObjects/2DOFGimbalFeedbackForm.hpp"
#include "gtest/gtest.h"

/**
 *  Тест управления двух осевым поворотным устройством с помощью sliding mode control
 */

using namespace ControlAlgorithms;

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

    // noise
    Matrix2d sigma = Matrix2d::Identity();
    std::mt19937 randomEngine;

    // Chattering avoidance
    const double phi = 0.1;

    const Integrators::ExplicitRK::IntegrationParameters<TwoDOFGimbalFeedbackForm> intParams = {integrationStep, controlClock};
    const double numOfIter = static_cast<uint>(controlClock / integrationStep);

    TwoDOFGimbalFeedbackForm::State currentState{.x = {0., 0., 0., 0.}, .t = 0.};  //!< начальное состояние системы
    Vector2d prevStepVelocity = currentState.x.segment<2>(2);
    Vector2d desiredState = {0., 0.};            // целевое состояние
    Vector2d desiredStateDerivative = {0., 0.};  // целевая скорость
    Vector2d uControl = {0., 0.};                // control input, u
    TwoDOFGimbalFeedbackForm::ConstantSystemParams constantSystemParams{.J1 = 5.72 * 1e1,
            .J2 = 5.79 * 1e1,
            .J3 = 7.04 * 1e1,
            .J4 = 6.22 * 1e1,
            .Kg = 0.1,
            .Fs = 0.,
            .gConstDiag = Matrix2d{{1, 0}, {0, 1}}};
    TwoDOFGimbalFeedbackForm::Params dynamicSystemParams{.constantSystemParams = constantSystemParams};

    Matrix2d K = Matrix2d{{18, 0}, {0, 18}};
    Matrix2d Lambda = Matrix2d{{5, 0}, {0, 5}};

    void SetUp() override { dynamicSystemParams.updateDynamicSystemParams(currentState, uControl); }
};

TEST_F(ControlTwoAxisGimbalSMCData, TEST1) {
    std::fstream file;
    file.open(PROJECT_DIR + "/tests/Control/dataFiles/Gimbal/TwoAxisGimbalSMC2.txt", std::ios::out);

    auto desiredTrajectory = [](const double t) -> Vector2d {
        return {0.35 + 0.01 * Utils::Math::sin(2 * t), 0.25 + 0.01 * Utils::Math::cos(2 * t)};
    };

    for (double t = timeStartModeling; t < timeEndModeling; t += controlClock) {
        dynamicSystemParams.updateDynamicSystemParams(currentState, uControl);

        desiredState = desiredTrajectory(t);
        desiredStateDerivative = (desiredTrajectory(t) - desiredTrajectory(t - controlClock)) / controlClock;

        //        uControl = FirstOrderSlidingModeWithTimeDelay::calculate(
        //            dynamicSystemParams, desiredStateDerivative, currentState.x.segment<2>(0) - desiredState,
        //            currentState.x.segment<2>(2) - desiredStateDerivative, Lambda, K, currentState.x.segment<2>(2),
        //            prevStepVelocity, uControl, controlClock, phi);

        uControl = FirstOrderSlidingModeWithTimeDelay::calculate(
                dynamicSystemParams, desiredStateDerivative, currentState.x.segment<2>(0) - desiredState,
                currentState.x.segment<2>(2) - desiredStateDerivative, Lambda, K, phi);

//        uControl = LaserP::Utils::Random::Noise::addVectorNoise<double, 2>(randomEngine, uControl, sigma);

        prevStepVelocity = currentState.x.segment<2>(2);
        currentState = Integrators::ExplicitRK::ExplicitRungeKutta<Integrators::ExplicitRK::ButcherTableERK4>::calc(
                currentState, dynamicSystemParams, intParams)[numOfIter];
        currentState.t = 0;
        if (t > checkTime) {
            ASSERT_NEAR(currentState.x(0), desiredState(0), angleTolerance);
            ASSERT_NEAR(currentState.x(1), desiredState(1), angleTolerance);
        }
        file << t << " " << currentState.x(0) << " " << currentState.x(1) << " " << desiredState(0) << " "
             << desiredState(1) << "\n";
    }
    file.close();
    std::cout << dynamicSystemParams.hMAx * controlClock << std::endl;
}
