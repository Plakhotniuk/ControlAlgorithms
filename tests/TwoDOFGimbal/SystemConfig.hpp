//
// Created by Арсений Плахотнюк on 19.06.2024.
//

#pragma once

#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"
#include "ControlAlgorithms/ComputeRHS/2DOFGimbal/Trajectory.hpp"

namespace tests::TwoDOFGimbal {
    using namespace ControlAlgorithms;
    using namespace ControlObjects::TwoDOFGimbal;

    class ControlTwoAxisGimbalData {
    protected:
        std::fstream file;
        // Время моделирования
        const double timeStartModeling = 0.;  //!< время начала моделирования
        const double timeEndModeling = 10.;   //!< время конца моделирования
        const double checkTime = 3.;  //!< момент времени, начиная с которого происходит проверка состояния системы
        const double angleTolerance = 2.e-3;  //!< угловая точность наведения

        // параметры интегрирования
        const double integrationStep = 0.001;
        const double integrTol = 1e-6;

        // System params
        State state{0, 0, 0, 0};
        std::mt19937 randomEngine;

        DirectFormParams params{
                .J1_ = 5.72 * 1e1,
                .J2_ = 5.79 * 1e1,
                .J3_ = 7.04 * 1e1,
                .J4_ = 6.22 * 1e1,
                .Kg_ = 0.1, .Fs_ = 0.1,
                .gConstDiag_ = Matrix2d{{1, 0},
                                        {0, 1}},
                .disturbanceSigma_ = Matrix2d{{1, 0},
                                              {0,   1}},
                .randomEngine_ = randomEngine};

        ComputeRHS::Trajectory desiredTraj{.omega_ = 2., .a1_ = 0.5, .a2_ = -0.5, .b1_ = 0.05, .b2_ = -0.05};
//        ComputeRHS::Trajectory desiredTraj{.omega_ = 0., .a1_ = 0.5, .a2_ = -0.5, .b1_ = 0.0, .b2_ = -0.0};
        const double maxControlValue = 200;

    };

}
