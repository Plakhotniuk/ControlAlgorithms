//
// Created by Арсений Плахотнюк on 19.06.2024.
//

#pragma once

#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include "ControlAlgorithms/Controllers/BasicController.hpp"

namespace ControlAlgorithms::Controllers {

    class PDComputedTorqueRegulator: public BasicController{
        double kP_;
        double kD_;
        double prevError_;      // ошибка на предыдущем шаге

    public:
        PDComputedTorqueRegulator(const double kP, const double kD): kP_(kP), kD_(kD), prevError_(0) {};

        /** Функция считает управляющее воздействие ПИД-регулятора
        *
        * @param stateError
        * @param timeStep
        * @return
        */
        void computeControl(const double stateError, const double timeStep) noexcept {
            const double derivativeError = (stateError - prevError_) / timeStep;
            prevError_ = stateError;

            controlAction_ = kP_ * stateError + kD_ * derivativeError;
        }
    };
}
