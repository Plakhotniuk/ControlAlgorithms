//
// Created by Арсений Плахотнюк on 18.06.2024.
//

#pragma once

#include "ControlAlgorithms/Utils/BasicTypes.hpp"

namespace ControlAlgorithms::Controllers {

    class PID {
        double kP_;
        double kI_;
        double kD_;
        double controlAction_;
        double integralError_;  // накопленная ошибка
        double prevError_;      // ошибка на предыдущем шаге

    public:
        PID(const double kP, const double kI, const double kD) : kP_(kP), kI_(kI), kD_(kD),
                                                                 controlAction_(0),
                                                                 integralError_(0),
                                                                 prevError_(0) {};

        /** Функция считает управляющее воздействие ПИД-регулятора
        *
        * @param stateError
        * @param timeStep
        * @return
        */
        void computeControl(const double stateError, const double timeStep) noexcept {
            integralError_ += stateError * timeStep;
            const double derivativeError = (stateError - prevError_) / timeStep;
            prevError_ = stateError;

            controlAction_ = kP_ * stateError + kI_ * integralError_ + kD_ * derivativeError;
        }

        [[nodiscard]] double getControl() const noexcept { return controlAction_; }

    };
}


