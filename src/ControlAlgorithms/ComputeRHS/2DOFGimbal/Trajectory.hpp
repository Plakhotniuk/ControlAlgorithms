//
// Created by Арсений Плахотнюк on 19.06.2024.
//

#pragma once
#include "ControlAlgorithms/Utils/BasicTypes.hpp"

namespace ControlAlgorithms::ComputeRHS{
    struct Trajectory {
        /**
         * Траектория: x = {a1 + b1 * sin(omega * t), a2 + b2 * cos(omega * t)}
         */
        double omega_;
        double a1_;
        double a2_;
        double b1_;
        double b2_;


        [[nodiscard]] Vector2d getPosition(const double t) const {
            return {a1_ + b1_ * std::sin(omega_ * t),
                    a2_ + b2_ * std::cos(omega_ * t)};
        }

        [[nodiscard]] Vector2d getVelocity(const double t) const {
            return {omega_ * b1_ * std::cos(omega_ * t),
                    -omega_ * b2_ * std::sin(omega_ * t)};
        }

        [[nodiscard]] Vector2d getAcceleration(const double t) const {
            return {-omega_ * omega_ * b1_ * std::sin(omega_ * t),
                    -omega_ * omega_ * b2_ * std::cos(omega_ * t)};
        }

        [[nodiscard]] Vector4d getState(const double t) const {
            return {a1_ + b1_ * std::sin(omega_ * t),
                    a2_ + b2_ * std::cos(omega_ * t),
                    omega_ * b1_ * std::cos(omega_ * t),
                    -omega_ * b2_ * std::sin(omega_ * t)};
        }

    };
}
