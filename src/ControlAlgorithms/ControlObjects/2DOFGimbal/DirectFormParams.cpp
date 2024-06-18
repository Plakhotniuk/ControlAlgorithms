//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#include "DirectFormParams.hpp"

namespace ControlAlgorithms::ControlObjects::TwoDOFGimbal {
    [[nodiscard]] Matrix2d DirectFormParams::momentOfInertiaMatrix(
            const double &q2) const noexcept {
        return Matrix2d{{J1_ + J2_ * std::cos(q2) * std::cos(q2) + J3_ * std::sin(q2) * std::sin(q2), 0.},
                        {0.,                                                                          J4_}};
    }

    [[nodiscard]] Matrix2d DirectFormParams::coriolisCentrifugalGiroscopeForcesMatrix(
            const double &q2, const double q1Dot, const double q2Dot) const noexcept {
        Matrix2d C{
                {q2Dot * (J2_ - J3_) * std::sin(q2) * std::cos(q2),
                        q1Dot * (J2_ - J3_) * std::sin(q2) * std::cos(q2)},
                {-q1Dot * (J2_ - J3_) * std::sin(q2) * std::cos(q2),
                        0.}};
        return C;
    }

    [[nodiscard]] Vector2d DirectFormParams::gravityVector(const double &q2) const noexcept {
        return {0., Kg_ * std::sin(q2)};
    }

    [[nodiscard]] Vector2d DirectFormParams::frictionForceVector(
            const ControlAlgorithms::Vector2d &velocityVec) const noexcept {
        return velocityVec * Fs_;
    }
}