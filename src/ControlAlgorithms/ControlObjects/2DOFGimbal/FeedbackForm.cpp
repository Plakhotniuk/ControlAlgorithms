//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include "FeedbackForm.hpp"

namespace ControlAlgorithms::ControlObjects {

    [[nodiscard]] Vector4d
    TwoDOFGimbalFeedbackForm::calcRHS(const Params::State &stateVector, const Vector2d &controlAction) const {
        const Matrix2d M = directFormParams_.momentOfInertiaMatrix(stateVector.x(1));
        const Matrix2d C = directFormParams_.coriolisCentrifugalGiroscopeForcesMatrix(stateVector.x(1), stateVector.x(2),
                                                                                      stateVector.x(3));
        const Vector2d G = directFormParams_.gravityVector(stateVector.x(1));
        const Vector2d F = directFormParams_.frictionForceVector(stateVector.x.segment<2>(2));

        /**
         * System feedback form for control design
         * x1' = x2
         * x2' = f(x, t) + g(x, t) * u(t)
         *
         * x1' = x2
         * x2' = h(t) + gConstDiag(x, t) * u(t)
         */
        const Vector2d x1Dot = stateVector.x.segment<2>(2); //!< angular velocities
        const Matrix2d g = M.inverse(); //!< the control matrix
        const Vector2d f = g * (-C * stateVector.x.segment<2>(2) - G - F); //!< nonlinear dynamics vector
        const Vector2d h = f + (g - directFormParams_.getConstDiagMatrixG()) * controlAction;
        const Vector2d x2Dot = h + directFormParams_.getConstDiagMatrixG() * controlAction;
        return {x1Dot(0), x1Dot(1), x2Dot(0), x2Dot(1)};
    }
}