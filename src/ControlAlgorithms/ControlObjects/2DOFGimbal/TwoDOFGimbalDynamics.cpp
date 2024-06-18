//
// Created by Арсений Плахотнюк on 15.06.2024.
//
#include "TwoDOFGimbalDynamics.hpp"

namespace ControlAlgorithms::ControlObjects::TwoDOFGimbal {

    [[nodiscard]] State
    computeDynamics(const State &stateVector, const Vector2d &controlAction, const DirectFormParams &params) {
        const Matrix2d M = params.momentOfInertiaMatrix(stateVector.x(1));
        const Matrix2d C = params.coriolisCentrifugalGiroscopeForcesMatrix(stateVector.x(1), stateVector.x(2),
                                                                                      stateVector.x(3));
        const Vector2d G = params.gravityVector(stateVector.x(1));
        const Vector2d F = params.frictionForceVector(stateVector.x.segment<2>(2));

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
        const Vector2d h = f + (g - params.getConstDiagMatrixG()) * controlAction;
        const Vector2d x2Dot = h + params.getConstDiagMatrixG() * controlAction;
        return {{x1Dot(0), x1Dot(1), x2Dot(0), x2Dot(1)}, stateVector.xd};
    }
}