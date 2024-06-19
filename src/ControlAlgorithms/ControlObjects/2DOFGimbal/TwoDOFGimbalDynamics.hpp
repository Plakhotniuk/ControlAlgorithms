//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#pragma once

#include "DirectFormParams.hpp"
#include "ControlAlgorithms/Utils/NoiseGeneration.hpp"

namespace ControlAlgorithms::ControlObjects::TwoDOFGimbal {

    /** Compute dynamics of 2DOF gimbal with control
     * System equation:
     * M(q) * q" + C(q, q') * q' + G(q) + F(q') = tau + tauD
     * u = tau
     *
     * where:
     *      - q - the joint angle vector
     *      - M(q) - gimbal moment of inertia matrix
     *      - C(q, q') - Coriolis matrix
     *      - G(q) - gravity force vector
     *      - F(q') - friction forces vector
     *      - tau - torque input vector
     *      - u -  control vector
     *      - tauD - disturbance
     * @param stateVector
     * @param controlAction
     */

    [[nodiscard]] inline State
    computeDynamics(const State &stateVector, const Vector2d &controlAction, const DirectFormParams &params){
        const Matrix2d M = params.momentOfInertiaMatrix(stateVector(1));
        const Matrix2d C = params.coriolisCentrifugalGiroscopeForcesMatrix(stateVector(1), stateVector(2),
                                                                           stateVector(3));
        const Vector2d G = params.gravityVector(stateVector(1));
        const Vector2d F = params.frictionForceVector(stateVector.segment<2>(2));

        /**
         * System feedback form for control design
         * x1' = x2
         * x2' = f(x, t) + g(x, t) * u(t)
         *
         * x1' = x2
         * x2' = h(t) + gConstDiag(x, t) * u(t)
         */
        const Vector2d x1Dot = stateVector.segment<2>(2); //!< angular velocities
        const Matrix2d g = M.inverse(); //!< the control matrix
        const Vector2d disturbanceVector = Random::getVectorNoise<double, 2>(params.randomEngine_, params.disturbanceSigma_);
        const Vector2d f = g * (-C * stateVector.segment<2>(2) - G - F - disturbanceVector); //!< nonlinear dynamics vector
        const Vector2d h = f + (g - params.getConstDiagMatrixG()) * controlAction;
        const Vector2d x2Dot = h + params.getConstDiagMatrixG() * controlAction;
        return {x1Dot(0), x1Dot(1), x2Dot(0), x2Dot(1), 0, 0, 0, 0};
    }
}
