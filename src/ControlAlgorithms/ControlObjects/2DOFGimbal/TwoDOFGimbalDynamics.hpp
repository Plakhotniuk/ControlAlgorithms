//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_TWODOFGIMBALDYNAMICS_HPP
#define CONTROLALGORITHMS_TWODOFGIMBALDYNAMICS_HPP

#include "DirectFormParams.hpp"

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

    [[nodiscard]] State
    computeDynamics(const State &stateVector, const Vector2d &controlAction, const DirectFormParams &params);

}

#endif //CONTROLALGORITHMS_TWODOFGIMBALDYNAMICS_HPP
