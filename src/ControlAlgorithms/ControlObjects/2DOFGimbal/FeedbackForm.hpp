//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_FEEDBACKFORM_HPP
#define CONTROLALGORITHMS_FEEDBACKFORM_HPP

#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include "DirectFormParams.hpp"

namespace ControlAlgorithms::ControlObjects {

    /**
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
     */
    class TwoDOFGimbalFeedbackForm {
        Params::DirectFormParams directFormParams_;

    public:
        explicit TwoDOFGimbalFeedbackForm(const Params::DirectFormParams &params) : directFormParams_(params) {};

        /** Calculate right hand side
         *
         * @param stateVector
         * @param controlU
         */
        [[nodiscard]] Vector4d calcRHS(const Params::State &stateVector, const Vector2d &controlAction) const;

    };
}

#endif //CONTROLALGORITHMS_FEEDBACKFORM_HPP
