//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_DIRECTFORMPARAMS_HPP
#define CONTROLALGORITHMS_DIRECTFORMPARAMS_HPP

#include "ControlAlgorithms/Utils/BasicTypes.hpp"

namespace ControlAlgorithms::ControlObjects::Params {

    /**
     * System state
     */
    struct State {
        Vector6d x;//!< state vector [x0 = theta, x1 = ksi, x2 = theta', x3 = ksi', x4 = theta'',x5 = ksi'']
        Vector6d xd;//!< state vector [xd0 = theta, xd1 = ksi, xd2 = theta', xd3 = ksi', xd4 = theta'',xd5 = ksi'']
        double t;  //!< time
    };

    /**
     * Parameters for integration
     */
    struct DirectFormParams {
        double J1_;//!< inertia matrix constant
        double J2_;//!< inertia matrix constant
        double J3_;//!< inertia matrix constant
        double J4_;//!< inertia matrix constant
        double Kg_;//!< (weight of gimbal) x (distance from center of mass to the axis of rotation)
        double Fs_;//!< friction force constant
        Matrix2d gConstDiag_;

        /** Calculate gimbal moment of inertia matrix M
         *
         * @param q
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] Matrix2d momentOfInertiaMatrix(
                const double &q2) const noexcept;

        /** Calculate Coriolis, Centrifugal, Giroscope Forces matrix
         *
         * @param q2
         * @param q1Dot
         * @param q2Dot
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] Matrix2d coriolisCentrifugalGiroscopeForcesMatrix(
                const double &q2, const double q1Dot, const double q2Dot) const noexcept;

        /** Gravity vector calculation
         *
         * @param q2
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] Vector2d gravityVector(const double &q2) const noexcept;

        /** Friction force calculation
         *
         * @param velocityVec
         * @return
         */
        [[nodiscard]] Vector2d frictionForceVector(const Vector2d &velocityVec) const noexcept;

        [[nodiscard]] Matrix2d getConstDiagMatrixG()const noexcept{return gConstDiag_;}
    };

    /**
     * State for computing RHS
     */
    struct SystemState{
        State state_;
        DirectFormParams params_;
    };

}

#endif //CONTROLALGORITHMS_DIRECTFORMPARAMS_HPP
