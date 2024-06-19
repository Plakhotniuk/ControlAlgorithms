//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#pragma once

#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include <random>

namespace ControlAlgorithms::ControlObjects::TwoDOFGimbal {

    /**
     * System state: state vector, desired state vector, time
     */
    typedef Vector8d State; //!< [x0 = theta, x1 = ksi, x2 = theta', x3 = ksi',
                            //!< x4 = theta_d, x5 = ksi_d, x6 = theta'_d, x7 = ksi'_d]

    struct push_back_state_and_time {
        std::vector<State> &m_states;
        std::vector<double> &m_times;

        push_back_state_and_time(std::vector<State> &states, std::vector<double> &times)
                : m_states(states), m_times(times) {}

        void operator()(const State &x, double t) {
            m_states.push_back(x);
            m_times.push_back(t);
        }
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
        Matrix2d disturbanceSigma_;
        std::mt19937 &randomEngine_;

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

        [[nodiscard]] Matrix2d getConstDiagMatrixG() const noexcept { return gConstDiag_; }
    };

}
