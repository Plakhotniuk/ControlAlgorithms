//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#ifndef CONTROLALGORITHMS_2DOFGIMBALFEEDBACKFORM_HPP
#define CONTROLALGORITHMS_2DOFGIMBALFEEDBACKFORM_HPP

#include "ControlAlgorithms/Utils/BasicTypes.hpp"

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
    struct TwoDOFGimbalFeedbackForm {
        constexpr static unsigned size = 4;
        using Time = double;

        struct ConstantSystemParams {
            double J1;//!< inertia matrix constant
            double J2;//!< inertia matrix constant
            double J3;//!< inertia matrix constant
            double J4;//!< inertia matrix constant
            double Kg;//!< (weight of gimbal) x (distance from center of mass to the axis of rotation)
            double Fs;//!< friction force constant
            Matrix2d gConstDiag;
        };

        /** Calculate gimbal moment of inertia matrix M
         *
         * @param q
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] static Matrix2d calculateMomentOfInertiaMatrix(
                const double &q2, const ConstantSystemParams &constantSystemParams) noexcept {
            Matrix2d M{{constantSystemParams.J1 + constantSystemParams.J2 * std::cos(q2) * std::cos(q2) +
                        constantSystemParams.J3 * std::sin(q2) * std::sin(q2),
                               0.},
                       {0.,    constantSystemParams.J4}};
            return M;
        }

        /** Calculate Coriolis, Centrifugal, Giroscope Forces matrix
         *
         * @param q2
         * @param q1Dot
         * @param q2Dot
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] static Matrix2d calculateCoriolisCentrifugalGiroscopeForcesMatrix(
                const double &q2, const double q1Dot, const double q2Dot,
                const ConstantSystemParams &constantSystemParams) noexcept {
            Matrix2d C{
                    {q2Dot * (constantSystemParams.J2 - constantSystemParams.J3) * std::sin(q2) * std::cos(q2),
                            q1Dot * (constantSystemParams.J2 - constantSystemParams.J3) * std::sin(q2) * std::cos(q2)},
                    {-q1Dot * (constantSystemParams.J2 - constantSystemParams.J3) * std::sin(q2) * std::cos(q2),
                            0.}};
            return C;
        }

        /** Gravity vector calculation
         *
         * @param q2
         * @param constantSystemParams
         * @return
         */
        [[nodiscard]] static Vector2d calculateGravityVector(const double &q2,
                                                             const ConstantSystemParams &constantSystemParams) noexcept {
            return {0., constantSystemParams.Kg * std::sin(q2)};
        }

        /**
         * System state
         */
        struct State {
            Vector4d x;//!< state vector [x0 = theta, x1 = ksi, x2 = theta', x3 = ksi']
            double t;  //!< time
        };

        /**
         * Parameters for integration
         */
        struct Params {
            ConstantSystemParams constantSystemParams;//!< constant system params
            Vector2d control;                         //!< the armature voltage vector
            Matrix2d M;                               //!< inertia matrix
            Matrix2d C;                               //!< Coriolis matrix
            Vector2d G;                               //!< gravity vector
            Vector2d F;                               //!< friction forces vector

            /**
             * System feedback form for control design
             * x1' = x2
             * x2' = f(x, t) + g(x, t) * u(t)
             *
             * x1' = x2
             * x2' = h(t) + gConstDiag(x, t) * u(t)
             */
            Vector2d x1;//!< angles
            Vector2d x2;//!< angular velocities
            Matrix2d g; //!< the control matrix
            Vector2d f; //!< nonlinear dynamics vector
            Vector2d h;
            Vector2d hMAx;

            /** Update dynamic system params
             *
             * @param stateVector
             * @param controlU
             */
            void updateDynamicSystemParams(const State &stateVector, const Vector2d &controlU) {
                control = controlU;
                M = calculateMomentOfInertiaMatrix(stateVector.x(1), constantSystemParams);
                C = calculateCoriolisCentrifugalGiroscopeForcesMatrix(stateVector.x(1), stateVector.x(2),
                                                                      stateVector.x(3),
                                                                      constantSystemParams);
                G = calculateGravityVector(stateVector.x(1), constantSystemParams);
                F = constantSystemParams.Fs * stateVector.x.segment<2>(2);

                x1 = stateVector.x.segment<2>(0);
                x2 = stateVector.x.segment<2>(2);
                g = M.inverse();
                f = g * (-C * stateVector.x.segment<2>(2) - G - F);
                h = f + (g - constantSystemParams.gConstDiag) * control;
                hMAx = h.norm() > hMAx.norm() ? h : hMAx;
            }
        };

        /** Integrate one step
         *
         * @param stateVector
         * @param dynamicSystemParams
         * @return
         */
        [[nodiscard]] static inline Vector4d calc(const State &stateVector, Params &dynamicSystemParams) {
            Vector2d x2 = dynamicSystemParams.x2;
            Vector2d x3 =
                    dynamicSystemParams.h +
                    dynamicSystemParams.constantSystemParams.gConstDiag * dynamicSystemParams.control;
            return {x2(0), x2(1), x3(0), x3(1)};
        }
    };
}// namespace ControlAlgorithms::ControlObjects

#endif//CONTROLALGORITHMS_2DOFGIMBALFEEDBACKFORM_HPP
