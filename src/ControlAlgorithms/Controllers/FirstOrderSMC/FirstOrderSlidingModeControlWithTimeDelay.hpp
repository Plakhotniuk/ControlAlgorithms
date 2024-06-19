//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#pragma once

#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include "ControlAlgorithms/Utils/MathFunctions.hpp"

namespace ControlAlgorithms::Controllers {

    class FirstOrderSMCTimeDelay {
        Matrix2d lambdaMatrix_;
        Matrix2d KMatrix_;
        double phi_;
        double maxControlValue_;
        Vector2d controlAction_ = Vector2d::Zero();

    public:
        FirstOrderSMCTimeDelay(const Matrix2d &lambdaMatrix, const Matrix2d &kMatrix,
                               const double phi, const double maxControlValue)
                : lambdaMatrix_(lambdaMatrix), KMatrix_(kMatrix), phi_(phi), maxControlValue_(maxControlValue) {};

        /** Check reaching sliding surface condition
         *
         * @param S
         * @param K
         * @return
         */
        [[nodiscard]] bool reachingConditionCheck(const Vector2d &S, const Matrix2d &K) const;

        static void chatteringAvoidance(Vector2d &S, const double phi);

        /** Sliding surface calculation
        *
        * @param trackingPositionError
        * @param trackingVelocityError
        * @param lambdaDiagMatrix
        * @return
        */
        [[nodiscard]] Vector2d createSwitchingFunction(const Vector2d &trackingPositionError,
                                                       const Vector2d &trackingVelocityError) const;

        /** Control computation (can be used with unknown disturbance in system state)
         *
         * @param gConstDiagInv
         * @param desiredTrajectoryVelocity
         * @param trackingPositionError
         * @param trackingVelocityError
         * @param prevStepControl
         * @param currentAcceleration
         * @return
         */
        void computeControl(const Matrix2d &gConstDiagInv,
                            const Vector2d &prevAcceleration,
                            const Vector2d &currentDesiredAcceleration,
                            const Vector2d &trackingPositionError,
                            const Vector2d &trackingVelocityError);

        [[nodiscard]] Vector2d getControl() const {
            return {std::clamp(controlAction_(0), -maxControlValue_, maxControlValue_),
                    std::clamp(controlAction_(1), -maxControlValue_, maxControlValue_)};
        }

    };

}// namespace ControlAlgorithms::Controllers
