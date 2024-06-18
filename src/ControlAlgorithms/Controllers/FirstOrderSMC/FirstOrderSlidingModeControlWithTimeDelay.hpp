//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#ifndef CONTROLALGORITHMS_FIRSTORDERSLIDINGMODECONTROLWITHTIMEDELAY_HPP
#define CONTROLALGORITHMS_FIRSTORDERSLIDINGMODECONTROLWITHTIMEDELAY_HPP

#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include "ControlAlgorithms/Utils/MathFunctions.hpp"

namespace ControlAlgorithms::Controllers {

    template<unsigned N>
    class FirstOrderSMCTimeDelay {
        MatrixNd<N> lambdaMatrix_;
        MatrixNd<N> KMatrix_;
        VectorNd<N> controlAction_;
        double phi_;

    public:
        FirstOrderSMCTimeDelay(const MatrixNd<N> &lambdaMatrix, const MatrixNd<N> &kMatrix,
                               const VectorNd<N> controlAction, const double phi)
                : lambdaMatrix_(lambdaMatrix), KMatrix_(kMatrix), controlAction_(controlAction), phi_(phi) {};

        /** Check reaching sliding surface condition
         *
         * @param S
         * @param K
         * @return
         */
        [[nodiscard]] bool reachingConditionCheck(const VectorNd<N> &S, const MatrixNd<N> &K) const;

        [[nodiscard]] VectorNd<N> chatteringAvoidance(const VectorNd<N> &S, const double phi) const;

        /** Sliding surface calculation
        *
        * @param trackingPositionError
        * @param trackingVelocityError
        * @param lambdaDiagMatrix
        * @return
        */
        [[nodiscard]] VectorNd<N> createSwitchingFunction(const VectorNd<N> &trackingPositionError,
                                                          const VectorNd<N> &trackingVelocityError) const;

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
        void computeControl(const MatrixNd<N> &gConstDiagInv,
                            const VectorNd<N> &prevAcceleration,
                            const VectorNd<N> &currentDesiredAcceleration,
                            const VectorNd<N> &trackingPositionError,
                            const VectorNd<N> &trackingVelocityError);

        [[nodiscard]] VectorNd<N> getControl() const noexcept {return controlAction_;}
    };

}// namespace ControlAlgorithms::Controllers

#endif//CONTROLALGORITHMS_FIRSTORDERSLIDINGMODECONTROLWITHTIMEDELAY_HPP
