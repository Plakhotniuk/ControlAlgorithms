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
        double phi_;

    public:
        explicit FirstOrderSMCTimeDelay(const MatrixNd<N> &lambdaMatrix, const MatrixNd<N> &kMatrix, const double phi)
                : lambdaMatrix_(
                lambdaMatrix), KMatrix_(kMatrix), phi_(phi) {};

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

        /** Control calculation
         *
         * @param gConstDiagInv
         * @param h
         * @param desiredTrajectoryVelocity
         * @param trackingPositionError
         * @param trackingVelocityError
         * @param phi
         * @return
         */
        [[nodiscard]] VectorNd<N> computeControl(const MatrixNd<N> &gConstDiagInv, const VectorNd<N> &h,
                                                 const VectorNd<N> &desiredTrajectoryVelocity,
                                                 const VectorNd<N> &trackingPositionError,
                                                 const VectorNd<N> &trackingVelocityError) const;
    };

}// namespace ControlAlgorithms::Controllers

#endif//CONTROLALGORITHMS_FIRSTORDERSLIDINGMODECONTROLWITHTIMEDELAY_HPP
