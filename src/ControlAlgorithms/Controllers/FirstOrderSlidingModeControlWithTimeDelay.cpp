//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#include "FirstOrderSlidingModeControlWithTimeDelay.hpp"

namespace ControlAlgorithms::Controllers {

    template<unsigned N>
    bool FirstOrderSMCTimeDelay<N>::reachingConditionCheck(const VectorNd<N> &S, const MatrixNd<N> &K) const{
        const Vector2d signS = S.unaryExpr(&MathFunc::sgn<double>);
        const Vector2d Sdot = -K * signS;
        return signS.dot(Sdot) < 0;
    }

    template<unsigned N>
    VectorNd<N> FirstOrderSMCTimeDelay<N>::chatteringAvoidance(const VectorNd<N> &S, const double phi) const{
        return S.unaryExpr([&phi](double x) { return std::abs(x) > phi ? MathFunc::sgn(x) : x / phi; });
    }

    template<unsigned N>
    [[nodiscard]] VectorNd<N> FirstOrderSMCTimeDelay<N>::createSwitchingFunction(const VectorNd<N> &trackingPositionError,
                                                      const VectorNd<N> &trackingVelocityError) const{
        return trackingVelocityError + lambdaMatrix_ * trackingPositionError;
    }

    template<unsigned N>
    [[nodiscard]] VectorNd<N> FirstOrderSMCTimeDelay<N>::computeControl(const MatrixNd<N> &gConstDiagInv, const VectorNd<N> &h,
                                                                        const VectorNd<N> &desiredTrajectoryVelocity, const VectorNd<N> &trackingPositionError,
                                                                        const VectorNd<N> &trackingVelocityError) const{
        VectorNd<N> S = createSwitchingFunction(trackingPositionError, trackingVelocityError, lambdaMatrix_);
        assert(reachingConditionCheck(S, KMatrix_));
        chatteringAvoidance(S, phi_);

        return gConstDiagInv *
               (-h + desiredTrajectoryVelocity - lambdaMatrix_ * trackingVelocityError - KMatrix_ * S);
    }

    template<unsigned N>
    [[nodiscard]] VectorNd<N> FirstOrderSMCTimeDelay<N>::computeControl(const SystemStateType &systemState,
                                                                        const DesiredStateType &desiredState) const{
        const VectorNd<N> trackingPositionError = systemState
        Vector2d S = createSwitchingFunction(trackingPositionError, trackingVelocityError, lambdaDiagMatrix);
        assert(reachingConditionCheck(S, K));

        for (int i = 0; i < S.size(); ++i) {
            S(i) = Utils::Math::sgn(S(i));
        }
        chatteringAvoidance(S, phi);
        const Vector2d acceleration = (prevStepVelocity - prevStepVelocity) / step;
        return prevStepControl +
               params.constantSystemParams.gConstDiag.inverse() *
               (-acceleration + desiredTrajectoryVelocity - lambdaDiagMatrix * trackingVelocityError - K * S);
    }
}
