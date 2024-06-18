//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#include "FirstOrderSlidingModeControlWithTimeDelay.hpp"

namespace ControlAlgorithms::Controllers {

    template<unsigned N>
    bool FirstOrderSMCTimeDelay<N>::reachingConditionCheck(const VectorNd<N> &S, const MatrixNd<N> &K) const {
        const Vector2d signS = S.unaryExpr(&MathFunc::sgn<double>);
        const Vector2d Sdot = -K * signS;
        return signS.dot(Sdot) < 0;
    }

    template<unsigned N>
    VectorNd<N> FirstOrderSMCTimeDelay<N>::chatteringAvoidance(const VectorNd<N> &S, const double phi) const {
        return S.unaryExpr([&phi](double x) { return std::abs(x) > phi ? MathFunc::sgn(x) : x / phi; });
    }

    template<unsigned N>
    [[nodiscard]] VectorNd<N> FirstOrderSMCTimeDelay<N>::createSwitchingFunction(const VectorNd<N> &trackingPositionError,
                                                                                 const VectorNd<N> &trackingVelocityError) const {
        return trackingVelocityError + lambdaMatrix_ * trackingPositionError;
    }

    template<unsigned N>
    void FirstOrderSMCTimeDelay<N>::computeControl(const MatrixNd<N> &gConstDiagInv,
                                                   const VectorNd<N> &prevAcceleration,
                                                   const VectorNd<N> &currentDesiredAcceleration,
                                                   const VectorNd<N> &trackingPositionError,
                                                   const VectorNd<N> &trackingVelocityError) {
        const Vector2d S = createSwitchingFunction(trackingPositionError, trackingVelocityError, lambdaMatrix_);
        assert(reachingConditionCheck(S, KMatrix_));

        const Vector2d signS = S.unaryExpr(&MathFunc::sgn<double>);
        chatteringAvoidance(S, phi_);

        controlAction_ += gConstDiagInv * (-prevAcceleration + currentDesiredAcceleration -
                                           lambdaMatrix_ * trackingVelocityError - KMatrix_ * signS);
    };

}
