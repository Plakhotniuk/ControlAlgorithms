//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#include "FirstOrderSlidingModeControlWithTimeDelay.hpp"

namespace ControlAlgorithms::Controllers {

    
    bool FirstOrderSMCTimeDelay::reachingConditionCheck(const Vector2d &S, const Matrix2d &K) const {
        const Vector2d signS = S.unaryExpr([](const double &x)-> double{return MathFunc::sgn<double>(x);});
        const Vector2d Sdot = -K * signS;
        return signS.dot(Sdot) < 0;
    }

    void FirstOrderSMCTimeDelay::chatteringAvoidance(Vector2d &S, const double phi) {
        S = S.unaryExpr([&phi](double x) { return std::abs(x) > phi ? MathFunc::sgn(x) : x / phi; });
    }

    
    [[nodiscard]] Vector2d FirstOrderSMCTimeDelay::createSwitchingFunction(const Vector2d &trackingPositionError,
                                                                                 const Vector2d &trackingVelocityError) const {
        return trackingVelocityError + lambdaMatrix_ * trackingPositionError;
    }

    
    void FirstOrderSMCTimeDelay::computeControl(const Matrix2d &gConstDiagInv,
                                                   const Vector2d &prevAcceleration,
                                                   const Vector2d &currentDesiredAcceleration,
                                                   const Vector2d &trackingPositionError,
                                                   const Vector2d &trackingVelocityError) {
        Vector2d S = createSwitchingFunction(trackingPositionError, trackingVelocityError);
        assert(reachingConditionCheck(S, KMatrix_));
        chatteringAvoidance(S, phi_);
        const Vector2d signS = S.unaryExpr([](const double &x)-> double{return MathFunc::sgn<double>(x);});

        controlAction_ += gConstDiagInv * (-prevAcceleration + currentDesiredAcceleration -
                                           lambdaMatrix_ * trackingVelocityError - KMatrix_ * signS);
    };

}
