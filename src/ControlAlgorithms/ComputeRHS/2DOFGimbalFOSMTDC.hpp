//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP
#define CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP

#include "ControlAlgorithms/Controllers/FirstOrderSMC/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"

namespace ControlAlgorithms::ComputeRHS::GimbalFOSMTDC {

    /** Compute RHS
     *
     * @param currentState
     * @param prevState
     * @param controller
     * @param params
     * @param timeStep
     * @return
     */
    [[nodiscard]] static inline Vector4d computeRHS(const ControlObjects::TwoDOFGimbal::State &currentState,
                                                    const ControlObjects::TwoDOFGimbal::State &prevState,
                                                    Controllers::FirstOrderSMCTimeDelay<2> &controller,
                                                    const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                                                    const double timeStep) {
        const Vector2d trackingPositionError =
                currentState.x.segment<2>(0) - currentState.xd.segment<2>(0);
        const Vector2d trackingVelocityError =
                currentState.x.segment<2>(2) - currentState.xd.segment<2>(2);
        const Vector2d prevAcceleration =
                (currentState.x.segment<2>(2) - prevState.x.segment<2>(2)) / timeStep;
        const Vector2d currentDesiredAcceleration =
                (currentState.xd.segment<2>(2) - prevState.xd.segment<2>(2)) / timeStep;
        controller.computeControl(params.gConstDiag_.inverse(), prevAcceleration,
                                  currentDesiredAcceleration, trackingPositionError, trackingVelocityError);

        return ControlObjects::TwoDOFGimbal::computeDynamics(currentState, controller.getControl(), params);
    }

    class TwoDOFGimbalRHS {
        ControlObjects::TwoDOFGimbal::State state_;
        Controllers::FirstOrderSMCTimeDelay<2> controller_;
        ControlObjects::TwoDOFGimbal::DirectFormParams params_;
        double timeStep_;

    public:
        TwoDOFGimbalRHS(const ControlObjects::TwoDOFGimbal::State &state,
                        const Controllers::FirstOrderSMCTimeDelay<2> &controller,
                        const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                        const double timeStep) : state_(state),
                                                 controller_(controller),
                                                 params_(params),
                                                 timeStep_(timeStep) {};

        void operator()(const ControlObjects::TwoDOFGimbal::State &currentState, ControlObjects::TwoDOFGimbal::State &dxdt,
                        const double /* t */) {
            const Vector2d trackingPositionError =
                    currentState.x.segment<2>(0) - currentState.xd.segment<2>(0);
            const Vector2d trackingVelocityError =
                    currentState.x.segment<2>(2) - currentState.xd.segment<2>(2);
            const Vector2d prevAcceleration =
                    (currentState.x.segment<2>(2) - state_.x.segment<2>(2)) / timeStep_;
            const Vector2d currentDesiredAcceleration =
                    (currentState.xd.segment<2>(2) - state_.xd.segment<2>(2)) / timeStep_;
            controller_.computeControl(params_.gConstDiag_.inverse(), prevAcceleration,
                                      currentDesiredAcceleration, trackingPositionError, trackingVelocityError);

            dxdt = ControlObjects::TwoDOFGimbal::computeDynamics(currentState, controller_.getControl(), params_);

        }
    };

}

#endif //CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP
