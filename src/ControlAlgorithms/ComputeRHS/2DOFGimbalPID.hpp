//
// Created by Арсений Плахотнюк on 18.06.2024.
//

#pragma once

#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"
#include "ControlAlgorithms/Controllers/PID/PID.hpp"

namespace ControlAlgorithms::ComputeRHS::GimbalPID {
    class TwoDOFGimbalRHS {
        ControlObjects::TwoDOFGimbal::State state_;
        Controllers::PID controller1_;
        Controllers::PID controller2_;
        ControlObjects::TwoDOFGimbal::DirectFormParams params_;
        double timeStep_;
    public:
        TwoDOFGimbalRHS(const ControlObjects::TwoDOFGimbal::State &state,
                        const Controllers::PID &controller1,
                        const Controllers::PID &controller2,
                        const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                        const double timeStep) : state_(state),
                                                 controller1_(controller1),
                                                 controller2_(controller2),
                                                 params_(params),
                                                 timeStep_(timeStep) {};

        void operator()(const ControlObjects::TwoDOFGimbal::State &currentState, ControlObjects::TwoDOFGimbal::State &dxdt,
                        const double /* t */) {
            const Vector2d trackingPositionError =
                    currentState.segment<2>(4) - currentState.segment<2>(0);

            controller1_.computeControl(trackingPositionError(0), timeStep_);
            controller2_.computeControl(trackingPositionError(1), timeStep_);

            dxdt = ControlObjects::TwoDOFGimbal::computeDynamics(currentState,
                                                                 {controller1_.getControl(), controller2_.getControl()},
                                                                 params_);
            state_ = currentState;
        }

    };
}