//
// Created by Арсений Плахотнюк on 19.06.2024.
//

#pragma once

#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"
#include "ControlAlgorithms/Controllers/PDComputedTorque/PDComputedTorque.hpp"
#include "Trajectory.hpp"

namespace ControlAlgorithms::ComputeRHS::GimbalPDComputedControl {
    class TwoDOFGimbalRHS {
        ControlObjects::TwoDOFGimbal::State state_;
        Controllers::PDComputedTorqueRegulator controller1_;
        Controllers::PDComputedTorqueRegulator controller2_;
        ControlObjects::TwoDOFGimbal::DirectFormParams params_;
        double timeStep_;
        Trajectory trajectory_;

    public:
        TwoDOFGimbalRHS(const ControlObjects::TwoDOFGimbal::State &state,
                        const Controllers::PDComputedTorqueRegulator &controller1,
                        const Controllers::PDComputedTorqueRegulator &controller2,
                        const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                        const double timeStep, const Trajectory &trajectory) : state_(state),
                                                 controller1_(controller1),
                                                 controller2_(controller2),
                                                 params_(params),
                                                 timeStep_(timeStep), trajectory_(trajectory) {};

        void operator()(const ControlObjects::TwoDOFGimbal::State &currentState, ControlObjects::TwoDOFGimbal::State &dxdt,
                        const double t) {
            const Vector2d trackingPositionError =
                    trajectory_.getPosition(t) - currentState.segment<2>(0);

            controller1_.computeControl(trackingPositionError(0), timeStep_);
            controller2_.computeControl(trackingPositionError(1), timeStep_);

            const Vector2d classicPD = {controller1_.getControl(), controller2_.getControl()};

            const Vector2d desiredAcceleration = trajectory_.getAcceleration(t);

            const Vector2d computedTorque = params_.momentOfInertiaMatrix(currentState(1)) *
                                            (desiredAcceleration + classicPD) + params_.gravityVector(currentState(1)) +
                                            params_.frictionForceVector(currentState.segment<2>(2));

            dxdt = ControlObjects::TwoDOFGimbal::computeDynamics(currentState,
                                                                 computedTorque,
                                                                 params_);
            state_ = currentState;
        }

    };
}
