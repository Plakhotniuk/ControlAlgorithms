//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#pragma once

#include "ControlAlgorithms/Controllers/FirstOrderSMC/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"
#include "Trajectory.hpp"
#include "third_party/eigen/Eigen/LU"

namespace ControlAlgorithms::ComputeRHS::GimbalFOSMTDC {

    class TwoDOFGimbalRHS {
        ControlObjects::TwoDOFGimbal::State state_;
        Controllers::FirstOrderSMCTimeDelay controller_;
        ControlObjects::TwoDOFGimbal::DirectFormParams params_;
        double timeStep_;
        Trajectory desiredTrajectory_;


    public:
        TwoDOFGimbalRHS(const ControlObjects::TwoDOFGimbal::State &state,
                        const Controllers::FirstOrderSMCTimeDelay &controller,
                        const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                        const double timeStep, const Trajectory &desiredTraj) : state_(state),
                                                                                controller_(controller),
                                                                                params_(params),
                                                                                timeStep_(timeStep),
                                                                                desiredTrajectory_(desiredTraj) {};

        void operator()(const ControlObjects::TwoDOFGimbal::State &currentState, ControlObjects::TwoDOFGimbal::State &dxdt,
                        const double t) {

            const Vector2d desiredPosition = desiredTrajectory_.getPosition(t);

            const Vector2d trackingPositionError =
                    currentState.segment<2>(0) - desiredTrajectory_.getPosition(t);
            const Vector2d trackingVelocityError =
                    currentState.segment<2>(2) - desiredTrajectory_.getVelocity(t);
            const Vector2d prevAcceleration =
                    (currentState.segment<2>(2) - state_.segment<2>(2)) / timeStep_;
            const Vector2d currentDesiredAcceleration = desiredTrajectory_.getAcceleration(t);
            controller_.computeControl(params_.gConstDiag_.inverse(), prevAcceleration,
                                       currentDesiredAcceleration, trackingPositionError, trackingVelocityError);

//            const Vector4d disturbanceVector = Random::getVectorNoise<double, 4>(params_.randomEngine_, params_.disturbanceSigma_);

            dxdt = ControlObjects::TwoDOFGimbal::computeDynamics(currentState, controller_.getControl(), params_);
            state_ = currentState;
        }
    };

}
