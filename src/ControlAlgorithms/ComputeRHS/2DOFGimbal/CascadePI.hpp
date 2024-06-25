//
// Created by Арсений Плахотнюк on 19.06.2024.
//

#pragma once


#include "ControlAlgorithms/ControlObjects/2DOFGimbal/TwoDOFGimbalDynamics.hpp"
#include "ControlAlgorithms/Controllers/PID/PID.hpp"
#include "Trajectory.hpp"

namespace ControlAlgorithms::ComputeRHS::GimbalCascadePI {
    class TwoDOFGimbalRHS {
        ControlObjects::TwoDOFGimbal::State state_;
        Controllers::PID positionController1_;
        Controllers::PID rateController1_;
        Controllers::PID positionController2_;
        Controllers::PID rateController2_;
        ControlObjects::TwoDOFGimbal::DirectFormParams params_;
        double timeStep_;
        Trajectory desiredTrajectory_;
    public:
        TwoDOFGimbalRHS(const ControlObjects::TwoDOFGimbal::State &state,
                        const Controllers::PID &positionController1,
                        const Controllers::PID &rateController1,
                        const Controllers::PID &positionController2,
                        const Controllers::PID &rateController2,
                        const ControlObjects::TwoDOFGimbal::DirectFormParams &params,
                        const double timeStep, const Trajectory trajectory) : state_(state),
                                                                              positionController1_(positionController1),
                                                                              rateController1_(rateController1),
                                                                              positionController2_(positionController2),
                                                                              rateController2_(rateController2),
                                                                              params_(params),
                                                                              timeStep_(timeStep),
                                                                              desiredTrajectory_(trajectory) {};

        void operator()(const ControlObjects::TwoDOFGimbal::State &currentState, ControlObjects::TwoDOFGimbal::State &dxdt,
                        const double t) {
            const Vector2d trackingPositionError =
                    desiredTrajectory_.getPosition(t) - currentState.segment<2>(0);

            positionController1_.computeControl(trackingPositionError(0), timeStep_);
            rateController1_.computeControl(positionController1_.getControl() - currentState(2), timeStep_);

            positionController2_.computeControl(trackingPositionError(1), timeStep_);
            rateController2_.computeControl(positionController2_.getControl() - currentState(3), timeStep_);



            dxdt = ControlObjects::TwoDOFGimbal::computeDynamics(currentState,
                                                                 Vector2d{rateController1_.getControl(),
                                                                  rateController2_.getControl()},
                                                                 params_);
            state_ = currentState;
        }

    };
}
