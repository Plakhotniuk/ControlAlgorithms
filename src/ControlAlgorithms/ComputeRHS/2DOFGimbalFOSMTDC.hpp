//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP
#define CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP
#include "ControlAlgorithms/Controllers/FirstOrderSlidingModeControlWithTimeDelay.hpp"
#include "ControlAlgorithms/ControlObjects/2DOFGimbal/DirectFormParams.hpp"
namespace ControlAlgorithms::ComputeRHS{

    struct GimbalFOSMTDC{
        [[nodiscard]] static Vector4d computeRHS(const ControlObjects::Params::SystemState &state, )
    };

}

#endif //CONTROLALGORITHMS_2DOFGIMBALFOSMTDC_HPP
