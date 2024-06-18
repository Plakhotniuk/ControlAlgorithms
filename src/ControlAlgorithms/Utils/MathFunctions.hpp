//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#pragma once

namespace ControlAlgorithms::MathFunc{
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}
