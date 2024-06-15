//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#ifndef CONTROLALGORITHMS_MATHFUNCTIONS_HPP
#define CONTROLALGORITHMS_MATHFUNCTIONS_HPP

namespace ControlAlgorithms::MathFunc{
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}

#endif //CONTROLALGORITHMS_MATHFUNCTIONS_HPP
