//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#ifndef CONTROLALGORITHMS_BASICTYPES_HPP
#define CONTROLALGORITHMS_BASICTYPES_HPP
#include "Eigen/Core"

namespace ControlAlgorithms{
    using Matrix2d = Eigen::Matrix<double, 2, 2>;  // Матрица размера 2 на 2 с типом scalar

    using Matrix3d = Eigen::Matrix<double, 3, 3>;  // Матрица размера 3 на 3 с типом scalar

    using Vector2d = Eigen::Matrix<double, 2, 1>;  // Вектор размера 2 с типом scalar

    using Vector3d = Eigen::Matrix<double, 3, 1>;  // Вектор размера 3 с типом scalar

    using Vector4d = Eigen::Matrix<double, 4, 1>;  // Вектор размера 4 с типом scalar

    using Vector5d = Eigen::Matrix<double, 5, 1>;  // Вектор размера 5 с типом scalar

    using Vector6d = Eigen::Matrix<double, 6, 1>;  // Вектор размера 6 с типом scalar

}

#endif //CONTROLALGORITHMS_BASICTYPES_HPP
