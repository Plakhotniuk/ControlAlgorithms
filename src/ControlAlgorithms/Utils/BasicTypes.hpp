//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#ifndef CONTROLALGORITHMS_BASICTYPES_HPP
#define CONTROLALGORITHMS_BASICTYPES_HPP

#include "Eigen/Core"

namespace ControlAlgorithms {
    using Matrix2d = Eigen::Matrix<double, 2, 2>;// Матрица размера 2 на 2 с типом double

    using Matrix3d = Eigen::Matrix<double, 3, 3>;// Матрица размера 3 на 3 с типом double

    template<unsigned N>
    using MatrixNd = Eigen::Matrix<double, N, N>;// Матрица размера N на N с типом double

    using Vector2d = Eigen::Matrix<double, 2, 1>;// Вектор размера 2 с типом double

    using Vector3d = Eigen::Matrix<double, 3, 1>;// Вектор размера 3 с типом double

    using Vector4d = Eigen::Matrix<double, 4, 1>;// Вектор размера 4 с типом double

    using Vector5d = Eigen::Matrix<double, 5, 1>;// Вектор размера 5 с типом double

    using Vector6d = Eigen::Matrix<double, 6, 1>;// Вектор размера 6 с типом double

    template<unsigned N>
    using VectorNd = Eigen::Matrix<double, N, 1>;// Вектор размера N с типом double

}// namespace ControlAlgorithms

#endif//CONTROLALGORITHMS_BASICTYPES_HPP
