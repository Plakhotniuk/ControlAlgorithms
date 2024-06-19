//
// Created by Арсений Плахотнюк on 15.06.2024.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

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

    using Vector8d = Eigen::Matrix<double, 8, 1>;// Вектор размера 8 с типом double

    template<unsigned N>
    using VectorNd = Eigen::Matrix<double, N, 1>;// Вектор размера N с типом double

    template<typename Type, int Rows, int MaxRows = Rows>
    using Vector =
            Eigen::Matrix<Type, static_cast<int>(Rows), 1, 0, static_cast<int>(MaxRows), 1>;  // Базовый класс векторов

    template<typename Type, int Rows, int Cols>
    using Matrix = Eigen::Matrix<Type, Rows, Cols>;

}// namespace ControlAlgorithms
