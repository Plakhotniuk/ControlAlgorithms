//
// Created by Арсений Плахотнюк on 25.02.2023.
//

#pragma once

#include <random>
#include "ControlAlgorithms/Utils/BasicTypes.hpp"
#include "Eigen/Dense"

namespace ControlAlgorithms::Random {


    class MultivariateNormalDistribution {
    private:
        std::mt19937 &randomEngine_;

    public:
        MultivariateNormalDistribution(std::mt19937 &randomEngine) : randomEngine_(randomEngine) {};

        template<typename T, int Size>
        Vector<T, Size> variate(const Vector<T, Size> &mean, const Matrix<T, Size, Size> &covMatrix) {
            // Проверка симметричности

            // Матрица преобразования находится из разложения Холецкого
            Matrix<T, Size, Size> normTransform;
            Eigen::LLT<Eigen::MatrixXd> cholSolver(covMatrix);

            // We can only use the cholesky decomposition if
            // the covariance matrix is symmetric, pos-definite.
            // But a covariance matrix might be pos-semi-definite.
            // In that case, we'll go to an EigenSolver
            if (cholSolver.info() == Eigen::Success) {
                // Use cholesky solver
                normTransform = cholSolver.matrixL();
            } else {
                // Use eigen solver
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covMatrix);
                normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
            }

            // Случайный вектор
            std::normal_distribution dist;
            Eigen::Vector<T, Size> randomVector;
            for (int i = 0; i < Size; ++i) randomVector(i) = dist(randomEngine_);

            return mean + (normTransform * randomVector);
        }
    };

    /** Генерит вектор шумов
     *
     * @tparam Size
     * @param randomEngine
     * @param sigma
     * @return
     */
    template<typename Type, unsigned Size>
    Vector<Type, Size> getVectorNoise(std::mt19937 &randomEngine, const Matrix<Type, Size, Size> &sigma) {
        MultivariateNormalDistribution multivariateNormalDistribution(randomEngine);
        Vector<Type, Size> zeroVec = Vector<Type, Size>::Zero();
        Vector<Type, Size> noiseVector = multivariateNormalDistribution.variate(zeroVec, sigma);
        return noiseVector;
    }


    template<typename Scalar>
    struct RandomRange {
        RandomRange(const Scalar& low, const Scalar& high,
                    std::default_random_engine &gen) : dis(low, high), gen(gen) {}
        const Scalar operator()() const { return dis(gen); }
        mutable std::uniform_int_distribution<> dis;
        std::default_random_engine &gen;
    };

    template<unsigned Size>
    VectorNd<Size> getRandomVector(const double low, const double high, std::default_random_engine &gen){
        VectorNd<Size> vec;
        RandomRange<double> randomRange(low, high, gen);
        for(unsigned i = 0; i < Size; ++i){
            vec(i) = randomRange();
        }
        return vec;
    }

}  // namespace LaserP::RandomDistribution

