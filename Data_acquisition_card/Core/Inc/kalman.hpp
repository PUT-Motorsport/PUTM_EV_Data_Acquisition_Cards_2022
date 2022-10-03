#pragma once

#include <type_traits>

template<typename DataType>
requires(std::is_floating_point<DataType>::value)
class KalmanFilter1D {
public:
    explicit constexpr KalmanFilter1D(DataType initialGuess, DataType initialVariance) : 
        F{}, H{}, R{}, Q{}, x_nn{initialGuess}, variance_nn{initialVariance} {}

    DataType iterate(DataType measurement) {
        //prediction phase
        DataType const x_predicted = F * x_nn;
        DataType const variance_predicted = variance_nn + Q;

        //update / correction phase phase
        DataType const KalmanGain = variance_predicted / (variance_predicted + R);
        x_nn = x_predicted + KalmanGain * (measurement - x_predicted);
        variance_nn = (1 - KalmanGain) * variance_predicted;

        return x_nn;
    }
    [[nodiscard]] DataType get() const {return x_nn;}

    DataType F, H, R, Q;    //may be modified inbetween iterations
private:

    DataType x_nn;
    DataType variance_nn;

};
