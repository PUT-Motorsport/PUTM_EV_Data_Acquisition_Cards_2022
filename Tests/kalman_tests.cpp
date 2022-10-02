#include "../Data_acquisition_card/Core/Inc/kalman.hpp"

#include <iostream>
#include <fstream>
#include <string>
/*
 * Open test_data.csv, parse all entries through the Kalman filter, save the results to results.csv to be plotted later.
*/

int main() {

    //Open the test_data.csv
    std::ifstream test_data;
    test_data.open("../test_data.csv");

    std::ofstream results;
    results.open("./results.csv");

    KalmanFilter1D<double> filter(0, 0);
    filter.H = 1;
    filter.F = 1;
    filter.Q = 0.5;
    filter.R = 10;

    if (test_data.is_open()) {

        std::string line;

        while (std::getline(test_data, line)) {
            double var = std::stod(line.c_str());

            std::cout << var << std::endl;

            results << filter.iterate(var) << std::endl;
        }
    } else {
        std::cout << "Could not open test_data.csv" << std::endl;
    }
    return 0;
}