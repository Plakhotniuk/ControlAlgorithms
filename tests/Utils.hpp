//
// Created by Арсений Плахотнюк on 16.06.2024.
//

#ifndef CONTROLALGORITHMS_UTILS_HPP
#define CONTROLALGORITHMS_UTILS_HPP

#include <string>

#define PROJECT_DIR std::string(PROJECT_DIR_RAW) + "/"

namespace tests::Utils{

    template<typename stateType, typename trajectoryType>
    void fileDrop(std::fstream &file, const stateType &state, const trajectoryType &trajectory, const double time){
        file << std::setprecision(10) << state.transpose() << " " << trajectory.getState(time).transpose() << " "
             << time << "\n";
    }

}

#endif //CONTROLALGORITHMS_UTILS_HPP
