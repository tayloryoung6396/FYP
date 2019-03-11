#ifndef UTILITY_MOTION_INVERSEKINEMATICS_HPP
#define UTILITY_MOTION_INVERSEKINEMATICS_HPP

#include <vector>

#include "utility/motion/ForwardKinematics.hpp"

namespace utility {
namespace motion {
    namespace kinematics {

        // using LimbID  = utility::input::LimbID;
        // using ServoID = utility::input::ServoID;

        bool legPoseValid();

        std::vector<std::pair<ServoID, float>> calculateLegJoints();


    }  // namespace kinematics
}  // namespace motion
}  // namespace utility

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_HPP
