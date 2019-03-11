#ifndef UTILITY_MOTION_FORWARDKINEMATICS_HPP
#define UTILITY_MOTION_FORWARDKINEMATICS_HPP

#include <vector>

namespace utility {
namespace motion {
    namespace kinematics {

        // using LimbID  = utility::input::LimbID;
        // using ServoID = utility::input::ServoID;

        inline std::map<ServoID, utility::math::matrix::Transform3D> calculateLegJointPosition() { return positions; }

        inline std::map<ServoID, utility::math::matrix::Transform3D> calculatePosition() {}
    }  // namespace kinematics

}  // namespace motion
}  // namespace utility
}  // namespace utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_HPP
