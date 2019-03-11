
#include "InverseKinematics.hpp"

namespace utility {
namespace motion {
    namespace kinematics {

        bool legPoseValid() { return (length < maxLegLength); }

        std::vector<std::pair<ServoID, float>> calculateLegJoints() { return positions; }

    }  // namespace kinematics
}  // namespace motion
}  // namespace utility
