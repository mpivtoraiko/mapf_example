#include "robot_state.h"


std::ostream &operator<<(std::ostream &os, RobotState const &robot_state) {
   os << "(" << robot_state.m_x << ", " << robot_state.m_y << ", "
      << robot_state.m_time << ")";
   return os;
}

std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory) {
   for (const auto &cur_state : trajectory) {
      os << cur_state << " ";
   }
   return os;
}

