#pragma once

#include <ostream>
#include <vector>
#include <boost/container_hash/hash_fwd.hpp>



/**
 * A simple space-time state
 */
struct RobotState {
   RobotState() : m_x(0), m_y(0), m_time(0) {}
   RobotState(std::size_t x, std::size_t y) : m_x(x), m_y(y), m_time(0) {}
   RobotState(const RobotState & other) : m_x(other.m_x), m_y(other.m_y), m_time(other.m_time) {}

   std::size_t m_x;
   std::size_t m_y;
   std::size_t m_time;

   inline bool operator==(const RobotState &other) const {
      return m_x == other.m_x && m_y == other.m_y && m_time == other.m_time;
   }

   friend std::ostream &operator<<(std::ostream &os,
                                   RobotState const &robot_state);
};


// needed by std::hash<Trajectory>()
inline std::size_t hash_value(const RobotState &self)
{
   std::size_t seed = 0;
   boost::hash_combine(seed, self.m_x);
   boost::hash_combine(seed, self.m_y);
   boost::hash_combine(seed, self.m_time);
   return seed;      
}

template <> struct std::hash<RobotState> {
   std::size_t operator()(const RobotState &self) const {
      return hash_value(self);
   }
};


/**
 * RobotState based paths (time-based)
 */
typedef std::vector<RobotState> Trajectory;

std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory);

template <> struct std::hash<Trajectory> {
   std::size_t operator()(const Trajectory &trajectory) const {
      return boost::hash_range(trajectory.begin(), trajectory.end());
   }
};

typedef std::vector<Trajectory> TrajectorySet;