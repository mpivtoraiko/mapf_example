#pragma once

#include <ostream>
#include <unordered_map>
#include <vector>

#include "planner_starter_code.hpp"

/**
 * A simple space-time state
 */
struct RobotState {
   std::size_t m_x;
   std::size_t m_y;
   std::size_t m_time;

   inline bool operator==(const RobotState &other) const {
      return m_x == other.m_x && m_y == other.m_y && m_time == other.m_time;
   }

   friend std::ostream &operator<<(std::ostream &os,
                                   RobotState const &robot_state);
};

template <> struct std::hash<RobotState> {
   std::size_t operator()(const RobotState &state_input) const {
      return std::hash<std::size_t>()(state_input.m_x) ^
             std::hash<std::size_t>()(state_input.m_y) ^
             std::hash<std::size_t>()(state_input.m_time);
   }
};

/**
 * Point based paths
 */
typedef std::vector<Point> Path;
typedef std::vector<Path> PathSet;
std::ostream &operator<<(std::ostream &os, const Path &path);
std::ostream &operator<<(std::ostream &os, const PathSet &path_set);

/**
 * RobotState based paths (time-based)
 */
typedef std::vector<RobotState> Trajectory;
typedef std::vector<Trajectory> TrajectorySet;
std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory);

/**
 * A representation of trajectory conflicts
 */
typedef std::unordered_map<std::size_t, Trajectory> TrajectoryConflictMap;
std::ostream &operator<<(std::ostream &os,
                         TrajectoryConflictMap const &conflict_set);
void conflict_map_append(size_t bot_idx, const RobotState &robot_state,
                         TrajectoryConflictMap &conflict_map);

/**
 * A best-first search tree node
 */
class CBSTreeNode {
 public:
   CBSTreeNode() : m_total_cost(0) {}
   CBSTreeNode(const TrajectoryConflictMap &conflicts, std::size_t cost)
       : m_conflicts(conflicts), m_total_cost(cost) {}

   inline const TrajectoryConflictMap &get_conflicts() const {
      return m_conflicts;
   }

   inline void set_cost(std::size_t cost) { m_total_cost = cost; }

   friend bool operator>(CBSTreeNode const &lhs, CBSTreeNode const &rhs) {
      return lhs.m_total_cost > rhs.m_total_cost;
   }

   friend std::ostream &operator<<(std::ostream &os, const CBSTreeNode &node);

 private:
   TrajectoryConflictMap m_conflicts;
   std::size_t m_total_cost;
};

/**
 * The Conflict Based Search main class
 */
class ConflictBasedSearch {
 public:
   ConflictBasedSearch(std::shared_ptr<Grid> grid, std::size_t num_robots)
       : m_num_robots(num_robots), m_grid(grid) {}

   std::size_t search(const std::vector<Point> &start_positions,
                      const std::vector<Point> &goal_positions,
                      PathSet &output_paths);

   std::size_t grid_search(const std::vector<Point> &start_positions,
                           const std::vector<Point> &goal_positions,
                           const TrajectoryConflictMap &conflicts,
                           PathSet &state_plans);

   void print_plans(const TrajectorySet &state_plans) const;

   void pathset2trajset(const PathSet &path_set, TrajectorySet &traj_set);

   void generate_combinations(
       const TrajectoryConflictMap &conflicts,
       std::vector<TrajectoryConflictMap> &conflict_combinations)
       const; // a recursive f-n that generates combinations
   std::size_t check_conflicts(const PathSet &paths,
                               TrajectoryConflictMap &conflicts);

 protected:
   std::size_t m_num_robots;
   std::shared_ptr<Grid> m_grid;
};
