#pragma once

#include <vector>
#include <unordered_map>

#include "planner_starter_code.hpp"

/**
 * A simple space-time state
 */
struct RobotState {
   std::size_t m_x;
   std::size_t m_y;
   std::size_t m_time;

   inline bool operator==(const RobotState &other) const {
      return m_x == other.m_x && m_y == other.m_y;
   }

   inline Point to_point() {return Point({int(m_x), int(m_y)});}
};

template <> struct std::hash<RobotState> {
   std::size_t operator()(const RobotState &state_input) const {
      return std::hash<std::size_t>()(state_input.m_x) ^
             std::hash<std::size_t>()(state_input.m_y) ^
             std::hash<std::size_t>()(state_input.m_time);
   }
};


typedef std::vector<std::vector<RobotState>> PathSet;
typedef std::tuple<RobotState, std::vector<std::size_t>> PathConflict;

class CBSTreeNode {
 public:
   CBSTreeNode() {}
   CBSTreeNode(const std::vector<PathConflict> & conflicts) : m_conflicts = conflicts {}

   std::size_t check_conflicts();
   void generate_combinations(std::vector<std::vector<PathConflict>> &conflict_combinations); // a recursive f-n that generates combinations

   PathSet & get_paths() {return m_paths;} // return a reference to paths to be set by the planner

   //inline bool no_conflicts() {return m_conflicts.size() == 0;}
   inline const std::vector<PathConflict> & get_conflicts() {return m_conflicts;}

 private:
   PathSet m_paths;
   std::vector<PathConflict> m_conflicts;
   std::size_t m_total_cost;
   //std::vector<std::shared_ptr<CBSTreeNode>> m_children;
   //std::shared_ptr<Grid> m_grid;
};


class ConflictBasedSearch {
 public:
   ConflictBasedSearch(std::shared_ptr<Grid> grid, std::size_t num_robots) : m_num_robots(num_robots),
                                                                             m_grid(grid) {}

   void set_goals(const std::vector<RobotState> &robot_goals);

   void search(const std::vector<Point> &start_positions,
                                const std::vector<Point> &goal_positions);

   void grid_search(const std::vector<Point> &start_positions, 
             const std::vector<Point> &goal_positions,
             const std::vector<PathConflict> &conflicts,
             std::vector<std::vector<RobotState>> &state_plans);


   void print_plans(const std::vector<std::vector<RobotState>> &state_plans);

 protected:
   // void enumerate_conflicts

   //         std::unordered_map < std::size_t,
   //     vector<RobotState> m_vertices;

   std::size_t m_num_robots;
   // std::vector<RobotState> m_robot_goals;


   std::shared_ptr<Grid> m_grid;
};