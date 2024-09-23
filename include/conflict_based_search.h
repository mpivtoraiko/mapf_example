#pragma once

#include <vector>
#include <unordered_map>
#include <ostream>

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

   inline Point to_point() {return Point({int(m_x), int(m_y)});}

   friend std::ostream & operator<<(std::ostream& os, RobotState const& robot_state); 
};

template <> struct std::hash<RobotState> {
   std::size_t operator()(const RobotState &state_input) const {
      return std::hash<std::size_t>()(state_input.m_x) ^
             std::hash<std::size_t>()(state_input.m_y) ^
             std::hash<std::size_t>()(state_input.m_time);
   }
};


typedef std::vector<Point> Path;
typedef std::vector<Path> PathSet;
typedef std::vector<RobotState> Trajectory; // a time-based path
typedef std::vector<Trajectory> TrajectorySet;
//typedef std::tuple<RobotState, std::vector<std::size_t>> PathConflict;
//typedef std::pair<std::size_t, Trajectory> TrajectoryConflict;
//typedef std::shared_ptr<TrajectoryConflict> TrajectoryConflictPtr;
typedef std::unordered_map<std::size_t, Trajectory> TrajectoryConflictMap;


// template <> struct std::hash<TrajectoryConflict> {
//    std::size_t operator()(const TrajectoryConflict &self) const;
// };


//std::ostream & operator<<(std::ostream& os, TrajectoryConflict const& conflict);
std::ostream & operator<<(std::ostream& os, TrajectoryConflictMap const& conflict_set);

void conflict_map_append(size_t bot_idx, const RobotState & robot_state, TrajectoryConflictMap &conflict_map)
{
   const auto &bot_key = conflict_map.find(bot_idx);
   if (bot_key == conflict_map.end()) {
   //if (conflict_map.contains(bot_idx)) {
      conflict_map.insert({bot_idx, Trajectory({robot_state})});
   }
   else {
      bot_key->second.push_back(robot_state);
   }
}


std::ostream & operator<<(std::ostream& os, const Path & path);
std::ostream & operator<<(std::ostream& os, const PathSet & path_set);
std::ostream & operator<<(std::ostream& os, const Trajectory & trajectory);

class CBSTreeNode {
 public:
   CBSTreeNode() : m_total_cost(0) {}
   CBSTreeNode(const TrajectoryConflictMap & conflicts, std::size_t cost) : m_conflicts(conflicts), m_total_cost(cost) {}

   // return a reference to (volatile) paths to be set by the planner
   //inline PathSet & get_path_set() {return m_paths;} 

   //inline bool no_conflicts() {return m_conflicts.size() == 0;}
   inline const TrajectoryConflictMap & get_conflicts() const {return m_conflicts;}
   //inline std::size_t get_cost() const {return m_total_cost;}
   inline void set_cost(std::size_t cost) {m_total_cost = cost;}

   friend bool operator>(CBSTreeNode const& lhs, CBSTreeNode const& rhs) {
      return lhs.m_total_cost > rhs.m_total_cost;
   }

   friend std::ostream & operator<<(std::ostream& os, const CBSTreeNode & node);

 private:
   //PathSet m_paths;
   TrajectoryConflictMap m_conflicts;
   std::size_t m_total_cost;
};


class ConflictBasedSearch {
 public:
   ConflictBasedSearch(std::shared_ptr<Grid> grid, std::size_t num_robots) : m_num_robots(num_robots),
                                                                             m_grid(grid) {}

   std::size_t search(const std::vector<Point> &start_positions,
               const std::vector<Point> &goal_positions,
               PathSet &output_paths);

   std::size_t  grid_search(const std::vector<Point> &start_positions, 
                    const std::vector<Point> &goal_positions,
                    const TrajectoryConflictMap &conflicts,
                    PathSet &state_plans);


   void print_plans(const TrajectorySet &state_plans) const;

   void pathset2trajset(const PathSet & path_set, TrajectorySet & traj_set);

   void generate_combinations(const TrajectoryConflictMap &conflicts,
                              std::vector<TrajectoryConflictMap> &conflict_combinations) const; // a recursive f-n that generates combinations
   std::size_t check_conflicts(const PathSet &paths, TrajectoryConflictMap &conflicts);

 protected:

   std::size_t m_num_robots;
   std::shared_ptr<Grid> m_grid;
};


