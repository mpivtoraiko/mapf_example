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
typedef std::pair<std::size_t, Trajectory> TrajectoryConflict;
//typedef std::shared_ptr<TrajectoryConflict> TrajectoryConflictPtr;
typedef std::unordered_set<TrajectoryConflict> TrajectoryConflictSet;


template <> struct std::hash<TrajectoryConflict> {
   std::size_t operator()(const TrajectoryConflict &self) const;
};


std::ostream & operator<<(std::ostream& os, TrajectoryConflict const& conflict);
std::ostream & operator<<(std::ostream& os, TrajectoryConflictSet const& conflict_set);

std::ostream & operator<<(std::ostream& os, Path const& path);
std::ostream & operator<<(std::ostream& os, PathSet const& path_set);

class CBSTreeNode {
 public:
   CBSTreeNode(std::size_t num_robots);
   CBSTreeNode(const TrajectoryConflictSet & conflicts, std::size_t cost) : m_conflicts(conflicts), m_total_cost(cost) {}

   // return a reference to (volatile) paths to be set by the planner
   //inline PathSet & get_path_set() {return m_paths;} 

   //inline bool no_conflicts() {return m_conflicts.size() == 0;}
   inline const TrajectoryConflictSet & get_conflicts() const {return m_conflicts;}
   //inline std::size_t get_cost() const {return m_total_cost;}
   inline void set_cost(std::size_t cost) {m_total_cost = cost;}

   friend bool operator>(CBSTreeNode const& lhs, CBSTreeNode const& rhs) {
      return lhs.m_total_cost > rhs.m_total_cost;
   }

   friend std::ostream & operator<<(std::ostream& os, CBSTreeNode const& node);

 private:
   //PathSet m_paths;
   TrajectoryConflictSet m_conflicts;
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
                    const TrajectoryConflictSet &conflicts,
                    PathSet &state_plans);


   void print_plans(const TrajectorySet &state_plans) const;

   void pathset2trajset(const PathSet & path_set, TrajectorySet & traj_set);

   void generate_combinations(const TrajectoryConflictSet &conflicts,
                              std::vector<TrajectoryConflictSet> &conflict_combinations) const; // a recursive f-n that generates combinations
   std::size_t check_conflicts(const PathSet &paths, TrajectoryConflictSet &conflicts);

 protected:

   std::size_t m_num_robots;
   std::shared_ptr<Grid> m_grid;
};