#include <boost/log/trivial.hpp>
#include <cstdio>
#include <queue>
#include <stdexcept>

#include "conflict_based_search.h"

#define EXCEPTION_MSG_MAX_SIZE 1024
char exception_msg[EXCEPTION_MSG_MAX_SIZE];

#define POINT_TO_STATE(position)                                               \
   (RobotState({std::size_t(position.x), std::size_t(position.y), 0}))
#define STATE_TO_POINT(state) (Point({int(state.m_x), int(state.m_y)}))

using namespace std;

// logger setup
using namespace boost::log::trivial;
boost::log::sources::severity_logger<severity_level> lg;

/*     ----------------     Output Functions     ----------------     */

std::ostream &operator<<(std::ostream &os, Path const &path) {
   for (const auto &cur_pt : path) {
      os << cur_pt << " ";
   }
   return os;
}

std::ostream &operator<<(std::ostream &os, PathSet const &path_set) {
   for (size_t path_idx = 0; path_idx < path_set.size(); ++path_idx) {
      os << path_idx << ": " << path_set[path_idx] << endl;
   }
   return os;
}

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



/*     ----------------     TrajectoryConflictMap     ----------------     */

std::ostream &operator<<(std::ostream &os,
                         TrajectoryConflictMap const &conflict_map) {
   os << "conflicts:" << endl;
   for (const auto &[bot_idx, conflicts] : conflict_map) {
      os << "  " << bot_idx << ": " << conflicts << endl;
   }
   return os;
}

void conflict_map_append(size_t bot_idx, const RobotState &robot_state,
                         TrajectoryConflictMap &conflict_map) {
   const auto &bot_key = conflict_map.find(bot_idx);
   if (bot_key == conflict_map.end()) {
      conflict_map.insert({bot_idx, Trajectory({robot_state})});
   } else {
      bot_key->second.push_back(robot_state);
   }
}

/*     ----------------     CBSTreeNode     ----------------     */

std::ostream &operator<<(std::ostream &os, CBSTreeNode const &node) {
   os << "cost " << node.m_total_cost << endl << node.m_conflicts;
   return os;
}

/*     ----------------     ConflictBasedSearch     ----------------     */

std::size_t
ConflictBasedSearch::grid_search(const std::vector<Point> &start_positions,
                                 const std::vector<Point> &goal_positions,
                                 const TrajectoryConflictMap &conflicts,
                                 PathSet &paths) {
   paths.clear();
   assert(start_positions.size() == m_num_robots);
   assert(goal_positions.size() == m_num_robots);

   paths.resize(m_num_robots);

   size_t total_cost = 0;

   for (size_t bot_idx = 0; bot_idx < m_num_robots; ++bot_idx) {
      // block off cells per conflicts, if any, for each robot
      const auto &bot_key = conflicts.find(bot_idx);
      size_t path_cost = 0;
      if (bot_key == conflicts.end()) {
         // no conflicts for this bot, so just plan without cost map
         // modification
         path_cost = m_grid->get_path(start_positions[bot_idx],
                                      goal_positions[bot_idx], paths[bot_idx]);
      } else {
         // we've got conflicts, implement them in the cost map
         for (const auto &cur_state : bot_key->second) {
            m_grid->setObstacle(STATE_TO_POINT(cur_state));
         }

         // replan the point paths per established constraints
         try {
            path_cost =
                m_grid->get_path(start_positions[bot_idx],
                                 goal_positions[bot_idx], paths[bot_idx]);
         } catch (const std::invalid_argument &exception_obj) {
            path_cost = 0; // planner failed; proceed to map recover, then early
                           // terminate
         }
         // undo robot conflicts
         for (const auto &cur_state : bot_key->second) {
            // not checking for special cells (dig or dropoff locations bc the
            // plan should already avoid them, so they can't be a conflict
            m_grid->unsetObstacle(STATE_TO_POINT(cur_state));
         }

         // early termination in case of planner failure
         if (path_cost == 0) {
            return 0;
         }
      } // else (bot_key == conflicts.end())
      total_cost += path_cost;
   } // for (size_t bot_idx = 0; bot_idx < m_num_robots; ++bot_idx)

   return total_cost;
}

void ConflictBasedSearch::print_plans(const TrajectorySet &state_plans) const {
   char cell_label[10];
   for (int x = 0; x < m_grid->getWidth(); ++x) {
      for (int y = 0; y < m_grid->getHeight(); ++y) {
         Point cur_pt{x, y};
         string cell = " . ";

         // draw plans
         for (auto cur_plan : state_plans) {
            for (auto cur_state : cur_plan) {
               if (cur_state.m_x == size_t(x) && cur_state.m_y == size_t(y)) {
                  snprintf(cell_label, 10, " %zu ", cur_state.m_time);
                  cell = string(cell_label).substr(0, 3);
                  break;
               }
            }
         }

         // draw robots
         for (size_t plan_idx = 0; plan_idx < state_plans.size(); ++plan_idx) {
            if (state_plans[plan_idx][0].m_x == size_t(x) &&
                state_plans[plan_idx][0].m_y == size_t(y)) {
               snprintf(cell_label, 10, "R%zu ", plan_idx + 1);
               cell = string(cell_label).substr(0, 3);
            }
         }

         if (m_grid->isObstacle(cur_pt)) {
            cell = " # ";
         } else if (m_grid->isDigLocation(cur_pt)) {
            cell = " D ";
         } else if (m_grid->isDropoffLocation(cur_pt)) {
            snprintf(cell_label, 10, "|%d|",
                     m_grid->getDropOffLocationValue(cur_pt));
            cell = string(cell_label).substr(0, 3);
         }

         std::cout << cell;
      }
      std::cout << std::endl;
   }
   std::cout << std::endl;
}

std::size_t
ConflictBasedSearch::search(const std::vector<Point> &start_positions,
                            const std::vector<Point> &goal_positions,
                            PathSet &output_paths) {
   output_paths.clear();

   assert(start_positions.size() == m_num_robots);
   assert(goal_positions.size() == m_num_robots);

   // setup the priority queue
   std::priority_queue<CBSTreeNode, vector<CBSTreeNode>,
                       std::greater<CBSTreeNode>>
       search_queue;

   // root node
   PathSet paths;
   size_t node_cost = this->grid_search(start_positions, goal_positions,
                                        TrajectoryConflictMap(), paths);
   BOOST_LOG_SEV(lg, trace) << "Paths:" << endl << paths << endl;

   TrajectoryConflictMap new_conflicts;
   if (this->check_conflicts(paths, new_conflicts) == 0) {
      BOOST_LOG_SEV(lg, debug) << "No conflicts!" << endl;
      // TODO: check if there are ties, and if so, look for the one at
      // shallowest depth (minimize constraints)
      output_paths = paths; // prepare the return value
      return 0;
   }

   BOOST_LOG_SEV(lg, trace) << new_conflicts;
   // the vertices we already explored, to prevent seacrh loops:
   std::unordered_set<TrajectoryConflictMap> closed_list;
   closed_list.insert(new_conflicts);
   search_queue.push(CBSTreeNode(new_conflicts, node_cost));

   size_t num_iterations = 0;
   for (; search_queue.empty() == false; search_queue.pop()) {
      const CBSTreeNode &node = search_queue.top();
      ++num_iterations;

      BOOST_LOG_SEV(lg, debug) << "*** #" << num_iterations << endl
                               << node << endl;

      // for all the conflict combinations found, enqueue them as children
      vector<TrajectoryConflictMap> conflict_combinations;
      this->generate_combinations(node.get_conflicts(), conflict_combinations);

      // generate successors
      for (const auto &cur_combination : conflict_combinations) {
         BOOST_LOG_SEV(lg, trace) << endl << "New succ: " << cur_combination;

         node_cost = this->grid_search(start_positions, goal_positions,
                                       cur_combination, paths);
         if (node_cost == 0) {
            // we're unable to find this plan, so skip this successor
            BOOST_LOG_SEV(lg, trace) << "No path" << endl;
            continue;
         }

         if (this->check_conflicts(paths, new_conflicts) == 0) {
            BOOST_LOG_SEV(lg, debug) << "No conflicts!" << endl;
            // TODO: check if there are ties, and if so, look for the one at
            // shallowest depth (minimize constraints)
            output_paths = paths; // prepare the return value
            return num_iterations;
         }
         BOOST_LOG_SEV(lg, trace) << "paths:" << endl << paths << new_conflicts; 

         // before we enqueue these conflicts, let's see if the identical ones
         // haven't already been queued up for exploration
         if (closed_list.count(new_conflicts) > 0) {
            BOOST_LOG_SEV(lg, trace) << ".. closed, skipping" << endl;
            continue; // skip this conflict set, we're already exploring them
         }
         closed_list.insert(new_conflicts);
         search_queue.push(CBSTreeNode(new_conflicts, node_cost));
         BOOST_LOG_SEV(lg, trace)
             << "queue size " << search_queue.size() << endl << endl;
      }    // for (const auto &cur_combination : conflict_combinations)
   }   // for (; search_queue.empty() == false; search_queue.pop())

   BOOST_LOG_SEV(lg, error)
       << "No solution found after " << num_iterations << " iterations" << endl;
   return num_iterations;
}

std::size_t
ConflictBasedSearch::check_conflicts(const PathSet &paths,
                                     TrajectoryConflictMap &conflicts) {
   conflicts.clear();

   if (paths.size() < 2) {
      return 0; // there can't be any conflicts
   }

   // find the longest path (TODO: actually, we need the second longest)
   size_t max_path_len = 0;
   for (const auto cur_path : paths) {
      if (max_path_len < cur_path.size()) {
         max_path_len = cur_path.size();
      }
   }

   // step through all paths and detect conflicts
   for (size_t path_step = 0; path_step < max_path_len; ++path_step) {
      for (size_t bot_idx = 0; bot_idx < paths.size(); ++bot_idx) {
         if (path_step >= paths[bot_idx].size()) {
            continue; // we're done checking this path
         }
         const Point &cur_pt = paths[bot_idx][path_step];
         for (size_t other_idx = (bot_idx + 1); other_idx < paths.size();
              ++other_idx) {
            if (path_step >= paths[other_idx].size()) {
               continue; // we're done checking this path
            }
            const Point &other_pt = paths[other_idx][path_step];
            if (cur_pt == other_pt) {
               RobotState conflict_state(
                   {size_t(cur_pt.x), size_t(cur_pt.y), path_step});
               conflict_map_append(bot_idx, conflict_state, conflicts);
               conflict_map_append(other_idx, conflict_state, conflicts);
            }
         }
      }
   }
   return conflicts.size();
}

void ConflictBasedSearch::generate_combinations(
    const TrajectoryConflictMap &conflict_map,
    std::vector<TrajectoryConflictMap> &conflict_combinations) const {
   conflict_combinations.clear();

   for (const auto &[bot_idx, all_conflicts] : conflict_map) {
      for (size_t end_idx = 0; end_idx < all_conflicts.size(); ++end_idx) {
         Trajectory cur_conflicts;
         for (size_t start_idx = 0; start_idx <= end_idx; ++start_idx) {
            cur_conflicts.push_back(all_conflicts[start_idx]);
         }
         TrajectoryConflictMap new_map;
         new_map.insert({bot_idx, cur_conflicts});
         conflict_combinations.push_back(new_map);
      }
   }

   BOOST_LOG_SEV(lg, trace) << "Generated " << conflict_combinations.size()
                            << " combinations:" << endl;
   for (const auto &cur_combination : conflict_combinations) {
      BOOST_LOG_SEV(lg, trace) << cur_combination;
   }
   BOOST_LOG_SEV(lg, trace) << endl;
}

void ConflictBasedSearch::pathset2trajset(const PathSet &path_set,
                                          TrajectorySet &traj_set) {
   traj_set.clear();
   for (const auto cur_path : path_set) {
      Trajectory cur_traj;
      for (size_t pt_idx = 0; pt_idx < cur_path.size(); ++pt_idx) {
         RobotState cur_state(POINT_TO_STATE(cur_path[pt_idx]));
         cur_state.m_time = pt_idx;
         cur_traj.push_back(cur_state);
      }
      traj_set.push_back(cur_traj);
   }
}
