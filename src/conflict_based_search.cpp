#include <stdexcept>
#include <cstdio>
#include <queue>

#include "conflict_based_search.h"

#define EXCEPTION_MSG_MAX_SIZE 1024
char exception_msg[EXCEPTION_MSG_MAX_SIZE];

#define POINT_TO_STATE(position) (RobotState({std::size_t(position.x), std::size_t(position.y), 0}))
#define STATE_TO_POINT(state)    (Point({int(state.m_x), int(state.m_y)}))

using namespace std;


// void ConflictBasedSearch::set_goals(const std::vector<RobotState> &robot_goals)
// {
//    if (robot_goals.size() != m_num_robots) {
//       snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
//                "Attempting to set a wrong number of robot goals: %zu vs %zu",
//                robot_goals.size(), m_num_robots);
//       throw std::invalid_argument(exception_msg);
//    }
//    m_robot_goals = robot_goals;
// }


/*     ----------------     TrajectoryConflict     ----------------     */

size_t hash<TrajectoryConflict>::operator()(const TrajectoryConflict &self) const {
   size_t seed = 0;
   boost::hash_combine(seed, self.first);
   for (const auto &cur_state : self.second) {
      boost::hash_combine(seed, cur_state.m_x);
      boost::hash_combine(seed, cur_state.m_y);
      boost::hash_combine(seed, cur_state.m_time);
   }   
   return seed;
}


std::ostream & operator<<(std::ostream& os, const TrajectoryConflict & conflict) {
   os << "[" << conflict.first << ", ";
   for (auto cur_state : conflict.second) {
      os << "(" << cur_state.m_x << ", " << cur_state.m_y << ", " << cur_state.m_time << ") ";
   }
   os << "]"; 
   return os;
}



std::ostream & operator<<(std::ostream& os, TrajectoryConflictSet const& conflict_set)
{
   os << "Conflict set of " << conflict_set.size() << ":" << endl;
   for (const auto & cur_conflict : conflict_set) {
      os << "  " << cur_conflict << endl;
   }
   return os;
}


std::ostream & operator<<(std::ostream& os, Path const& path) 
{
   for (const auto &cur_pt : path) {
      os << cur_pt << " ";
   }
   return os;
}


std::ostream & operator<<(std::ostream& os, PathSet const& path_set) 
{
   for (size_t path_idx = 0; path_idx < path_set.size(); ++path_idx) {
      os << path_idx << ": " << path_set[path_idx] << endl;
   }
   return os;
}


/*     ----------------     CBSTreeNode     ----------------     */
CBSTreeNode::CBSTreeNode(std::size_t num_robots) : m_total_cost(0)
{
   for (size_t bot_idx = 0; bot_idx < num_robots; ++bot_idx) {
      pair<size_t, Trajectory> cur_pair(bot_idx, Trajectory());
      m_conflicts.insert(cur_pair);
   }
}



std::ostream & operator<<(std::ostream& os, CBSTreeNode const& node) {
   os << "cost " << node.m_total_cost << ", " << node.m_conflicts;
   return os;
}




/*     ----------------     ConflictBasedSearch     ----------------     */



std::size_t ConflictBasedSearch::grid_search(const std::vector<Point> &start_positions,
                                      const std::vector<Point> &goal_positions,
                                      const TrajectoryConflictSet &conflicts,
                                      PathSet &paths)
{
   paths.clear();
   assert(start_positions.size() == m_num_robots);
   assert( goal_positions.size() == m_num_robots);
   
   paths.resize(m_num_robots);

   size_t total_cost = 0;

   for (size_t bot_idx = 0; bot_idx < m_num_robots; ++bot_idx) {
      // block off cells per conflicts for each robot
      for (const auto & cur_conflict : conflicts) {
         if (cur_conflict.first != bot_idx) {  
            continue; // skip other robots
         }
         for (const auto & cur_state : cur_conflict.second) {           
            m_grid->setObstacle(STATE_TO_POINT(cur_state));
         }

         // replan the point paths per established constraints   
         size_t cost = m_grid->get_path(start_positions[bot_idx], goal_positions[bot_idx], paths[bot_idx]);

         // undo robot conflicts
         for (const auto & cur_state : cur_conflict.second) {
            // not checking for special cells (dig or dropoff locations bc the plan 
            // should already avoid them, so they can't be a conflict             
            m_grid->unsetObstacle(STATE_TO_POINT(cur_state));
         }

         total_cost += cost;
      }   // for (const auto & cur_conflict : conflicts)
   }   // for (size_t bot_idx = 0; bot_idx < m_num_robots; ++bot_idx)

   return total_cost;
}


void ConflictBasedSearch::print_plans(const TrajectorySet &state_plans) const
{
   char cell_label[10];
   for (int x = 0; x < m_grid->getWidth(); ++x) {
      for (int y = 0; y < m_grid->getHeight(); ++y) {
         Point cur_pt{x, y};
         string cell = " . ";

         // draw plans
         for (auto cur_plan : state_plans) {
            for (auto cur_state : cur_plan) {
               //printf("trying %zu, %zu\n", cur_state.m_x, cur_state.m_y);
               if (cur_state.m_x == size_t(x) && cur_state.m_y == size_t(y)) {
                  snprintf(cell_label, 10, " %zu ", cur_state.m_time);
                  cell = string(cell_label).substr(0, 3);
                  break;
               }
            }
         }

         // draw robots
         for (size_t plan_idx = 0; plan_idx < state_plans.size(); ++plan_idx) {
            if (state_plans[plan_idx][0].m_x == size_t(x) && state_plans[plan_idx][0].m_y == size_t(y)) {
               snprintf(cell_label, 10, "R%zu ", plan_idx + 1);
               cell = string(cell_label).substr(0, 3);
            }
         }

         if (m_grid->isObstacle(cur_pt)) {
            cell = " # ";
         } else if (m_grid->isDigLocation(cur_pt)) {
            cell = " D ";
         } else if (m_grid->isDropoffLocation(cur_pt)) {
            snprintf(cell_label, 10, "|%d|", m_grid->getDropOffLocationValue(cur_pt));
            cell = string(cell_label).substr(0, 3);
         }

         std::cout << cell; // << ' ';
      }
      std::cout << std::endl;
   }
   std::cout << std::endl;
}






std::size_t ConflictBasedSearch::search(const std::vector<Point> &start_positions,
                                 const std::vector<Point> &goal_positions,
                                 PathSet &output_paths)
{
   output_paths.clear();

   assert(start_positions.size() == m_num_robots);
   assert( goal_positions.size() == m_num_robots);

   // setup the priority queue
   std::priority_queue<CBSTreeNode, vector<CBSTreeNode>, std::greater<CBSTreeNode>> search_queue;

   // root node
   CBSTreeNode root_node(m_num_robots);
   PathSet paths;
   size_t node_cost = this->grid_search(start_positions, goal_positions, root_node.get_conflicts(), paths);
   cout << "Paths:" << endl << paths << endl;

   TrajectoryConflictSet new_conflicts;
   if (this->check_conflicts(paths, new_conflicts) == 0) {
      cout << "No conflicts!" << endl;
      // TODO: check if there are ties, and if so, look for the one at shallowest depth (minimize constraints)
      output_paths = paths; // prepare the return value
      return 0;
   }

   cout << new_conflicts << endl << endl;
   search_queue.push(CBSTreeNode(new_conflicts, node_cost));

   size_t num_iterations = 0;
   for (; search_queue.empty() == false; search_queue.pop()) {
      const CBSTreeNode & node = search_queue.top();
      ++num_iterations; 

      cout << "***" << endl << "#" << num_iterations << ": " << node;

      // for all the conflict combinations found, enqueue them as children
      vector<TrajectoryConflictSet> conflict_combinations;
      this->generate_combinations(node.get_conflicts(), conflict_combinations);

      // generate successors
      for (const auto & cur_combination : conflict_combinations) {
         cout << "New succ: " << cur_combination << endl;
         //PathSet paths;

         node_cost = this->grid_search(start_positions, goal_positions, cur_combination, paths);
         cout << "cost " << node_cost << ", paths:" << endl << paths << endl;

         //TrajectoryConflictSet new_conflicts;
         if (this->check_conflicts(paths, new_conflicts) == 0) {
            cout << "No conflicts!" << endl;
            // TODO: check if there are ties, and if so, look for the one at shallowest depth (minimize constraints)
            output_paths = paths; // prepare the return value
            return num_iterations;
         }

         cout << new_conflicts << endl << endl;
         search_queue.push(CBSTreeNode(cur_combination, node_cost));
      }
    }

    printf("Warning: no solution found after %zu iterations\n", num_iterations);
    return num_iterations;
}


std::size_t ConflictBasedSearch::check_conflicts(const PathSet &paths, TrajectoryConflictSet &conflicts)
{
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
         const Point &cur_pt = paths[bot_idx][path_step];
         for (size_t other_idx = (bot_idx + 1); other_idx < paths.size(); ++other_idx) {
            const Point &other_pt = paths[other_idx][path_step];
            if (cur_pt == other_pt) {
               //TrajectoryConflict obj()
               //m_conflicts.insert(make_pair<size_t, Trajectory>(bot_idx, {RobotState({size_t(cur_pt.x), size_t(cur_pt.y), path_step})}));
               Trajectory conflict_traj({RobotState({size_t(cur_pt.x), size_t(cur_pt.y), path_step})});
               conflicts.insert(TrajectoryConflict(bot_idx,   conflict_traj));
               conflicts.insert(TrajectoryConflict(other_idx, conflict_traj));
               //m_conflicts.insert(make_pair<size_t, Trajectory>(bot_idx, {RobotState({size_t(cur_pt.x), size_t(cur_pt.y), path_step})}));
            }
         }
      } 
   }
   return conflicts.size();
}



void ConflictBasedSearch::generate_combinations(const TrajectoryConflictSet &conflicts,
                                                std::vector<TrajectoryConflictSet> &conflict_combinations) const
{
   conflict_combinations.clear();
   for (const auto & cur_conflict : conflicts) {
      TrajectoryConflictSet cur_conflict_set;
      cur_conflict_set.insert(cur_conflict);
      conflict_combinations.push_back(cur_conflict_set);
   }
}



void ConflictBasedSearch::pathset2trajset(const PathSet & path_set, TrajectorySet & traj_set)
{
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
