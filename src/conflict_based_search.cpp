#include <stdexcept>
#include <cstdio>
#include <queue>

#include "conflict_based_search.h"

#define EXCEPTION_MSG_MAX_SIZE 1024
char exception_msg[EXCEPTION_MSG_MAX_SIZE];

#define POSITION_TO_STATE(position) (RobotState({std::size_t(position.x), std::size_t(position.y), 0}))

using namespace std;


void ConflictBasedSearch::set_goals(const std::vector<RobotState> &robot_goals)
{
   if (robot_goals.size() != m_num_robots) {
      snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
               "Attempting to set a wrong number of robot goals: %zu vs %zu",
               robot_goals.size(), m_num_robots);
      throw std::invalid_argument(exception_msg);
   }
   m_robot_goals = robot_goals;
}


void ConflictBasedSearch::grid_search(const std::vector<Point> &start_positions,
                                      const std::vector<Point> &goal_positions,
                                      const std::vector<PathConflict> &conflicts,
                                      vector<vector<RobotState>> &state_plans)
{
   state_plans.clear();
   assert(start_positions.size() == m_num_robots);
   assert (goal_positions.size() == m_num_robots);

   // for (auto cur_pt : goal_positions) 
   //    m_robot_goals.push_back(POSITION_TO_STATE(cur_pt));
   
   state_plans.resize(m_num_robots);

   size_t total_cost = 0;
   for (size_t bot_idx = 0; bot_idx < m_num_robots; ++bot_idx) {
      // block off sells per conflicts for each robot
      for (auto cur_conflict : conflicts) {
         if (cur_conflict->second == bot_idx) {  // TODO: -> second is actually a vector!
            grid->setObstacle(Point({int(cur_conflict->first.m_x), int(cur_conflict->first.m_y)}));
         }
      }

      // plan the point path
      vector<Point> cur_point_path;      
      size_t cost = m_grid->get_path(start_positions[bot_idx], goal_positions[bot_idx], cur_point_path);

      // undo robot conflicts
      for (auto cur_conflict : conflicts) {
         if (cur_conflict->second == bot_idx) {
            grid->unsetObstacle(Point({int(cur_conflict->first.m_x), int(cur_conflict->first.m_y)}));
         }
      }

      // convert from point plan to state plan
      vector<RobotState> &state_plan = state_plans[bot_idx];
      for (size_t cur_step = 0; cur_step < cur_point_path.size(); ++ cur_step) {
         const Point &cur_pt = cur_point_path[cur_step];
         state_plan.push_back(RobotState({size_t(cur_pt.x), size_t(cur_pt.y), cur_step + 1}));
      }

      total_cost += cost;
   }
   // printf("total cost: %zu\n", total_cost);
   // printf("returning %zu plans:\n", state_plans.size());
   // for (auto cur_plan : state_plans)
   //    printf("  %zu steps\n", cur_plan.size());
   //printf("%zu %zu\n", state_plans[0].size(), state_plans[1].size());
}


void ConflictBasedSearch::print_plans(const vector<vector<RobotState>> &state_plans)
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


void ConflictBasedSearch::search(const std::vector<Point> &start_positions,
                                 const std::vector<Point> &goal_positions)
{
   //printf("%zu, %zu", start_positions.size(), goal_positions.size());
   assert(start_positions.size() == goal_positions.size() == m_num_robots);
   std::queue<shared_ptr<CBSTreeNode>> search_queue;
   shared_ptr<CBSTreeNode> root_node = make_shared<CBSTreeNode>();
   search_queue.push(root_node);

   while (search_queue.empty() == false) {
      shared_ptr<CBSTreeNode> node = search_queue.front();
      grid_search(start_positions, goal_positions, node->get_conflicts(), node->get_paths());
      node->check_conflicts();
      vector<vector<PathConflict>> conflict_combinations;
      node->generate_combinations(conflict_combinations);
      for (auto cur_combination : conflict_combinations) {
         search_queue.push(make_shared<CBSTreeNode>(cur_combination));
      }
      search_queue.pop();

      // for all the conflicts found, enqueue children
    }
}
