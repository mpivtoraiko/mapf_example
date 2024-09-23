#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <stdexcept>

#include "assignment.h"
#include "planner_starter_code.hpp"

#define DEBUG

#ifdef DEBUG
#include <cstdio>
#endif

#define EXCEPTION_MSG_MAX_SIZE 1024
#define MAX_AUGMENTED_ENTITIES 10
#define MAX_EDGE_COST 100000

using namespace std;

Assignment::Assignment() {
   m_src_vx = boost::add_vertex(m_graph);
   m_sink_vx = boost::add_vertex(m_graph);
}

void Assignment::reset_graph() {
   std::set<edge_t> edge_set;

   // enqueue edges emanating from the source vertex
   auto src_edges = boost::out_edges(m_src_vx, m_graph);
   for (auto cur_edge = src_edges.first; cur_edge != src_edges.second;
        ++cur_edge) {
      if (!m_graph[*cur_edge].m_is_reverse) {
         edge_set.insert(*cur_edge);
         edge_set.insert(m_graph[*cur_edge].m_reverse);
      }
   }

   // enqueue edges emanating from all the robots
   for (const auto &cur_robot : m_robots) {
      auto robot_edges = boost::out_edges(cur_robot.right, m_graph);
      for (auto cur_edge = robot_edges.first; cur_edge != robot_edges.second;
           ++cur_edge) {
         if (!m_graph[*cur_edge].m_is_reverse) {
            edge_set.insert(*cur_edge);
            edge_set.insert(m_graph[*cur_edge].m_reverse);
         }
      }
   }

   // enqueue edges coming into the sink vertex
   auto sink_edges = boost::in_edges(m_sink_vx, m_graph);
   for (auto cur_edge = sink_edges.first; cur_edge != sink_edges.second;
        ++cur_edge) {
      if (!m_graph[*cur_edge].m_is_reverse) {
         edge_set.insert(*cur_edge);
         edge_set.insert(m_graph[*cur_edge].m_reverse);
      }
   }

   for (const auto &cur_edge : edge_set) {
      boost::remove_edge(cur_edge, m_graph);
   }
}

void Assignment::set_cost(const Robot &robot, const Point &goal,
                          uint32_t cost) {
   auto robot_iter = m_robots.left.find(robot);
   vertex_t robot_vx;
   if (robot_iter == m_robots.left.end()) {
      robot_vx = boost::add_vertex(m_graph);
      upsert_edge(m_src_vx, robot_vx, 0);
      m_robots.insert(robot_map_t::value_type(robot, robot_vx));
   } else {
      robot_vx = robot_iter->second;
   }

   auto goal_iter = m_goals.left.find(goal);
   vertex_t goal_vx;
   if (goal_iter == m_goals.left.end()) {
      goal_vx = boost::add_vertex(m_graph);
      upsert_edge(goal_vx, m_sink_vx, 0);
      m_goals.insert(goal_map_t::value_type(goal, goal_vx));
   } else {
      goal_vx = goal_iter->second;
   }

   upsert_edge(robot_vx, goal_vx, cost);
}

void Assignment::upsert_edge(vertex_t src_vx, vertex_t sink_vx, int32_t cost) {
   auto edge = boost::edge(src_vx, sink_vx, m_graph);
   if (edge.second) {
      m_graph[edge.first].m_cost = cost;
      m_graph[m_graph[edge.first].m_reverse].m_cost = -cost;
   } else {
      // this pair of edges doesn't exist, so insert
      auto edge_forward = boost::add_edge(src_vx, sink_vx, m_graph);
      m_graph[edge_forward.first].m_cost = cost;
      m_graph[edge_forward.first].m_capacity = 1;

      auto edge_backward = boost::add_edge(sink_vx, src_vx, m_graph);
      m_graph[edge_backward.first].m_is_reverse = true;
      m_graph[edge_backward.first].m_cost = -cost;
      m_graph[edge_backward.first].m_capacity = 0;

      m_graph[edge_forward.first].m_reverse = edge_backward.first;
      m_graph[edge_backward.first].m_reverse = edge_forward.first;
   }
}

uint32_t Assignment::run_solver(std::map<Robot, Point> &solution,
                                std::list<Point> &unallocated_goals) {
   uint32_t cost = 0;
   solution.clear();

   // manage any problem unbalance
   if (m_goals.size() > m_robots.size()) { // more goals than robots
      size_t robots_to_add = m_goals.size() - m_robots.size();
      assert(robots_to_add > 0);
      if (robots_to_add > MAX_AUGMENTED_ENTITIES) {
         char exception_msg[EXCEPTION_MSG_MAX_SIZE];
         snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
                  "Attempting to add an excessive number of robots: %zu",
                  robots_to_add);
         throw std::invalid_argument(exception_msg);
      }
      for (auto goal_iter = m_goals.left.begin();
           goal_iter != m_goals.left.end(); ++goal_iter) {
         for (size_t idx = 0; idx < robots_to_add; ++idx)
            // insert a fake placeholder robot to balance the number of goals
            // (it has to be unique since it's going into a bimap)
            set_cost(Robot(-1 * int(idx + 1), Point({0, 0})), goal_iter->first,
                     MAX_EDGE_COST);
      }
   } else if (m_robots.size() > m_goals.size()) { // more robots than goals
      size_t goals_to_add = m_robots.size() - m_goals.size();
      assert(goals_to_add > 0);
      if (goals_to_add > MAX_AUGMENTED_ENTITIES) {
         char exception_msg[EXCEPTION_MSG_MAX_SIZE];
         snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
                  "Attempting to add an excessive number of goals: %zu",
                  goals_to_add);
         throw std::invalid_argument(exception_msg);
      }
      for (auto bot_iter = m_robots.left.begin();
           bot_iter != m_robots.left.end(); ++bot_iter) {
         for (size_t idx = 0; idx < goals_to_add; ++idx)
            // insert a fake placeholder goal to balance the number of robots
            // (it has to be unique since it's going into a bimap)
            set_cost(bot_iter->first, Point({-1 * int(idx + 1), -1}),
                     MAX_EDGE_COST);
      }
   }

#ifdef DEBUG
   printf("%zu robots, %zu goals\n", m_robots.size(), m_goals.size());
#endif
   assert(m_robots.size() ==
          m_goals.size()); // the problem should now be balanced
   boost::successive_shortest_path_nonnegative_weights(
       m_graph, m_src_vx, m_sink_vx,
       boost::capacity_map(get(&Edge::m_capacity, m_graph))
           .residual_capacity_map(get(&Edge::m_residual_capacity, m_graph))
           .weight_map(get(&Edge::m_cost, m_graph))
           .reverse_edge_map(get(&Edge::m_reverse, m_graph)));

   // convert the flow solution into the assignment solution
   auto src_edges = out_edges(m_src_vx, m_graph);
   for (auto src_edge_it = src_edges.first; src_edge_it != src_edges.second;
        ++src_edge_it) {
      vertex_t robot_vx = target(*src_edge_it, m_graph);
      auto task_edges = out_edges(robot_vx, m_graph);
      for (auto task_edge_it = task_edges.first;
           task_edge_it != task_edges.second; ++task_edge_it) {
         if (!m_graph[*task_edge_it].m_is_reverse) {
            vertex_t goal_vx = target(*task_edge_it, m_graph);
            if (m_graph[*task_edge_it].m_residual_capacity == 0) {
               int32_t edge_cost =
                   m_graph[edge(robot_vx, goal_vx, m_graph).first].m_cost;
               assert(edge_cost > 0);
               const Point &cur_pt = m_goals.right.at(goal_vx);
               if (edge_cost >= MAX_EDGE_COST) {
                  if (cur_pt.x >= 0)
                     unallocated_goals.push_back(
                         cur_pt); // record this goal as unallocated (unless
                                  // it's a placeholder goal)
                  continue;
               }
               solution[m_robots.right.at(robot_vx)] = cur_pt;
               cost += uint32_t(edge_cost);
#ifdef DEBUG
               printf("R %d [%zu] -> Pt (%d, %d) [%zu]: %d [%u]\n",
                      m_robots.right.at(robot_vx).get_id(), robot_vx, 
                      cur_pt.x, cur_pt.y, goal_vx, 
                      edge_cost, cost);
#endif
               break;
            }
         }
      }
   }

   return cost;
}
