#include <ostream>

#include "grid_search.h"



std::size_t GridSearch::solve(std::vector<Point> & solution) {
   solution.clear();

   boost::static_property_map<std::size_t> edge_cost(1);
   typedef boost::unordered_map<VertexDescriptor, VertexDescriptor, VertexHash>
       PredecessorMap;
   PredecessorMap pred_map;
   boost::associative_property_map<PredecessorMap> pred_property_map(pred_map);

   typedef boost::unordered_map<VertexDescriptor, std::size_t, VertexHash>
       CostMap;
   CostMap cost_map;
   boost::associative_property_map<CostMap> cost_property_map(cost_map);

   GridHeuristic heuristic(m_goal_vx);
   GoalVisitor goal_visitor(m_goal_vx);

  try {
      astar_search(m_barrier_grid, m_start_vx, heuristic,
                   boost::weight_map(edge_cost)
                       .predecessor_map(pred_property_map)
                       .distance_map(cost_property_map)
                       .visitor(goal_visitor));
   } catch (FoundGoal found_goal_exception) {
      for (VertexDescriptor cur_vx = m_goal_vx; cur_vx != m_start_vx;
           cur_vx = pred_map[cur_vx]) {
         solution.push_back(Point({int(cur_vx[0]), int(cur_vx[1])}));
      }
      std::reverse(solution.begin(), solution.end());
      return cost_map[m_goal_vx];
   }

   return 0;
}

