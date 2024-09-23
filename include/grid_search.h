#pragma once

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>

#include <boost/random/uniform_int.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "point.h"

#define POINT_TO_VX_DESCRIPTOR(pt)                                             \
   {                                                                           \
      { (long unsigned int)pt.x, ((long unsigned int)pt.y) }                   \
   }

/**
 * Inner graph type and its vertex types
 */
typedef boost::grid_graph<2> SearchGrid;
typedef boost::graph_traits<SearchGrid>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<SearchGrid>::vertices_size_type VertexSizeType;

struct VertexHash : std::unary_function<VertexDescriptor, std::size_t> {
   std::size_t operator()(VertexDescriptor const &in_vx) const {
      std::size_t seed = 0;
      boost::hash_combine(seed, in_vx[0]);
      boost::hash_combine(seed, in_vx[1]);
      return seed;
   }
};

typedef boost::unordered_set<VertexDescriptor, VertexHash> VertexSet;
typedef boost::vertex_subset_complement_filter<SearchGrid, VertexSet>::type
    FilteredSearchGrid;

/**
 * Outer interface class
 */
class GridSearch {
 public:
   GridSearch(std::size_t x_dim, std::size_t y_dim)
       : m_grid(boost::array<std::size_t, 2>({{x_dim, y_dim}})),
         m_obstacle_grid(
             boost::make_vertex_subset_complement_filter(m_grid, m_obstacles)) {
   }

   void set_start(const Point &start_pt) {
      m_start_vx = POINT_TO_VX_DESCRIPTOR(start_pt);
   }
   void set_goal(const Point &goal_pt) {
      m_goal_vx = POINT_TO_VX_DESCRIPTOR(goal_pt);
   }

   void set_obstacle(const Point &obstacle_pt) {
      m_obstacles.insert(POINT_TO_VX_DESCRIPTOR(obstacle_pt));
   }

   void unset_obstacle(const Point &obstacle_pt) {
      m_obstacles.erase(POINT_TO_VX_DESCRIPTOR(obstacle_pt));
   }

   std::size_t solve(std::vector<Point> &solution);

 private:
   VertexDescriptor m_start_vx;
   VertexDescriptor m_goal_vx;

   SearchGrid m_grid;
   FilteredSearchGrid m_obstacle_grid;
   VertexSet m_obstacles;
   std::size_t m_solution_length;
};

/**
 * A-star heuristic on a grid: L1-distance
 */
class GridHeuristic
    : public boost::astar_heuristic<FilteredSearchGrid, std::size_t> {
 public:
   GridHeuristic(VertexDescriptor goal_vx) : m_goal(goal_vx) {};

   std::size_t operator()(VertexDescriptor query_vx) {
      return std::abs(int(m_goal[0] - query_vx[0])) +
             std::abs(int(m_goal[1] - query_vx[1]));
   }

 private:
   VertexDescriptor m_goal;
};

/**
 * Termination condition: goal achievement and the type to
 * throw as exception to interrupt the search
 */
struct FoundGoal {};

struct GoalVisitor : public boost::default_astar_visitor {
   GoalVisitor(VertexDescriptor goal) : m_goal_vx(goal) {};

   void examine_vertex(VertexDescriptor query_vx, const FilteredSearchGrid &) {
      if (query_vx == m_goal_vx)
         throw FoundGoal();
   }

 private:
   VertexDescriptor m_goal_vx;
};
