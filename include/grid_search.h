#pragma once

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <utility>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/operators.hpp>
#include <boost/ref.hpp>

#include "robot_state.h"

namespace MotionStep {
enum direction { START = 0, UP = START, DOWN, LEFT, RIGHT, PAUSE, END };
}

/*     ----------------     Cell     ----------------     */
struct Cell
    : RobotState,
      boost::additive<Cell,
                      boost::totally_ordered<Cell, boost::equivalent<Cell>>> {
   Cell(std::size_t x = 0, std::size_t y = 0) : RobotState(x, y) {}
   Cell(const Cell &other) : RobotState(other) {}

   Cell &operator=(Cell const &other) {
      m_x = other.m_x;
      m_y = other.m_y;
      return *this;
   }

   bool operator<(Cell const &other) const 
   {
      return m_x < other.m_x || (m_x == other.m_x && m_y < other.m_y);
   }

   Cell get_neighbor(MotionStep::direction direction) const;
};

std::ostream &operator<<(std::ostream &os, Cell const &cell)
{
   os << "(" << cell.m_x << ", " << cell.m_y << ", " << cell.m_time << ")";
   return os;
}

/*     ----------------     NeighborIterator     ----------------     */
struct NeighborIterator
    : public boost::iterator_facade<NeighborIterator, std::pair<Cell, Cell>,
                                    boost::forward_traversal_tag,
                                    std::pair<Cell, Cell>> {
 public:
   NeighborIterator() : m_cell(), m_direction() {}
   NeighborIterator(Cell cell, MotionStep::direction direction)
       : m_cell(cell), m_direction(direction) {}

   NeighborIterator &operator=(NeighborIterator const &other) {
      m_cell = other.m_cell;
      m_direction = other.m_direction;
      return *this;
   }

   std::pair<Cell, Cell> operator*() const {
      std::pair<Cell, Cell> const retval = std::make_pair(m_cell, m_cell.get_neighbor(m_direction));
      return retval;
   }

   NeighborIterator &increment() { // needed by iterator_facade.hpp
      m_direction = static_cast<MotionStep::direction>(int(m_direction + 1));
      return *this;
   }

   bool operator==(NeighborIterator const &other) const {
      return m_cell == other.m_cell && m_direction == other.m_direction;      
   }

   bool equal(NeighborIterator const &other) const {
      return operator==(other);
   } // needed by iterator_facade.hpp

 private:
   Cell m_cell;
   MotionStep::direction m_direction;
};

/*     ----------------     GridGraph     ----------------     */
struct GridGraph {
   GridGraph() {}

   typedef Cell vertex_descriptor;
   typedef std::pair<Cell, Cell> edge_descriptor;
   typedef boost::undirected_tag directed_category;
   typedef boost::disallow_parallel_edge_tag edge_parallel_category;
   typedef boost::incidence_graph_tag traversal_category;

   typedef NeighborIterator out_edge_iterator;
   typedef int degree_size_type;
};

namespace boost {
template <> struct graph_traits<GridGraph> {
   typedef GridGraph::vertex_descriptor vertex_descriptor;
   typedef GridGraph::edge_descriptor edge_descriptor;
   typedef GridGraph::out_edge_iterator out_edge_iterator;

   typedef GridGraph::directed_category directed_category;
   typedef GridGraph::edge_parallel_category edge_parallel_category;
   typedef GridGraph::traversal_category traversal_category;

   typedef GridGraph::degree_size_type degree_size_type;

   typedef void in_edge_iterator;
   typedef void vertex_iterator;
   typedef void vertices_size_type;
   typedef void edge_iterator;
   typedef void edges_size_type;
};
} // namespace boost

std::pair<GridGraph::out_edge_iterator, GridGraph::out_edge_iterator>
out_edges(GridGraph::vertex_descriptor vertex, GridGraph const & graph) {
   (void)graph;
   return std::make_pair(GridGraph::out_edge_iterator(vertex, MotionStep::START),
                         GridGraph::out_edge_iterator(vertex, MotionStep::END));
}

GridGraph::degree_size_type out_degree(GridGraph::vertex_descriptor vertex,
                                       GridGraph const & graph) {
   (void)graph;
   (void)vertex;
   return MotionStep::END;
}                                          


GridGraph::vertex_descriptor source(GridGraph::edge_descriptor edge,
                                    GridGraph const &graph) {
   (void)graph;
   return edge.first;
}

GridGraph::vertex_descriptor target(GridGraph::edge_descriptor edge,
                                    GridGraph const &graph) {
   (void)graph;
   return edge.second;
}

/*     ----------------     AstarGoalVisitor     ----------------     */
struct FoundGoal {}; // exception for termination

class AstarGoalVisitor : public boost::default_astar_visitor {
 public:
   AstarGoalVisitor(const Cell &goal) : m_goal(goal) {}

   void examine_vertex(const Cell &cell, GridGraph const &g) {
      (void)g; // squash unused-parameter warning
      std::cout << "Exploring " << cell << "..." << std::endl;
      if (cell == m_goal)
         throw FoundGoal();
   }

 private:
   Cell m_goal;
};

/*     ----------------     DefaultMap     ----------------     */
template <typename Key, typename Value> class DefaultMap {
 public:
   typedef Key key_type;
   typedef Value data_type;
   typedef std::pair<Key, Value> value_type;

   DefaultMap(Value const &default_value)
       : m_map(), m_default_value(default_value) {}

   Value &operator[](Key const &key) {
      if (m_map.find(key) == m_map.end()) {
         m_map[key] = m_default_value;
      }
      return m_map[key];
   }

 private:
   std::map<Key, Value> m_map;
   Value const m_default_value;
};

/*     ----------------     PredecessorMap     ----------------     */
struct PredecessorMap {
   PredecessorMap() : m_map() {}
   PredecessorMap(PredecessorMap const &other) : m_map(other.m_map) {}

   typedef Cell key_type;
   typedef Cell value_type;
   typedef Cell &reference_type;
   typedef boost::read_write_property_map_tag category;

   Cell &operator[](const Cell &cell) { return m_map[cell]; }

   std::map<Cell, Cell> m_map;
};

/*     ----------------     DistanceHeuristic     ----------------     */
template <typename Graph>
class DistanceHeuristic : public boost::astar_heuristic<Graph, std::size_t> {
 public:
   DistanceHeuristic(Cell goal) : m_goal(goal) {}
   unsigned operator()(Cell cell) {
      int dx = std::abs(int(m_goal.m_x) - int(cell.m_x));
      int dy = std::abs(int(m_goal.m_y) - int(cell.m_y));
      return static_cast<std::size_t>(dx + dy);
   }

 private:
   Cell m_goal;
};

/**
 * Outer interface class
 */
#if 0
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
#endif