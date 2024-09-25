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


namespace MotionStep
{
enum direction
{
    MIN = 0,
    UP = MIN, DOWN, LEFT, RIGHT, PAUSE, NONE
};
}


// TODO: consider merging with RobotState (fconflict_based_search.h) via multiple inheritance
struct Cell : public boost::additive<Cell, boost::totally_ordered<Cell, boost::equivalent<Cell>>>
{
   Cell(std::size_t x = 0, std::size_t y = 0);
   Cell(const Cell & other);

   Cell & operator=(Cell const& other);
   Cell & operator+=(Cell const& other);

   bool operator<(Cell const& other) const;

   std::size_t m_x;
   std::size_t m_y;
   std::size_t m_time;

   Cell get_neighbor(MotionStep::direction direction) const;
   std::set<Cell> get_all_neighbors() const;
};


std::ostream & operator<<(std::ostream & os, Cell const& cell);

struct NeighborIterator;

struct GridGraph
{
   GridGraph();

   typedef Cell                              vertex_descriptor;
   typedef std::pair<Cell, Cell>             edge_descriptor;
   typedef boost::undirected_tag             directed_category;
   typedef boost::disallow_parallel_edge_tag edge_parallel_category;
   typedef boost::incidence_graph_tag        traversal_category;

   typedef NeighborIterator                 out_edge_iterator;
   typedef int                               degree_size_type;
};

namespace boost
{
   template <> struct graph_traits<GridGraph>
   {
      typedef GridGraph::vertex_descriptor      vertex_descriptor;
      typedef GridGraph::edge_descriptor        edge_descriptor;
      typedef GridGraph::out_edge_iterator      out_edge_iterator;

      typedef GridGraph::directed_category      directed_category;
      typedef GridGraph::edge_parallel_category edge_parallel_category;
      typedef GridGraph::traversal_category     traversal_category;

      typedef GridGraph::degree_size_type       degree_size_type;

      typedef void in_edge_iterator;
      typedef void vertex_iterator;
      typedef void vertices_size_type;
      typedef void edge_iterator;
      typedef void edges_size_type;
   };
}

std::pair<GridGraph::out_edge_iterator, 
          GridGraph::out_edge_iterator> out_edges(GridGraph::vertex_descriptor, GridGraph const&);
GridGraph::degree_size_type out_degree(GridGraph::vertex_descriptor, GridGraph const&);
GridGraph::vertex_descriptor source(GridGraph::edge_descriptor, GridGraph const&);
GridGraph::vertex_descriptor target(GridGraph::edge_descriptor, GridGraph const&);


struct NeighborIterator : public boost::iterator_facade<NeighborIterator,
                                                        std::pair<Cell, Cell>,
                                                        boost::forward_traversal_tag,
                                                        std::pair<Cell, Cell> >
{
 public:
   NeighborIterator();
   NeighborIterator(Cell cur_cell, MotionStep::direction direction);

   NeighborIterator & operator=(NeighborIterator const& other);
   std::pair<Cell, Cell> operator*() const;
   //NeighborIterator& operator++();
   NeighborIterator& increment(); // {return operator++();}  // needed by iterator_facade.hpp

   bool operator==(NeighborIterator const& other) const;
   bool equal(NeighborIterator const &other) const { return operator==(other); }  // needed by iterator_facade.hpp

 private:
   Cell m_cell;
   MotionStep::direction m_direction;
};


/**
 * A traversal filter
 */
// struct orthogonal_only
// {
//     typedef std::pair<Cell, Cell> Edge;
//     bool operator()(Edge const& edge) const
//     {
//         return edge.first.m_x == edge.second.m_x || edge.first.m_y == edge.second.m_y;
//     }
// };

template <typename Graph> class DistanceHeuristic;

struct FoundGoal {}; // exception for termination


class astar_goal_visitor : public boost::default_astar_visitor
{
 public:
   astar_goal_visitor(const Cell & goal) : m_goal(goal) {}

   void examine_vertex(const Cell & cell, GridGraph const& g) {
      (void) g;   // squash unused-parameter warning
      std::cout << "Exploring " << cell << "..." << std::endl;
      if(cell == m_goal)
         throw FoundGoal();
   }

 private:
   Cell m_goal;
};



template <typename Key, typename Value> class DefaultMap
{
public:
   typedef Key key_type;
   typedef Value data_type;
   typedef std::pair<Key, Value> value_type;

   DefaultMap(Value const& default_value) : m_map(), m_default_value(default_value) {}

   Value & operator[](Key const& key) {
      if (m_map.find(key) == m_map.end()) {
         m_map[key] = m_default_value;
      }
      return m_map[key];
   }

private:
    std::map<Key, Value> m_map;
    Value const m_default_value;
};



struct PredecessorMap
{
   PredecessorMap() : m_map() {}
   PredecessorMap(PredecessorMap const& other) : m_map(other.m_map) {}

   typedef Cell key_type;
   typedef Cell value_type;
   typedef Cell & reference_type;
   typedef boost::read_write_property_map_tag category;

   Cell & operator[](const Cell & cell) { return m_map[cell]; }

   std::map<Cell, Cell> m_map;
};


template <typename Graph> class DistanceHeuristic : public boost::astar_heuristic<Graph, std::size_t>
{
public:
   DistanceHeuristic(Cell goal) : m_goal(goal) {}
   unsigned operator()(Cell cell)
   {
      int dx = std::abs(int(m_goal.m_x) - int(cell.m_x));
      int dy = std::abs(int(m_goal.m_y) - int(cell.m_y));
      return static_cast<std::size_t>(dx + dy);
   }
private:
    Cell m_goal;
};

