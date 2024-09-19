#pragma once

// #ifndef ASSIGNMENT_H
// #define ASSIGNMENT_H

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <cstdint>

#include "planner_starter_code.hpp"

class Assignment {
 public:
   Assignment();

   void set_cost(const Robot &robot, const Point &goal, uint32_t cost);
   uint32_t run_solver(std::map<Robot, Point> &solution,
                       std::list<Point> &unallocated_robots);

 protected:
   typedef boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                        boost::bidirectionalS>
       graphTraits_t;
   typedef graphTraits_t::vertex_descriptor vertex_t;
   typedef graphTraits_t::edge_descriptor edge_t;

   struct Vertex {};

   struct Edge {
      Edge()
          : m_cost(0), m_capacity(0), m_residual_capacity(0), m_reverse(),
            m_is_reverse(false) {}

      int32_t m_cost;
      int32_t m_capacity;
      int32_t m_residual_capacity;
      edge_t m_reverse;
      bool m_is_reverse;
   };

   typedef boost::adjacency_list<boost::vecS, boost::vecS,
                                 boost::bidirectionalS, Vertex, Edge>
       Graph;

   void reset_graph();
   void upsert_edge(vertex_t src_vx, vertex_t sink_vx, int32_t cost);

 private:
   typedef boost::bimap<
       boost::bimaps::unordered_set_of<Robot, Robot::Hash, Robot::Equality>,
       vertex_t>
       robot_map_t;
   typedef boost::bimap<
       boost::bimaps::unordered_set_of<Point, Point::Hash, Point::Equality>,
       vertex_t>
       goal_map_t;

   robot_map_t m_robots;
   goal_map_t m_goals;

   boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                         Vertex, Edge>
       m_graph;
   vertex_t m_src_vx;
   vertex_t m_sink_vx;
};

// #endif // ASSIGNMENT_H