#include "conflict_based_search.h"

void solve_cbs() {
   using namespace boost;

   Graph g(5);

   std::cout << "Vertices, outgoing edges, and adjacent vertices" << std::endl;
   vertex_iterator vi, vi_end;
   for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; vi++) {
      vertex_descriptor u = *vi;
      std::cout << "Vertex " << u << ": ";
      // Adjacenct edges
      out_edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ei++)
         std::cout << *ei << "  ";
      std::cout << " Adjacent vertices ";
      // Adjacent vertices
      // Here we want our adjacency_iterator and not boost::adjacency_iterator.
      ::adjacency_iterator ai, ai_end;
      for (boost::tie(ai, ai_end) = adjacent_vertices(u, g); ai != ai_end;
           ai++) {
         std::cout << *ai << " ";
      }
      std::cout << std::endl;
   }
   std::cout << num_vertices(g) << " vertices" << std::endl << std::endl;

   std::cout << "Edges and weights" << std::endl;
   edge_iterator ei, ei_end;
   for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
      edge_descriptor e = *ei;
      std::cout << e << " weight " << get(edge_weight, g, e) << std::endl;
   }
   std::cout << num_edges(g) << " edges" << std::endl << std::endl;

   vertex_descriptor source = 0;
   std::vector<vertex_descriptor> pred(num_vertices(g));
   std::vector<edge_weight_map_value_type> dist(num_vertices(g));
   iterator_property_map<std::vector<vertex_descriptor>::iterator,
                         property_map<Graph, vertex_index_t>::const_type>
       pred_pm(pred.begin(), get(vertex_index, g));
   iterator_property_map<std::vector<edge_weight_map_value_type>::iterator,
                         property_map<Graph, vertex_index_t>::const_type>
       dist_pm(dist.begin(), get(vertex_index, g));

   dijkstra_shortest_paths(g, source,
                           predecessor_map(pred_pm).distance_map(dist_pm));

   std::cout << "Dijkstra search from vertex " << source << std::endl;
   for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
      vertex_descriptor u = *vi;
      std::cout << "Vertex " << u << ": "
                << "parent " << pred[u] << ", "
                << "distance " << dist[u] << std::endl;
   }
}
