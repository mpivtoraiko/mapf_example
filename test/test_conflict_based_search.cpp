#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/properties.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <utility>


class Graph;
class IncidentEdgeIterator;
class AdjacencyIterator;
class EdgeIterator;
struct EdgeWeightMap;


template <> struct boost::property_map<Graph, boost::edge_weight_t> {
   typedef EdgeWeightMap type;
   typedef EdgeWeightMap const_type;
};

template <> struct boost::property_map<const Graph, boost::edge_weight_t> {
   typedef EdgeWeightMap type;
   typedef EdgeWeightMap const_type;
};

struct TraversalCatetory : virtual public boost::bidirectional_graph_tag,
                           virtual public boost::adjacency_graph_tag,
                           virtual public boost::vertex_list_graph_tag,
                           virtual public boost::edge_list_graph_tag {};


class Graph {
 public:
   typedef std::size_t vertex_descriptor;
   typedef boost::undirected_tag directed_category;
   typedef boost::disallow_parallel_edge_tag edge_parallel_category;
   typedef TraversalCatetory traversal_category;


   typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;
   typedef IncidentEdgeIterator out_edge_iterator;
   typedef IncidentEdgeIterator in_edge_iterator;
   typedef std::size_t degree_size_type;

   typedef AdjacencyIterator adjacency_iterator;

   typedef boost::counting_iterator<vertex_descriptor> vertex_iterator;
   typedef std::size_t vertices_size_type;

   typedef EdgeIterator edge_iterator;
   typedef std::size_t edges_size_type;

   typedef vertex_descriptor vertex_property_type;

   Graph(std::size_t n) : m_n(n) {};
   std::size_t n() const { return m_n; }

 private:

   std::size_t m_n;
};


typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<Graph>::in_edge_iterator in_edge_iterator;
typedef boost::graph_traits<Graph>::adjacency_iterator adjacency_iterator;
typedef boost::graph_traits<Graph>::degree_size_type degree_size_type;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type;
typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
typedef boost::graph_traits<Graph>::edges_size_type edges_size_type;

struct iterator_position {};
struct iterator_start : virtual public iterator_position {};
struct iterator_end : virtual public iterator_position {};


class IncidentEdgeIterator
    : public boost::iterator_adaptor<
          IncidentEdgeIterator, boost::counting_iterator<std::size_t>,
          edge_descriptor, boost::use_default, edge_descriptor> {
 public:
   IncidentEdgeIterator()
       : IncidentEdgeIterator::iterator_adaptor_(0), m_n(0), m_u(0) {};
   explicit IncidentEdgeIterator(const Graph &g, vertex_descriptor u,
                                 iterator_start)
       : IncidentEdgeIterator::iterator_adaptor_(0), m_n(g.n()), m_u(u) {};
   explicit IncidentEdgeIterator(const Graph &g, vertex_descriptor u,
                                 iterator_end)
       : IncidentEdgeIterator::iterator_adaptor_(g.n() > 2 ? 2 : 1), m_n(g.n()),
         m_u(u) {};

 private:
   friend class boost::iterator_core_access;

   edge_descriptor dereference() const {
      static const int ring_offset[] = {1, -1};
      vertex_descriptor v;

      std::size_t p = *this->base_reference();
      if (m_u == 0 && p == 1)
         v = m_n - 1; 
      else
         v = (m_u + ring_offset[p]) % m_n;
      return edge_descriptor(m_u, v);
   }

   std::size_t m_n;       
   vertex_descriptor m_u; 
};


vertex_descriptor source(edge_descriptor e, const Graph &) {
   return e.first;
}

vertex_descriptor target(edge_descriptor e, const Graph &) {
   return e.second;
}

std::pair<out_edge_iterator, out_edge_iterator> out_edges(vertex_descriptor u,
                                                          const Graph &g) {
   return std::pair<out_edge_iterator, out_edge_iterator>(
       out_edge_iterator(g, u, iterator_start()),
       out_edge_iterator(g, u, iterator_end()));
}

degree_size_type out_degree(vertex_descriptor, const Graph &) {
   // All vertices in a ring graph have two neighbors.
   return 2;
}

// BidirectionalGraph valid expressions
std::pair<in_edge_iterator, in_edge_iterator> in_edges(vertex_descriptor u,
                                                       const Graph &g) {
   // The in-edges and out-edges are the same in an undirected graph.
   return out_edges(u, g);
}

degree_size_type in_degree(vertex_descriptor u, const Graph &g) {
   // The in-degree and out-degree are both equal to the number of incident
   // edges in an undirected graph.
   return out_degree(u, g);
}

degree_size_type degree(vertex_descriptor u, const Graph &g) {
   // The in-degree and out-degree are both equal to the number of incident
   // edges in an undirected graph.
   return out_degree(u, g);
}

/*
Iterator over vertices adjacent to a given vertex.

This iterates over the target vertices of all the incident edges.
*/
class AdjacencyIterator
    : public boost::adjacency_iterator_generator<Graph, vertex_descriptor,
                                                 out_edge_iterator>::type {
   // The parent class is an iterator_adpator that turns an iterator over
   // out edges into an iterator over adjacent vertices.
   typedef boost::adjacency_iterator_generator<
       Graph, vertex_descriptor, out_edge_iterator>::type parent_class;

 public:
   AdjacencyIterator() {};
   AdjacencyIterator(vertex_descriptor u, const Graph &g, iterator_start)
       : parent_class(out_edge_iterator(g, u, iterator_start()), &g) {};
   AdjacencyIterator(vertex_descriptor u, const Graph &g, iterator_end)
       : parent_class(out_edge_iterator(g, u, iterator_end()), &g) {};
};

// AdjacencyGraph valid expressions
std::pair<adjacency_iterator, adjacency_iterator>
adjacent_vertices(vertex_descriptor u, const Graph &g) {
   return std::pair<adjacency_iterator, adjacency_iterator>(
       adjacency_iterator(u, g, iterator_start()),
       adjacency_iterator(u, g, iterator_end()));
}

// VertexListGraph valid expressions
vertices_size_type num_vertices(const Graph &g) { return g.n(); };

std::pair<vertex_iterator, vertex_iterator> vertices(const Graph &g) {
   return std::pair<vertex_iterator, vertex_iterator>(
       vertex_iterator(0),                // The first iterator position
       vertex_iterator(num_vertices(g))); // The last iterator position
}

/*
Iterator over edges in a ring graph.

This object iterates over all the vertices in the graph, then for each
vertex returns its first outgoing edge.

It is implemented with the boost::iterator_adaptor class, because it is
essentially a vertex_iterator with a customized deference operation.
*/
class EdgeIterator
    : public boost::iterator_adaptor<EdgeIterator, vertex_iterator,
                                     edge_descriptor, boost::use_default,
                                     edge_descriptor> {
 public:
   EdgeIterator() : EdgeIterator::iterator_adaptor_(0), m_g(NULL) {};
   explicit EdgeIterator(const Graph &g, iterator_start)
       : EdgeIterator::iterator_adaptor_(vertices(g).first), m_g(&g) {};
   explicit EdgeIterator(const Graph &g, iterator_end)
       : EdgeIterator::iterator_adaptor_(
             // Size 2 graphs have a single edge connecting the two vertices.
             g.n() == 2 ? ++(vertices(g).first) : vertices(g).second),
         m_g(&g) {};

 private:
   friend class boost::iterator_core_access;

   edge_descriptor dereference() const {
      // The first element in the incident edge list of the current vertex.
      return *(out_edges(*this->base_reference(), *m_g).first);
   }

   // The graph being iterated over
   const Graph *m_g;
};

// EdgeListGraph valid expressions
std::pair<edge_iterator, edge_iterator> edges(const Graph &g) {
   return std::pair<edge_iterator, edge_iterator>(
       EdgeIterator(g, iterator_start()), EdgeIterator(g, iterator_end()));
}

edges_size_type num_edges(const Graph &g) {
   // There are as many edges as there are vertices, except for size 2
   // graphs, which have a single edge connecting the two vertices.
   return g.n() == 2 ? 1 : g.n();
}

// AdjacencyMatrix valid expressions
std::pair<edge_descriptor, bool> edge(vertex_descriptor u, vertex_descriptor v,
                                      const Graph &g) {
   if ((u == v + 1 || v == u + 1) && u > 0 && u < num_vertices(g) && v > 0 &&
       v < num_vertices(g))
      return std::pair<edge_descriptor, bool>(edge_descriptor(u, v), true);
   else
      return std::pair<edge_descriptor, bool>(edge_descriptor(), false);
}

/*
Map from edges to weight values
*/
struct EdgeWeightMap {
   typedef double value_type;
   typedef value_type reference;
   typedef edge_descriptor key_type;
   typedef boost::readable_property_map_tag category;

   // Edges have a weight equal to the average of their endpoint indexes.
   reference operator[](key_type e) const { return (e.first + e.second) / 2.0; }
};

// Use these propety_map and property_traits parameterizations to refer to
// the associated property map types.
typedef boost::property_map<Graph, boost::edge_weight_t>::const_type
    const_edge_weight_map;
typedef boost::property_traits<const_edge_weight_map>::reference
    edge_weight_map_value_type;
typedef boost::property_traits<const_edge_weight_map>::key_type
    edge_weight_map_key;

// PropertyMap valid expressions
edge_weight_map_value_type get(const_edge_weight_map pmap,
                               edge_weight_map_key e) {
   return pmap[e];
}

// ReadablePropertyGraph valid expressions
const_edge_weight_map get(boost::edge_weight_t, const Graph &) {
   return const_edge_weight_map();
}

edge_weight_map_value_type get(boost::edge_weight_t tag, const Graph &g,
                               edge_weight_map_key e) {
   return get(tag, g)[e];
}

// This expression is not part of a graph concept, but is used to return the
// default vertex index map used by the Dijkstra search algorithm.
boost::identity_property_map get(boost::vertex_index_t, const Graph &) {
   // The vertex descriptors are already unsigned integer indices, so just
   // return an identity map.
   return boost::identity_property_map();
}

// Print edges as (x, y)
std::ostream &operator<<(std::ostream &output, const edge_descriptor &e) {
   return output << "(" << e.first << ", " << e.second << ")";
}

int main(int argc, char const *argv[]) {
   using namespace boost;
   // Check the concepts that graph models.  This is included to demonstrate
   // how concept checking works, but is not required for a working program
   // since Boost algorithms do their own concept checking.
   BOOST_CONCEPT_ASSERT((BidirectionalGraphConcept<Graph>));
   BOOST_CONCEPT_ASSERT((AdjacencyGraphConcept<Graph>));
   BOOST_CONCEPT_ASSERT((VertexListGraphConcept<Graph>));
   BOOST_CONCEPT_ASSERT((EdgeListGraphConcept<Graph>));
   BOOST_CONCEPT_ASSERT((AdjacencyMatrixConcept<Graph>));
   BOOST_CONCEPT_ASSERT(
       (ReadablePropertyMapConcept<const_edge_weight_map, edge_descriptor>));
   BOOST_CONCEPT_ASSERT(
       (ReadablePropertyGraphConcept<Graph, edge_descriptor, edge_weight_t>));

   // Specify the size of the graph on the command line, or use a default size
   // of 5.
   std::size_t n = argc == 2 ? boost::lexical_cast<std::size_t>(argv[1]) : 5;

   // Create a small ring graph.
   Graph g(n);

   // Print the outgoing edges of all the vertices.  For n=5 this will print:
   //
   // Vertices, outgoing edges, and adjacent vertices
   // Vertex 0: (0, 1)  (0, 4)   Adjacent vertices 1 4
   // Vertex 1: (1, 2)  (1, 0)   Adjacent vertices 2 0
   // Vertex 2: (2, 3)  (2, 1)   Adjacent vertices 3 1
   // Vertex 3: (3, 4)  (3, 2)   Adjacent vertices 4 2
   // Vertex 4: (4, 0)  (4, 3)   Adjacent vertices 0 3
   // 5 vertices
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

   // Print all the edges in the graph along with their weights.  For n=5 this
   // will print:
   //
   // Edges and weights
   // (0, 1) weight 0.5
   // (1, 2) weight 1.5
   // (2, 3) weight 2.5
   // (3, 4) weight 3.5
   // (4, 0) weight 2
   // 5 edges
   std::cout << "Edges and weights" << std::endl;
   edge_iterator ei, ei_end;
   for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
      edge_descriptor e = *ei;
      std::cout << e << " weight " << get(edge_weight, g, e) << std::endl;
   }
   std::cout << num_edges(g) << " edges" << std::endl;

   if (n > 0) {
      std::cout << std::endl;
      // Do a Dijkstra search from vertex 0.  For n=5 this will print:
      //
      // Dijkstra search from vertex 0
      // Vertex 0: parent 0, distance 0
      // Vertex 1: parent 0, distance 0.5
      // Vertex 2: parent 1, distance 2
      // Vertex 3: parent 2, distance 4.5
      // Vertex 4: parent 0, distance 2
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
                   << "parent " << pred[*vi] << ", "
                   << "distance " << dist[u] << std::endl;
      }
   }

   return 0;
}