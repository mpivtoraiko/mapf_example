#include "grid_search.h"

Cell get(PredecessorMap const& predecessor_map, Cell cell)
{
    std::map<Cell,Cell>::const_iterator found = predecessor_map.m_map.find(cell);
    return (found != predecessor_map.m_map.end()) ? found->second : cell;
}

void put(PredecessorMap & predecessor_map, Cell key, Cell cell)
{
    predecessor_map.m_map[key] = cell;
}

int main()
{
    GridGraph g; //baseGraph;
    //boost::filtered_graph<GridGraph, orthogonal_only> g(baseGraph, orthogonal_only());
    //BOOST_CONCEPT_ASSERT((IncidenceGraphConcept< boost::filtered_graph<GridGraph, orthogonal_only> >));

    Cell start(0,0);
    Cell goal(5,7);

    std::cout << "Start vertex: " << start << std::endl;
    std::cout << "Goal vertex: " << goal << std::endl;

    PredecessorMap p;
    typedef boost::associative_property_map< DefaultMap<Cell,unsigned> > DistanceMap;
    typedef DefaultMap<Cell,unsigned> WrappedDistanceMap;
    WrappedDistanceMap wrappedMap = WrappedDistanceMap(std::numeric_limits<unsigned>::max());
    wrappedMap[start] = 0;
    DistanceMap d = DistanceMap(wrappedMap);
    auto weight_map = DefaultMap<std::pair<Cell,Cell>,unsigned>(1);
    auto vertex_index_map = std::map<Cell,unsigned>();
    auto rank_map = std::map<Cell,unsigned>();
    auto color_map = std::map<Cell,boost::default_color_type>();

    try {
        astar_search_no_init(g, 
            start,
            DistanceHeuristic<GridGraph>(goal)
            , visitor(astar_goal_visitor(goal))
            . distance_map(d)
            . predecessor_map(boost::ref(p))
            . weight_map(boost::associative_property_map< DefaultMap<std::pair<Cell,Cell>,unsigned> >(weight_map))
            . vertex_index_map(boost::associative_property_map< std::map<Cell,unsigned> >(vertex_index_map))
            . rank_map(boost::associative_property_map< std::map<Cell,unsigned> >(rank_map))
            . color_map(boost::associative_property_map< std::map<Cell,boost::default_color_type> >(color_map))
            . distance_compare(std::less<unsigned>())
            . distance_combine(std::plus<unsigned>())
            );
    } catch(FoundGoal const&) { // found     path to the goal
        std::list<Cell> shortest_path;
        for(Cell xy = goal;; xy = p[xy]) {
            shortest_path.push_front(xy);
            if(p[xy] == xy)
                break;
        }
        std::cout << "Shortest path from " << start << " to "
            << goal << ": ";
        std::list<Cell>::iterator spi = shortest_path.begin();
        std::cout << start;
        for(++spi; spi != shortest_path.end(); ++spi) 
            std::cout << " -> " << (*spi);
        std::cout << std::endl;
        return 0;
    }

    std::cout << "Didn't find a path from " << start << "to"
        << goal << "!" << std::endl;
    return 0;
}

GridGraph::GridGraph()
{}

std::pair<GridGraph::out_edge_iterator, GridGraph::out_edge_iterator> 
out_edges(GridGraph::vertex_descriptor v,
          GridGraph const& g)
{
    (void)g;
    return std::make_pair(
        GridGraph::out_edge_iterator(v, MotionStep::MIN), 
        GridGraph::out_edge_iterator(v, MotionStep::NONE) );
}

GridGraph::degree_size_type 
out_degree(GridGraph::vertex_descriptor v,
           GridGraph const& g)
{
    (void)g;
    return v.get_all_neighbors().size();
}

GridGraph::vertex_descriptor 
source(GridGraph::edge_descriptor e,
       GridGraph const& g)
{
    (void)g;
    return e.first;
}

GridGraph::vertex_descriptor target(
    GridGraph::edge_descriptor e,
    GridGraph const& g)
{
    (void)g;
    return e.second;
}

NeighborIterator::NeighborIterator() : m_cell() , m_direction() { }

NeighborIterator::NeighborIterator(Cell xy, MotionStep::direction direction)
: m_cell(xy)
, m_direction(direction)
{
}

NeighborIterator & NeighborIterator::operator=(NeighborIterator const& other)
{
    m_cell = other.m_cell;
    m_direction = other.m_direction;
    return *this;
}

std::pair<Cell,Cell> NeighborIterator::operator*() const
{
    std::pair<Cell,Cell> const retval = std::make_pair(m_cell, m_cell.get_neighbor(m_direction));
    return retval;
}

NeighborIterator& NeighborIterator::increment()
{
    m_direction = static_cast<MotionStep::direction>(int(m_direction) + 1); // TODO: modulo enum size!
    return *this;
}

bool NeighborIterator::operator==(NeighborIterator const& other) const
{
    return m_cell == other.m_cell && m_direction == other.m_direction;
}


Cell::Cell(std::size_t x, std::size_t y)
: m_x(x)
, m_y(y)
{
}


Cell::Cell(const Cell & other) : m_x(other.m_x), m_y(other.m_y) {}

// bool Cell::adjacentTo(Cell const& other) const
// {
//     return abs(m_x - other.m_x) <= 1 && abs(m_y - other.m_y) <= 1;
// }

Cell & Cell::operator=(Cell const& other)
{
    m_x = other.m_x;
    m_y = other.m_y;
    return *this;
}

Cell & Cell::operator+=(Cell const& other)
{
    m_x += other.m_x;
    m_y += other.m_y;
    return *this;
}

bool Cell::operator<(Cell const& other) const
{
    return m_x < other.m_x || (m_x == other.m_x && m_y < other.m_y);
}

std::ostream & operator<<(std::ostream & os, Cell const& cell)
{
    os << "(" << cell.m_x << ", " << cell.m_y << ", " << cell.m_time << ")";
    return os;
}

Cell Cell::get_neighbor(MotionStep::direction direction) const
{
    using namespace MotionStep;

    int dx = direction, dy = 0;

   //  switch (direction)
   //  {
   //  case LEFT:
   //      dx = -1;
   //      break;
   //  case RIGHT:
   //      dx = 1;
   //      break;
   //  case UP:
   //      dy = -1;   
   //      break;
   //  case DOWN:
   //      dy = 1;
   //      break;
   //  }
    Cell const get_neighbor(m_x + dx, m_y + dy);
    return get_neighbor;
}

std::set<Cell> Cell::get_all_neighbors() const
{
    std::set<Cell> neighbors;

    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            neighbors.insert(Cell(m_x+dx, m_y+dy));

    return neighbors;
}
