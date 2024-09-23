
//#include <boost/test/included/unit_test.hpp>
#include "conflict_based_search.h"

using namespace std;

void basic_test()
{
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   ConflictBasedSearch cbs_planner(grid, 2);
   std::vector<Point> start_positions = {{0, 0}, {9, 0}};
   std::vector<Point> goal_positions = {{9, 9}, {0, 9}};

   CBSTreeNode node(2);
   PathSet path_set;
   cbs_planner.grid_search(start_positions, goal_positions, node.get_conflicts(), path_set);  
   TrajectorySet traj_set; 
   cbs_planner.pathset2trajset(path_set, traj_set);
   cbs_planner.print_plans(traj_set);
}

// BOOST_AUTO_TEST_CASE(BasicTest) {
//    solve_cbs();
//    BOOST_CHECK(true);
// }


void check_conflicts()
{
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   ConflictBasedSearch cbs_planner(grid, 2);
   std::vector<Point> start_positions = {{5, 0}, {9, 0}};
   std::vector<Point> goal_positions = {{9, 9}, {0, 9}};

   CBSTreeNode node(2);
   PathSet path_set;
   cbs_planner.grid_search(start_positions, goal_positions, node.get_conflicts(), path_set);  

   TrajectoryConflictSet new_conflicts;
   cbs_planner.check_conflicts(path_set, new_conflicts);
   cout << new_conflicts;

   TrajectorySet traj_set; 
   cbs_planner.pathset2trajset(path_set, traj_set);
   cbs_planner.print_plans(traj_set);
}


void test_search_simple()
{
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   std::vector<Point> start_positions = {{5, 0}, {9, 0}};
   std::vector<Point> goal_positions = {{9, 9}, {0, 9}};
   ConflictBasedSearch cbs_planner(grid, 2);
   PathSet solution_paths;
   cbs_planner.search(start_positions, goal_positions, solution_paths);

   TrajectorySet traj_set; 
   cbs_planner.pathset2trajset(solution_paths, traj_set);
   cbs_planner.print_plans(traj_set);
}

int main() {
   //check_conflicts();
   test_search_simple();

   return 0;
}