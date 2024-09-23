
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

   PathSet path_set;
   cbs_planner.grid_search(start_positions, goal_positions, TrajectoryConflictMap(), path_set);  
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

   PathSet path_set;
   cbs_planner.grid_search(start_positions, goal_positions, TrajectoryConflictMap(), path_set);  

   TrajectoryConflictMap new_conflicts;
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


#if 0
void test_search_medium()
{
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (int cur_x = 0; cur_x < 10; ++cur_x) {
      if (cur_x != 4 && cur_x != 5) {
         obstacles.push_back(Point({cur_x, 5}));
      }
   }
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   std::vector<Point> start_positions = {{9, 0}, {8, 0}, {8, 1}, {7, 0}, {7, 1}};
   std::vector<Point> goal_positions = {{0, 9}, {0, 8}, {1, 8}, {1, 9}, {0, 7}};
   ConflictBasedSearch cbs_planner(grid, 5);
   PathSet solution_paths;
   cbs_planner.search(start_positions, goal_positions, solution_paths);

   TrajectorySet traj_set; 
   cbs_planner.pathset2trajset(solution_paths, traj_set);
   cbs_planner.print_plans(traj_set);
}
#endif


void test_search_medium()
{
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (int cur_x = 0; cur_x < 10; ++cur_x) {
      if (cur_x != 4 && cur_x != 5) {
         obstacles.push_back(Point({cur_x, 5}));
      }
   }
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   std::vector<Point> start_positions = {{9, 0}, {8, 0}, {8, 1}, {7, 2}};
   std::vector<Point> goal_positions = {{0, 9}, {0, 8}, {1, 8}, {1, 9}};
   ConflictBasedSearch cbs_planner(grid, 4);
   PathSet solution_paths;
   cbs_planner.search(start_positions, goal_positions, solution_paths);

   TrajectorySet traj_set; 
   cbs_planner.pathset2trajset(solution_paths, traj_set);
   cbs_planner.print_plans(traj_set);
}



int main() {
   //basic_test();
   //check_conflicts();
   //test_search_simple();
   test_search_medium();

   return 0;
}