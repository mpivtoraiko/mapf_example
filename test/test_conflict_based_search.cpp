
//#include <boost/test/included/unit_test.hpp>
#include "conflict_based_search.h"

using namespace std;

int main() {
   auto grid = std::make_shared<Grid>(10, 10);
   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};

   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   ConflictBasedSearch cbs_planner(grid, 2);
   std::vector<Point> start_positions = {{0, 0}, {9, 0}};
   std::vector<Point> goal_positions = {{9, 9}, {0, 9}};

   vector<vector<RobotState>> robot_paths;
   cbs_planner.grid_search(start_positions, goal_positions, robot_paths);   
   cbs_planner.print_plans(robot_paths);

   return 0;
}

// BOOST_AUTO_TEST_CASE(BasicTest) {
//    solve_cbs();
//    BOOST_CHECK(true);
// }
