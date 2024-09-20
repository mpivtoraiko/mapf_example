#include "planner_starter_code.hpp"
#include <cassert>

/*

    A main function using your planner functionality could look like this.
    Here we add a few obstacles, dig- & dropoff-locations. Finally, we
    assert that all the n dig locations ended up with a totoal of n items
    dropped off at the dropoff locations.

    build:
        g++ -std=c++20 planner_starter_code.cpp main.cpp -o run

    run:
        ./run
*/

int main() {
   auto grid = std::make_shared<Grid>(10, 10);

   // for initial testing, create obstacles, dig locations and drop off
   // locations:

   std::vector<Point> obstacles = {{0, 2}, {0, 6}, {1, 1}, {1, 3}, {3, 2},
                                   {6, 1}, {7, 3}, {7, 9}, {9, 1}};
   for (const auto &obstacle : obstacles) {
      grid->setObstacle(obstacle);
   }

   std::vector<Point> digLocations = {{1, 7}, {7, 4}, {8, 0}};
   for (const auto &digLoc : digLocations) {
      grid->setDigLocation(digLoc);
   }

   std::vector<Point> dropOffLocations = {{0, 9}, {5, 5}};
   for (const auto &dropLoc : dropOffLocations) {
      grid->setDropoffLocation(dropLoc);
   }

   auto robot1 = std::make_shared<Robot>(1, Point{0, 0});
   auto robot2 = std::make_shared<Robot>(2, Point{9, 9});

   Planner planner(grid);
   planner.addRobot(robot1);
   planner.addRobot(robot2);

   printState(grid, {robot1, robot2});

   // this loop simulates time advancing and after some time (80 ticks here) we
   //  submit a new job in the form of a new dig location
   int tick = 0;
   int N = 11; // 100;
   while (tick < N) {
      // at every tick the planner monitors if there is new work
      planner.monitor();
      printState(grid, {robot1, robot2});

      // insert a new dig location after some time.
      if (tick == 8) {
         Point newDigLocations = {8, 9};
         grid->setDigLocation(newDigLocations);
      }
      ++tick;
   }

   // Here we can print how the grid looks like after N ticks.
   std::cout << "Final grid state:" << std::endl;
   printState(grid, {robot1, robot2});

   // as a final check for this example we want to ensure that the combined
   // values of all the drop off location is 4
   //  (explanation: we started with 3 initial dig locations and later added 1
   //  extra)
   assert(grid->getCombinedDropOffLocationValues() == 4);

   return 0;
}
