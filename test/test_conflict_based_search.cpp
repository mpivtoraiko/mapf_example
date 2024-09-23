#define BOOST_TEST_MODULE ConflictBasedSearch

#include <boost/test/included/unit_test.hpp>

#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/console.hpp>

#include "conflict_based_search.h"

void init_logger() {
   namespace logging = boost::log;
   logging::add_console_log(std::cout, logging::keywords::format =
                                           "%Severity%\t%Message%");
   logging::core::get()->set_filter(logging::trivial::severity >=
                                    logging::trivial::info);
}

BOOST_AUTO_TEST_CASE(GridPlanningTest) {
   init_logger();
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
   cbs_planner.grid_search(start_positions, goal_positions,
                           TrajectoryConflictMap(), path_set);

   BOOST_CHECK(path_set.size() == 2);
   BOOST_CHECK(path_set[0].size() == 18);
   BOOST_CHECK(path_set[1].size() == 18);
}

BOOST_AUTO_TEST_CASE(CheckConflicts) {
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
   cbs_planner.grid_search(start_positions, goal_positions,
                           TrajectoryConflictMap(), path_set);

   TrajectoryConflictMap conflicts;
   cbs_planner.check_conflicts(path_set, conflicts);

   BOOST_CHECK(conflicts.size() == 2);
   BOOST_CHECK(conflicts[0].size() == 3);
   BOOST_CHECK(conflicts[1].size() == 3);
}

BOOST_AUTO_TEST_CASE(TestSearch) {
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
   BOOST_CHECK(solution_paths[0].size() == 13);
   BOOST_CHECK(solution_paths[1].size() == 18);
}