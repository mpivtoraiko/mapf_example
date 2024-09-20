#define BOOST_TEST_MODULE GridSearchTests

#include <boost/test/included/unit_test.hpp>
#include "grid_search.h"

BOOST_AUTO_TEST_CASE(NoObstacles) {
   GridSearch grid_search(10, 10);
   grid_search.set_start(Point({0, 0}));
   grid_search.set_goal(Point({9, 9}));
   std::vector<Point> solution;
   std::size_t cost = grid_search.solve(solution);
   BOOST_CHECK(cost == 18);
}


BOOST_AUTO_TEST_CASE(WithObstacles) {
   GridSearch grid_search(10, 10);
   grid_search.set_start(Point({0, 0}));
   grid_search.set_goal(Point({9, 9}));
   for (int cur_x = 0; cur_x < 9; ++cur_x) 
      grid_search.set_obstacle(Point({cur_x, 5}));
   std::vector<Point> solution;
   std::size_t cost = grid_search.solve(solution);
   BOOST_CHECK(cost == 18);
   BOOST_CHECK(count(solution.begin(), solution.end(), Point({9, 4})) == 1);
   for (int cur_x = 0; cur_x < 9; ++cur_x) 
      BOOST_CHECK(count(solution.begin(), solution.end(), Point({cur_x, 5})) == 0);
}
   
