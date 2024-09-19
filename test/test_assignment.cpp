#define BOOST_TEST_MODULE AssignmentTests
#include <boost/test/included/unit_test.hpp>
#include <iostream>

#include "assignment.h"

using namespace std;

BOOST_AUTO_TEST_CASE(MyTestCase)
{
   Robot bot1(1, Point({1, 2}));
   Robot bot2(2, Point({2, 1}));
   Robot bot3(3, Point({3, 2}));

   Point goal1 = {5, 6};
   Point goal2 = {6, 5};
   Point goal3 = {7, 6};

   Assignment assignment;
   // bot1 costs
   assignment.set_cost(bot1, goal1, 10);
   assignment.set_cost(bot1, goal2, 20);
   assignment.set_cost(bot1, goal3, 30);

   // bot2 costs
   assignment.set_cost(bot2, goal1, 30);
   assignment.set_cost(bot2, goal2, 20);
   assignment.set_cost(bot2, goal3, 10);

   // bot3 costs
   assignment.set_cost(bot3, goal1, 20);
   assignment.set_cost(bot3, goal2, 10);
   assignment.set_cost(bot3, goal3, 30);

   std::map<Robot, Point> solution;
   uint32_t total_cost = assignment.run_solver(solution);
   BOOST_CHECK(total_cost == 30);
   // cout << solution[bot1] << endl;
   // cout << solution[bot2] << endl;
   // cout << solution[bot3] << endl;
   BOOST_CHECK(solution[bot1] == goal1);
   BOOST_CHECK(solution[bot2] == goal3);
   BOOST_CHECK(solution[bot3] == goal2);

}