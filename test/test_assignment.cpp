#define BOOST_TEST_MODULE AssignmentTests
#include <boost/test/included/unit_test.hpp>
#include <iostream>

#include "assignment.h"

using namespace std;

BOOST_AUTO_TEST_CASE(BalancedAssignment) {
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
   std::list<Point> unallocated_goals;
   uint32_t total_cost = assignment.run_solver(solution, unallocated_goals);
   BOOST_CHECK(total_cost == 30);
   BOOST_CHECK(solution[bot1] == goal1);
   BOOST_CHECK(solution[bot2] == goal3);
   BOOST_CHECK(solution[bot3] == goal2);
   BOOST_CHECK(unallocated_goals.size() == 0);
}

BOOST_AUTO_TEST_CASE(UnbalancedAssignmentMoreGoals) {
   Robot bot1(1, Point({1, 2}));
   Robot bot2(2, Point({2, 1}));
   Robot bot3(3, Point({3, 2}));

   Point goal1 = {5, 6};
   Point goal2 = {6, 5};
   Point goal3 = {6, 7};
   Point goal4 = {7, 6};
   Point goal5 = {7, 8};

   Assignment assignment;
   // bot1 costs
   assignment.set_cost(bot1, goal1, 10);
   assignment.set_cost(bot1, goal2, 20);
   assignment.set_cost(bot1, goal3, 30);
   assignment.set_cost(bot1, goal4, 20);
   assignment.set_cost(bot1, goal5, 30);

   // bot2 costs
   assignment.set_cost(bot2, goal1, 30);
   assignment.set_cost(bot2, goal2, 20);
   assignment.set_cost(bot2, goal3, 10);
   assignment.set_cost(bot2, goal4, 40);
   assignment.set_cost(bot2, goal5, 30);

   // bot3 costs
   assignment.set_cost(bot3, goal1, 20);
   assignment.set_cost(bot3, goal2, 10);
   assignment.set_cost(bot3, goal3, 30);
   assignment.set_cost(bot3, goal4, 30);
   assignment.set_cost(bot3, goal5, 40);

   std::map<Robot, Point> solution;
   std::list<Point> unallocated_goals;
   uint32_t total_cost = assignment.run_solver(solution, unallocated_goals);
   BOOST_CHECK(total_cost == 30);
   BOOST_CHECK(solution[bot1] == goal1);
   BOOST_CHECK(solution[bot2] == goal3);
   BOOST_CHECK(solution[bot3] == goal2);
   BOOST_CHECK(unallocated_goals.size() == 2);
   // for (auto pt_iter = unallocated_goals.begin(); pt_iter !=
   // unallocated_goals.end(); ++pt_iter) {
   //    cout << *pt_iter << endl;
   // }
}

BOOST_AUTO_TEST_CASE(UnbalancedAssignmentMoreRobots) {
   Robot bot1(1, Point({1, 2}));
   Robot bot2(2, Point({2, 1}));
   Robot bot3(3, Point({3, 2}));

   Point goal1 = {5, 6};

   Assignment assignment;
   // bot1 costs
   assignment.set_cost(bot1, goal1, 20);

   // bot2 costs
   assignment.set_cost(bot2, goal1, 30);

   // bot3 costs
   assignment.set_cost(bot3, goal1, 10);

   std::map<Robot, Point> solution;
   std::list<Point> unallocated_goals;
   uint32_t total_cost = assignment.run_solver(solution, unallocated_goals);
   BOOST_CHECK(total_cost == 10);
   BOOST_CHECK(solution[bot3] == goal1);
   BOOST_CHECK(unallocated_goals.size() == 0);
}