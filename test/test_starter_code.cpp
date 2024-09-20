#define BOOST_TEST_MODULE StarterCodeTests

#include "planner_starter_code.hpp"
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(RobotInit) {
   Robot bot(1, Point({1, 2}));
   BOOST_CHECK(bot.getCurrentPosition().x == 1 &&
               bot.getCurrentPosition().y == 2);
}