#include <boost/log/trivial.hpp>
#include <ostream>
#include <stdexcept>

#include "assignment.h"
#include "conflict_based_search.h"
#include "planner_starter_code.hpp"

#define EXCEPTION_MSG_MAX_SIZE 1024

using namespace std;

// logger setup
using namespace boost::log::trivial;
extern boost::log::sources::severity_logger<severity_level> lg;

// --------------- Point ---------------

std::ostream &operator<<(std::ostream &os, const Point &point) {
   os << "(" << point.x << ", " << point.y << ")";
   return os;
}

// --------------- Grid ---------------

bool Grid::isValidCell(const Point &p) const {
   return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
}

void Grid::setObstacle(const Point &p) {
   if (isValidCell(p)) {
      obstacles.insert(p);
      gridSearch.set_obstacle(p);
   }
}

void Grid::unsetObstacle(const Point &p) {
   if (isValidCell(p)) {
      obstacles.erase(p);
      gridSearch.unset_obstacle(p);
   }
}

void Grid::setDropoffLocation(const Point &p) {
   if (isValidCell(p)) {
      // initially drop off locations have 0 accumulative load
      dropOffLocations[p] = 0;
   }
}

void Grid::setDigLocation(const Point &p) {
   if (isValidCell(p)) {
      digLocations.insert(p);
      newDigLocation = true;
   }
}

void Grid::clearDigLocation(const Point &p) {
   digLocations.erase(p);
   gridSearch.unset_obstacle(p);
}

bool Grid::isObstacle(const Point &p) const { return obstacles.count(p); }

bool Grid::isDropoffLocation(const Point &p) const {
   return dropOffLocations.count(p);
}

bool Grid::isDigLocation(const Point &p) const { return digLocations.count(p); }

// when dropping off a payload at a drop off location we simply increment its
// value counter
void Grid::addToDropOffLocation(const Point &p) { dropOffLocations[p]++; }

int Grid::getDropOffLocationValue(const Point &p) {
   return dropOffLocations.at(p);
}

// sum up the values of each drop off location
int Grid::getCombinedDropOffLocationValues() {
   return std::accumulate(
       dropOffLocations.begin(), dropOffLocations.end(), 0,
       [](int acc, const auto &pair) { return acc + pair.second; });
}

int Grid::getWidth() { return width; }
int Grid::getHeight() { return height; }

std::vector<Point> Grid::getNeighbors(const Point &p) {
   std::vector<Point> neighbors;
   std::vector<Point> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

   for (const auto &dir : directions) {
      Point neighbor{p.x + dir.x, p.y + dir.y};
      if (isValidCell(neighbor)) {
         neighbors.push_back(neighbor);
      }
   }

   return neighbors;
}

std::size_t Grid::get_path(const Point &start_pt, const Point &end_pt,
                           std::vector<Point> &path) {
   path.clear();
   gridSearch.set_start(start_pt);
   gridSearch.set_goal(end_pt);
   std::size_t path_cost = gridSearch.solve(path);
   if (path_cost == 0) {
      char exception_msg[EXCEPTION_MSG_MAX_SIZE];
      snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
               "Grid search failed: (%d, %d) -> (%d, %d)", start_pt.x,
               start_pt.y, end_pt.x, end_pt.y);
      throw std::invalid_argument(exception_msg);
   }
   return path_cost;
}

Point Grid::find_best_dropoff_path(const Point &start_pt,
                                   std::vector<Point> &path) {
   std::size_t min_dist = 100000;
   Point best_dropoff;
   path.clear();
   for (const auto &[dropoff_pt, dropoff_count] : dropOffLocations) {
      vector<Point> cur_path;
      std::size_t cur_dist = get_path(start_pt, dropoff_pt, cur_path);
      if (cur_dist < min_dist) {
         min_dist = cur_dist;
         path = cur_path;
         best_dropoff = dropoff_pt;
      }
   }
   return best_dropoff;
}

// --------------- Robot ---------------

void Robot::setGoal(const Point &goal) {
   this->goal = goal;
   BOOST_LOG_SEV(lg, info) << "Robot " << id << " to " << goal << endl;
}

void Robot::executePlan(const std::vector<Point> &plan, bool dig_goal) {
   currentPlan = plan;
   currentPlanStep = 0;
   busy = true;
   digGoal = dig_goal;
   position = plan[0];
}

void Robot::advancePlan() {
   if (++currentPlanStep >= (currentPlan.size() - (digGoal ? 0 : 1))) {
      // finished this plan, reset
      BOOST_LOG_SEV(lg, info)
          << "Robot " << id << " completed plan " << currentPlan[0] << " -> "
          << currentPlan.back() << endl;
      currentPlanStep = 0;
      currentPlan.clear();
      busy = false;
      digGoal = false;
      // position member should still be accurate after the previous call to
      // this f-n
      return; // stopping bot, it remains at the current position
   }
   position = currentPlan[currentPlanStep];
   return;
}

Point Robot::getCurrentPosition() const { return position; }

void Robot::dig(std::shared_ptr<Grid> grid) {
   if (grid->isDigLocation(position)) {
      grid->clearDigLocation(position);
      BOOST_LOG_SEV(lg, info)
          << "Robot " << id << " dug at position (" << position.x << ", "
          << position.y << ")" << std::endl;
   }
}

void Robot::clearPlan() {
   busy = false;
   digGoal = false;
   currentPlanStep = 0;
   currentPlan.clear();
}

bool Robot::isInActivePath(const Point &check_pt) {
   if (currentPlan.size() == 0)
      return false;
   for (const auto &cur_pt : currentPlan) {
      if (cur_pt == check_pt)
         return true;
   }
   return false;
}

// --------------- Planner ---------------

void Planner::addRobot(std::shared_ptr<Robot> robot) {
   robots.push_back(robot);
}

// the planner checks frequently if there is work to perform (i.e., unassigned
// dig locations)
//  and creates plans for the robots accordingly.
void Planner::monitor() {
   ++totalTime;

   BOOST_LOG_SEV(lg, info) << "t = " << totalTime << endl;

   for (auto cur_bot : robots) {
      BOOST_LOG_SEV(lg, info) << "R" << cur_bot->get_id() << ": ";
      if (cur_bot->isBusy()) {
         bool dig_goal = cur_bot->isDigGoal();
         BOOST_LOG_SEV(lg, info) << "busy, ";
         if (dig_goal) {
            BOOST_LOG_SEV(lg, info) << "to dig" << endl;
         } else {
            BOOST_LOG_SEV(lg, info) << "to dropoff" << endl;
         }

         cur_bot->advancePlan();   // if at the end of the plan, will switch to
                                   // idle internally
         if (!cur_bot->isBusy()) { // bot just got idle, so it arrived
            if (dig_goal) {        // if it was a dig goal, then dig
               cur_bot->dig(grid);
               vector<Point> dropoff_path;
               Point dropoff = grid->find_best_dropoff_path(
                   cur_bot->getCurrentPosition(), dropoff_path);
               cur_bot->setGoal(dropoff);
               cur_bot->executePlan(dropoff_path, false);
            } else { // it was a dropoff goal
               grid->addToDropOffLocation(cur_bot->getGoal());
               replan();
            }
         }
      } // if (cur_bot->isBusy())
      else
         BOOST_LOG_SEV(lg, info) << "idle" << endl;
   } // end for (auto cur_bot : robots)

   if (!grid->isNewDigLocation())
      return; // if no new dig locations, we're done

   // got new dig location(s): force a replan for all robots not moving to a
   // dropoff
   BOOST_LOG_SEV(lg, info) << "New dig!" << endl;
   grid->acknowledgeNewDigLocation();
   replan();
}

int Planner::estimateDistanceHeuristic(const Point &start_pt,
                                       const Point &end_pt) {
   Path path;
   return int(grid->get_path(start_pt, end_pt, path));
}

void Planner::replan() {
   vector<shared_ptr<Robot>> available_robots;
   for (auto cur_bot : robots) {
      if (cur_bot->isBusy()) {
         if (cur_bot->isDigGoal()) {
            cur_bot->clearPlan();
            available_robots.push_back(cur_bot);
         }
      } else
         available_robots.push_back(cur_bot);
   }

   BOOST_LOG_SEV(lg, debug)
       << available_robots.size() << " robots, "
       << grid->getDigLocations().size() << " dig locations" << endl;

   // setup the assignment solver
   Assignment assignment;
   for (auto cur_bot : available_robots) {
      for (const auto &cur_pt : grid->getDigLocations()) {
         size_t cost =
             estimateDistanceHeuristic(cur_bot->getCurrentPosition(), cur_pt);
         assignment.set_cost(*cur_bot, cur_pt, cost);
      }
   }

   // run the assignment solver
   std::map<Robot, Point> solution;
   std::list<Point> unallocated_goals;
   BOOST_LOG_SEV(lg, debug)
       << "available robots: " << available_robots.size() << endl;
   assignment.run_solver(solution, unallocated_goals);

   // setup start and goal point arrays to plug into CBS
   vector<Point> start_points;
   vector<Point> goal_points;
   for (const auto &[solution_robot, solution_goal] : solution) {
      start_points.push_back(solution_robot.getCurrentPosition());
      goal_points.push_back(solution_goal);
   }
   PathSet solution_paths;
   ConflictBasedSearch cbs_planner(this->grid, solution.size());
   cbs_planner.search(start_points, goal_points, solution_paths);

   // set the computed paths for execution:
   // we reiterate through the assignment solution std::map in order
   // to make sure to preserve robot ID - to - vector index mapping
   // that we used to setup the start_points vector
   // (reconsider ConflictBasedSearch API to simplify this)
   size_t path_idx = 0;
   for (const auto &[solution_robot, solution_goal] : solution) {
      // TODO: the inner loop below is needed since sol_iter->first is returned
      // as a const. Would need to move away from the std::map container.
      for (auto cur_bot : robots) {
         if (cur_bot->get_id() == solution_robot.get_id()) {
            cur_bot->setGoal(solution_goal);
            cur_bot->executePlan(solution_paths[path_idx], true);
            break;
         }
      }
      ++path_idx;
   }
}

void printState(const std::shared_ptr<Grid> &grid,
                const std::vector<std::shared_ptr<Robot>> &robots) {
   for (int x = 0; x < grid->getWidth(); ++x) {
      for (int y = 0; y < grid->getHeight(); ++y) {
         Point p{x, y};
         std::string cell = ".";

         for (const auto &robot : robots) {
            if (robot->getCurrentPosition() == p) {
               cell = "R";
               break;
            }
            // check if this point is in any motion plans
            if (robot->isInActivePath(p)) {
               cell = "*";
               break;
            }
         }

         if (grid->isObstacle(p)) {
            cell = "#";
         } else if (grid->isDigLocation(p)) {
            cell = "D";
         } else if (grid->isDropoffLocation(p)) {
            cell = std::to_string(grid->getDropOffLocationValue(p));
         }

         std::cout << cell << ' ';
      }
      std::cout << std::endl;
   }
   std::cout << std::endl;
}
