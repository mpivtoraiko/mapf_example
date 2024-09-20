#define DEBUG

#ifdef DEBUG
#include <cstdio>
#include <ostream>
#endif

#include <stdexcept>

#include "assignment.h"
#include "planner_starter_code.hpp"

#define EXCEPTION_MSG_MAX_SIZE 1024

using namespace std;

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

void Grid::setDropoffLocation(const Point &p) {
   if (isValidCell(p)) {
      // initially drop off locations have 0 accumulative load
      dropOffLocations[p] = 0; 
      //gridSearch.set_obstacle(p);
   }
}

void Grid::setDigLocation(const Point &p) {
   if (isValidCell(p)) {
      digLocations.insert(p);
      //gridSearch.set_obstacle(p);
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


std::size_t Grid::get_path(const Point &start_pt, const Point &end_pt, std::vector<Point> &path)
{
   path.clear();
   gridSearch.set_start(start_pt);
   gridSearch.set_goal(end_pt);
   std::size_t path_cost = gridSearch.solve(path);
   if (path_cost == 0) {
      char exception_msg[EXCEPTION_MSG_MAX_SIZE];
      snprintf(exception_msg, EXCEPTION_MSG_MAX_SIZE,
               "Grid search failed: (%d, %d) -> (%d, %d)",
               start_pt.x, start_pt.y, end_pt.x, end_pt.y);
      throw std::invalid_argument(exception_msg);
   }
   return path_cost;
}


Point Grid::find_best_dropoff_path(const Point &start_pt, std::vector<Point> &path)
{
   std::size_t min_dist = 100000;
   Point best_dropoff;
   path.clear();
   for (auto cur_dropoff = dropOffLocations.begin(); cur_dropoff != dropOffLocations.end(); ++cur_dropoff) {
      vector<Point> cur_path;
      std::size_t cur_dist = get_path(start_pt, cur_dropoff->first, cur_path);
      if (cur_dist < min_dist) {
         min_dist = cur_dist;
         path = cur_path;
         best_dropoff = cur_dropoff->first;
      }
   }
   return best_dropoff;
}


// --------------- Robot ---------------

void Robot::setGoal(const Point &goal) {
   this->goal = goal;
   cout << "Robot " << id << " to " << goal << endl;
}

void Robot::executePlan(const std::vector<Point> &plan, bool dig_goal) {
   currentPlan = plan;
   currentPlanStep = 0;
   busy = true;
   digGoal = dig_goal;
   position = plan[0];
   // cout << "R" << id << ": ";
   // for (auto cur_pt : plan) {
   //    cout << cur_pt << ", ";
   // }
   // cout << endl;
}

void Robot::advancePlan() {
   if (++currentPlanStep >= (currentPlan.size() - (digGoal ? 0 : 1))) {
      // finished this plan, reset
      cout << "Robot " << id << " completed plan " << currentPlan[0] << " -> "
           << currentPlan.back() << endl;
      currentPlanStep = 0;
      currentPlan.clear();
      busy = false;
      digGoal = false;
      // position member should still be accurate after the previous call to
      // this f-n
      return;
   }
   position = currentPlan[currentPlanStep];
   return;
}

Point Robot::getCurrentPosition() const { return position; }

void Robot::dig(std::shared_ptr<Grid> grid) {
   if (grid->isDigLocation(position)) {
      grid->clearDigLocation(position);
      std::cout << "Robot " << id << " dug at position (" << position.x << ", "
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
   for (auto cur_pt : currentPlan) {
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

   printf("t = %d\n", totalTime);

   for (auto cur_bot : robots) {
      printf("R%d: ", cur_bot->get_id());
      if (cur_bot->isBusy()) {
         bool dig_goal = cur_bot->isDigGoal();
         printf("busy, ");
         dig_goal ? printf("to dig\n") : printf("to dropoff\n");
         cur_bot->advancePlan(); // if at the end of the plan, will switch to
                                 // idle internally
         if (!cur_bot->isBusy()) { // bot just got idle, so it arrived           
            if (dig_goal) { // if it was a dig goal, then dig
               cur_bot->dig(grid);
               vector<Point> dropoff_path;
               Point dropoff = grid->find_best_dropoff_path(cur_bot->getCurrentPosition(), dropoff_path);
               cur_bot->setGoal(dropoff);
               cur_bot->executePlan(dropoff_path, false);
            }
            else { // it was a dropoff goal
               grid->addToDropOffLocation(cur_bot->getGoal());
               replan();
            }
         }
      } // if (cur_bot->isBusy()) 
      else
         printf("idle\n");
   } // end for (auto cur_bot : robots)

   if (!grid->isNewDigLocation())
      return; // if no new dig locations, we're done

   // got new dig location(s): force a replan for all robots not moving to a
   // dropoff
   printf("New digs!\n");
   grid->acknowledgeNewDigLocation();
   replan();
}

int Planner::estimateDistanceHeuristic(const Point &start_pt,
                                       const Point &end_pt) {
   //return start_pt.L1Distance(end_pt); // L1 distance for now
   std::vector<Point> path;
   return int(grid->get_path(start_pt, end_pt, path));
}


void Planner::replan() 
{
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

   Assignment assignment;
   for (auto cur_bot : available_robots) {
      for (auto cur_pt : grid->getDigLocations()) {
         //         Point bot_position = cur_bot->getCurrentPosition();
         uint32_t cost =
             estimateDistanceHeuristic(cur_bot->getCurrentPosition(), cur_pt);
         assignment.set_cost(*cur_bot, cur_pt, cost);
      }
   }
   std::map<Robot, Point> solution;
   std::list<Point> unallocated_goals;
   assignment.run_solver(solution, unallocated_goals);
   for (map<Robot, Point>::iterator sol_iter = solution.begin();
        sol_iter != solution.end(); ++sol_iter) {
      // TODO: the inner loop below is needed since sol_iter->first is returned
      // as a const. Would need to move away from the std::map container.
      for (auto cur_bot : robots) {
         if (cur_bot->get_id() == sol_iter->first.get_id()) {
            cur_bot->setGoal(sol_iter->second);
            std::vector<Point> path;
            grid->get_path(cur_bot->getCurrentPosition(), sol_iter->second, path);
            cur_bot->executePlan(path, true);
            break;
         }
      }
   }

   // TODO: run CBS and set plans to all bots
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


