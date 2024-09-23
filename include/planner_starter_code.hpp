#ifndef PLANNER_STARTER_CODE_H
#define PLANNER_STARTER_CODE_H

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "grid_search.h"
#include "point.h"

// --------------- Grid ---------------

class Grid {
 public:
   Grid(int width, int height)
       : width(width), height(height), newDigLocation(false),
         gridSearch(width, height) {}

   bool isValidCell(const Point &p) const;

   void setObstacle(const Point &p);

   void unsetObstacle(const Point &p);

   void setDropoffLocation(const Point &p);

   void setDigLocation(const Point &p);

   void clearDigLocation(const Point &p);

   inline bool isNewDigLocation() { return newDigLocation; }

   inline void acknowledgeNewDigLocation() { newDigLocation = false; }

   inline const std::unordered_set<Point, Point::Hash, Point::Equality> &
   getDigLocations() {
      return digLocations;
   }

   bool isObstacle(const Point &p) const;

   bool isDropoffLocation(const Point &p) const;

   bool isDigLocation(const Point &p) const;

   // when dropping off a payload at a drop off location we simply increment its
   // value counter
   void addToDropOffLocation(const Point &p);

   int getDropOffLocationValue(const Point &p);

   // sum up the values of each drop off location
   int getCombinedDropOffLocationValues();

   int getWidth();
   int getHeight();

   std::size_t get_path(const Point &start_pt, const Point &end_pt,
                        std::vector<Point> &path);
   Point find_best_dropoff_path(const Point &start_pt,
                                std::vector<Point> &path);

 private:
   int width, height;
   std::unordered_set<Point, Point::Hash, Point::Equality> obstacles;
   std::unordered_map<Point, int, Point::Hash, Point::Equality>
       dropOffLocations;
   std::unordered_set<Point, Point::Hash, Point::Equality> digLocations;

   std::vector<Point> getNeighbors(const Point &p);
   bool newDigLocation;

   GridSearch gridSearch;
};

// --------------- Robot ---------------

class Robot {
 public:
   Robot(int id, const Point &start)
       : id(id), position(start), busy(false), digGoal(false) {}

   void setGoal(const Point &goal);
   inline Point getGoal() { return goal; }

   void executePlan(const std::vector<Point> &plan, bool dig_goal = false);

   void advancePlan();

   void clearPlan();

   Point getCurrentPosition() const;

   void dig(std::shared_ptr<Grid> grid);

   friend bool operator==(const Robot &lhs, const Robot &rhs) {
      return lhs.id == rhs.id;
   }

   int get_id() const { return id; }

   inline bool isBusy() { return busy; }

   inline bool isDigGoal() { return digGoal; }

   bool isInActivePath(const Point &check_pt);

   struct Hash {
      size_t operator()(const Robot &a) const {
         size_t seed = 0;
         boost::hash_combine(seed, a.id);
         return seed;
      }
   };

   struct Equality {
      bool operator()(const Robot &lhs, const Robot &rhs) const {
         return lhs.id == rhs.id;
      }
   };

   friend bool operator<(const Robot &lhs, const Robot &rhs) {
      return lhs.id < rhs.id;
   }

 private:
   int id;
   Point position;
   Point goal;
   bool busy;
   bool digGoal;
   std::vector<Point> currentPlan;
   size_t currentPlanStep;
};

// --------------- Planner ---------------

class Planner {
 public:
   Planner(std::shared_ptr<Grid> grid) : grid(grid), totalTime(0) {}

   void addRobot(std::shared_ptr<Robot> robot);

   void monitor();

 private:
   std::shared_ptr<Grid> grid;
   std::vector<std::shared_ptr<Robot>> robots;
   int totalTime;
   // std::unordered_map<std::shared_ptr<Robot>, std::vector<Point>>

   int estimateDistanceHeuristic(const Point &start_pt, const Point &end_pt);
   void replan();
};

void printState(const std::shared_ptr<Grid> &grid,
                const std::vector<std::shared_ptr<Robot>> &robots);

#endif
