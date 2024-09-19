#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ostream>

#include <boost/functional/hash.hpp>

#ifndef PLANNER_STARTER_CODE_H
#define PLANNER_STARTER_CODE_H

// class Point {
//  public: 
//    Point();
//    Point(int x_input, int y_input);

struct Point
{
   int x, y;

   inline bool operator==(const Point &other) const {
      return x == other.x && y == other.y;
   }

   struct Hash {
      size_t operator()(const Point &self) const {
         size_t seed = 0;
         boost::hash_combine(seed, self.x);
         boost::hash_combine(seed, self.y);
         return seed;
      }
   };

   struct Equality {
      bool operator()(const Point &lhs, const Point &rhs) const {
         return lhs == rhs;
      }
   };
};

// used to enable Point objects to be used as keys in unordered maps or sets.
namespace std {
template <> struct hash<Point> {
   size_t operator()(const Point &p) const {
      return hash<int>()(p.x) ^ hash<int>()(p.y);
   }
};
} // namespace std


std::ostream& operator<<(std::ostream& os, const Point& point);

// --------------- Grid ---------------

class Grid {
 public:
   Grid(int width, int height) : width(width), height(height) {}

   bool isValidCell(const Point &p) const;

   void setObstacle(const Point &p);

   void setDropoffLocation(const Point &p);

   void setDigLocation(const Point &p);

   void clearDigLocation(const Point &p);

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

 private:
   int width, height;
   std::unordered_set<Point, Point::Hash, Point::Equality> obstacles;
   std::unordered_map<Point, int, Point::Hash, Point::Equality>
       dropOffLocations;
   std::unordered_set<Point, Point::Hash, Point::Equality> digLocations;

   std::vector<Point> getNeighbors(const Point &p);
};

// --------------- Robot ---------------

class Robot {
 public:
   Robot(int id, const Point &start) : id(id), position(start) {}

   void setGoal(const Point &goal);

   void executePlan(const std::vector<Point> &plan);

   Point getCurrentPosition() const;

   void dig(std::shared_ptr<Grid> grid);

   friend bool operator==(const Robot &lhs, const Robot &rhs) {
      return lhs.id == rhs.id;
   }

   int get_id() const { return id; }

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

   friend bool operator<(const Robot &lhs, const Robot &rhs) { return lhs.id < rhs.id; }

 private:
   int id;
   Point position;
   Point goal;
   bool isBusy = false;
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
};

void printState(const std::shared_ptr<Grid> &grid,
                const std::vector<std::shared_ptr<Robot>> &robots);

#endif
