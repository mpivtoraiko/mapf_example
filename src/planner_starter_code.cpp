#include "planner_starter_code.hpp"


std::ostream& operator<<(std::ostream& os, const Point& point)
{
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
   }
}

void Grid::setDropoffLocation(const Point &p) {
   if (isValidCell(p)) {
      dropOffLocations[p] =
          0; // initially drop off locations have 0 accumulative load
   }
}

void Grid::setDigLocation(const Point &p) {
   if (isValidCell(p)) {
      digLocations.insert(p);
   }
}

void Grid::clearDigLocation(const Point &p) { digLocations.erase(p); }

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

// --------------- Robot ---------------

void Robot::setGoal(const Point &goal) { this->goal = goal; }

void Robot::executePlan(const std::vector<Point> &plan) {
   for (const auto &p : plan) {
      position = p;
   }
}

Point Robot::getCurrentPosition() const { return position; }

void Robot::dig(std::shared_ptr<Grid> grid) {
   if (grid->isDigLocation(position)) {
      grid->clearDigLocation(position);
      std::cout << "Robot " << id << " dug at position (" << position.x << ", "
                << position.y << ")" << std::endl;
   }
}

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

// --------------- Planner ---------------

void Planner::addRobot(std::shared_ptr<Robot> robot) {
   robots.push_back(robot);
}

// the planner checks frequently if there is work to perform (i.e., unassigned
// dig locations)
//  and creates plans for the robots accordingly.
void Planner::monitor() {

   // TODO: add code here
}

void printState(const std::shared_ptr<Grid> &grid,
                const std::vector<std::shared_ptr<Robot>> &robots) {
   for (int x = 0; x < grid->getWidth(); ++x) {
      for (int y = 0; y < grid->getHeight(); ++y) {
         Point p{x, y};
         std::string cell = ".";
         if (grid->isObstacle(p)) {
            cell = "#";
         } else if (grid->isDigLocation(p)) {
            cell = "D";
         } else if (grid->isDropoffLocation(p)) {
            cell = std::to_string(grid->getDropOffLocationValue(p));
         }
         for (const auto &robot : robots) {
            if (robot->getCurrentPosition() == p) {
               cell = "R";
               break;
            }
         }
         std::cout << cell << ' ';
      }
      std::cout << std::endl;
   }
   std::cout << std::endl;
}
