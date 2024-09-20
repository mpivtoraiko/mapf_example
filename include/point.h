#pragma once

struct Point {
   int x, y;

   inline bool operator==(const Point &other) const {
      return x == other.x && y == other.y;
   }

   inline int LinfDistance(const Point &other) const {
      return std::min(std::abs(x - other.x), std::abs(y - other.y));
   }

   inline int L1Distance(const Point &other) const {
      return std::abs(x - other.x) + std::abs(y - other.y);
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

std::ostream &operator<<(std::ostream &os, const Point &point);
