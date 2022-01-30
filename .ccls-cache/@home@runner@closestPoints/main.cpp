#include "general/vectorUtility.h"
#include "general/numberUtility.h"
#include "general/ansi_codes.h"

#include <utility>
#include <functional>
#include <algorithm>

using vecUtil::operator<<;

struct Point {

  int x;
  int y;

  double dist(const Point& other) const {
    return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
  }

};

ostream& operator << (ostream& out, const Point& point) {
  out << "(" << point.x << ", " << point.y << ")";
  return out;
}

pair<Point, Point> closestPair_bruteForce(const vector<Point>& points) {

  if (points.size() <= 1) {
    throw "not enough points";
  }

  pair<Point, Point> minPair = {points.at(0), points.at(1)};
  double minDist = minPair.first.dist(minPair.second);

  Point pi, pr;
  double currDist;

  for (int i = 0; i < points.size() - 1; i++) {
    for (int r = i + 1; r < points.size(); r++) {

      pi = points.at(i);
      pr = points.at(r);
      currDist = pi.dist(pr);

      if (currDist < minDist) {
        minDist = currDist;
        minPair = {pi, pr};
      }

    }
  }

  return minPair;

}

vector<pair<Point, int>> copy(const vector<Point>& points) {
  vector<pair<Point, int>> theCopy;
  for (int i = 0; i < points.size(); i++) {
    theCopy.push_back({points.at(i), i});
  }
  return theCopy;
}

vector<Point> noPos(const vector<pair<Point, int>>& points) {
  vector<Point> noPosCopy;
  for (const auto& point : points) {
    noPosCopy.push_back(point.first);
  }
  return noPosCopy;
}

void resetPos(vector<pair<Point, int>>& points) {
  for (int i = 0; i < points.size(); i++) {
    points.at(i).second = i;
  }
}

pair<Point, Point> closestPairRecursive(const vector<pair<Point, int>>& Px, const vector<pair<Point, int>>& Py) {

  if (Px.size() <= 3) {
    return closestPair_bruteForce(noPos(Px));
  }

  int medianPos = Px.size() / 2;
  vector<pair<Point, int>> Qx, Qy, Rx, Ry;

  for (const auto& Px_point : Px) {
    if (Px_point.second < medianPos) {
      Qx.push_back(Px_point);
    } else {
      Rx.push_back(Px_point);
    }
  }
  resetPos(Rx);

  for (const auto& Py_point : Py) {
    if (Py_point.second < medianPos) {
      Qy.push_back(Py_point);
    } else {
      Ry.push_back(Py_point);
    }
  }

  for (auto& Ry_point : Ry) {
    Ry_point.second -= medianPos;
  }

  pair<Point, Point> q0q1 = closestPairRecursive(Qx, Qy);
  pair<Point, Point> r0r1 = closestPairRecursive(Rx, Ry);

  Point q0 = q0q1.first, q1 = q0q1.second, r0 = r0r1.first, r1 = r0r1.second;
  double delta = min(q0.dist(q1), r0.dist(r1));
  int xStar = Qx.back().first.x;

  vector<Point> Sy;
  for (const auto& Py_point : Py) {
    if (Py_point.first.dist({xStar, Py_point.first.y}) < delta) {
      Sy.push_back(Py_point.first);
    }
  }

  vector<pair<pair<Point, Point>, double>> distances;
  for (int i = 0; i < Sy.size(); i++) {
    for (int r = i + 1; r <= i + 15; r++) {
      if (r < Sy.size()) {
        distances.push_back({{Sy.at(i), Sy.at(r)}, Sy.at(i).dist(Sy.at(r))});
      }
    }
  }

  pair<Point, Point> minDistPairInS = min_element(distances.begin(), distances.end(), [] (const auto& dist1, const auto& dist2) {
    return dist1.second < dist2.second;
  })->first;
  Point s = minDistPairInS.first, sPrime = minDistPairInS.second;

  if (s.dist(sPrime) < delta) {
    return {s, sPrime};
  } else if (q0.dist(q1) < r0.dist(r1)) {
    return {q0, q1};
  } else {
    return {r0, r1};
  }

}

pair<Point, Point> closestPair(const vector<Point>& points) {
  vector<pair<Point, int>> Px = copy(points);
  sort(Px.begin(), Px.end(), [] (const auto& p1, const auto& p2) {
    return p1.first.x < p2.first.x;
  });
  resetPos(Px);
  vector<pair<Point, int>> Py = Px;
  sort(Py.begin(), Py.end(), [] (const auto& p1, const auto& p2) {
    return p1.first.y < p2.first.y;
  });
  return closestPairRecursive(Px, Py);
}

vector<Point> generatePoints(int numPoints, int minX, int maxX, int minY, int maxY) {
  vector<Point> points;
  for (int i = 1; i <= numPoints; i++) {
    points.push_back({randUtil::randint(minX, maxX), randUtil::randint(minY, maxY)});
  }
  return points;
}

void testAlgorithm(const vector<Point>& points, function<pair<Point, Point> (const vector<Point>&)> algorithm, bool dispVector = true) {
  if (dispVector) {
    cout << "points = " << points << "\n";
  }
  pair<Point, Point> result = algorithm(points);
  cout << ANSI_GREEN << "closest 2 points are " << result.first << " and " << result.second << ANSI_NORMAL;
}

int main(int argc, char** argv) {

  if (argc != 8) {
    cout << "\nUSAGE: ./a.out numPoints minX maxX minY maxY displayPoints algorithm\n"
         << "\nnumPoints = number of points"
         << "\nminX = minimum X-value of any point"
         << "\nmaxX = maximum X-value of any point"
         << "\nminY = minimum Y-value of any point"
         << "\nmaxY = maximum Y-value of any point"
         << "\ndisplayPoints = whether to display all points (0 = no, 1 = yes)"
         << "\nalgorithm = which algorithm to use (0 = brute force, 1 = smart, 2 = both)\n\n";
    return 0;
  }

  int numPoints = stoi(argv[1]), minX = stoi(argv[2]), maxX = stoi(argv[3]), minY = stoi(argv[4]), maxY = stoi(argv[5]), alg = stoi(argv[7]);
  bool dispVec = stoi(argv[6]);

  vector<Point> points = generatePoints(numPoints, minX, maxX, minY, maxY);

  if (alg != 1) {
    cout << ANSI_RED << "\nbrute force algorithm:\n" << ANSI_NORMAL;
    testAlgorithm(points, closestPair_bruteForce, dispVec);
    cout << "\n";
  }

  if (alg != 0) {
    cout << ANSI_RED << "\nsmart algorithm:\n" << ANSI_NORMAL;
    testAlgorithm(points, closestPair, dispVec);
    cout << "\n";
  }

  cout << "\n";

}