#pragma once

#include <frc2/command/SubsystemBase.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <iostream>

class GridSubsystem : public frc2::SubsystemBase {
 public:
  GridSubsystem();

  void Periodic() override;

  void readFromFile();
  void printGrid();

  // Call this ONCE when you want a path
  bool findPath(int startX, int startY, int goalX, int goalY);

  // Access the final path
  const std::vector<std::pair<int, int>>& getPath() const;

 private:
  static constexpr int kWidth  = 14;
  static constexpr int kHeight = 14;

  int field2DGrid[kHeight][kWidth] = {0};

  struct Node {
    int x, y;
    double g;
    double h;
    double f;

    Node(int x, int y, double g, double h)
        : x(x), y(y), g(g), h(h), f(g + h) {}
  };

  struct NodeCompare {
    bool operator()(const Node& a, const Node& b) const {
      return a.f > b.f;  // min-heap by f
    }
  };

  // Stores final reconstructed path
  std::vector<std::pair<int, int>> m_path;

  bool m_pathFound = false;

  double heuristic(int x1, int y1, int x2, int y2) const;
  bool inBounds(int x, int y) const;
};
