// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <set>
#include <queue>
#include <unordered_set>


class GridSubsystem : public frc2::SubsystemBase {
 public:
  GridSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void readFromFile();
  void printGrid();
  void findPath(int startX, int startY, int goalX, int goalY);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  int field2DGrid[14][14];
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("ARRAYY");
  std::ostringstream content;
  struct Node {
    int x, y;
    double g;
    double h;
    double f;
    Node* parent;

    Node(int x, int y, double g = 0, double h = 0, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

    bool operator<(const Node& other) const {
        return f > other.f; 
    }

  };
//   std::priority_queue<Node> openSet;

  struct NodeHash {
      size_t operator()(const Node& node) const {
          return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
      }
  };

  struct NodeEqual {
      bool operator()(const Node& a, const Node& b) const {
          return a.x == b.x && a.y == b.y;
      }
  };
  bool pathfound = false;

    std::vector<std::pair<int, int>> path; // To store the path in reverse order
    std::set<Node*> visitedNodes; // To track visited nodes


//   std::unordered_set<Node> closedSet
};
