// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GridSubsystem.h"

#include <frc/Filesystem.h>
#include <fstream>
#include <iostream>
#include <algorithm>

// -------------------- Constructor --------------------
GridSubsystem::GridSubsystem() = default;

// -------------------- Periodic --------------------
void GridSubsystem::Periodic() {
  // Do NOT run A* here repeatedly
  // Path following will eventually go here
}

// -------------------- File Reading --------------------
void GridSubsystem::readFromFile() {
  std::ifstream inputFile(
      frc::filesystem::GetDeployDirectory() + "/mapFiles/map.txt");

  if (!inputFile.is_open()) {
    std::cout << "FAILED TO OPEN MAP FILE\n";
    return;
  }

  int row = 0;
  int col = 0;
  char c;

  while (inputFile.get(c)) {
    if (c == '\n' || c == '\r') continue;

    if (c != ' ') {
      field2DGrid[row][col] = c - '0';
      col++;
    }

    if (col == kWidth) {
      col = 0;
      row++;
      if (row == kHeight) break;
    }
  }

  inputFile.close();
}

// -------------------- Utility --------------------
double GridSubsystem::heuristic(int x1, int y1, int x2, int y2) const {
  return std::abs(x1 - x2) + std::abs(y1 - y2);  // Manhattan
}

bool GridSubsystem::inBounds(int x, int y) const {
  return x >= 0 && x < kWidth && y >= 0 && y < kHeight;
}

// -------------------- A* Pathfinding --------------------
bool GridSubsystem::findPath(int startX, int startY, int goalX, int goalY) {
    m_path.clear();
    m_pathFound = false;

    std::cout << field2DGrid[12][12] << std::endl;

    // Ensure the goal node is walkable
    if (field2DGrid[goalX][goalY] != 0) {
        // std::cout << "Goal node is blocked!\n";
        
        return false;
    }

    std::priority_queue<Node, std::vector<Node>, NodeCompare> openSet;

    bool closed[kHeight][kWidth] = {false};
    double gScore[kHeight][kWidth];
    std::fill(&gScore[0][0], &gScore[0][0] + kHeight * kWidth, 1e9);
    std::pair<int, int> parent[kHeight][kWidth];

    gScore[startY][startX] = 0;
    openSet.emplace(
        startX,
        startY,
        0,
        heuristic(startX, startY, goalX, goalY));

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (closed[current.y][current.x]) continue;
        closed[current.y][current.x] = true;

        // Goal reached
        if (current.x == goalX && current.y == goalY) {
            int cx = goalX;
            int cy = goalY;

            while (!(cx == startX && cy == startY)) {
                m_path.emplace_back(cx, cy);
                auto p = parent[cy][cx];
                cx = p.first;
                cy = p.second;
            }

            m_path.emplace_back(startX, startY);
            std::reverse(m_path.begin(), m_path.end());

            m_pathFound = true;

            std::cout << "\nPATH FOUND:\n";
            for (auto& cell : m_path) {
                std::cout << "(" << cell.first << ", " << cell.second << ") ";
                field2DGrid[cell.second][cell.first] = 2;  // mark path
            }
            std::cout << "\n";

            return true;
        }

        // Explore neighbors
        for (int i = 0; i < 4; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!inBounds(nx, ny)) continue;
            if (field2DGrid[ny][nx] != 0) continue;
            if (closed[ny][nx]) continue;

            double tentativeG = gScore[current.y][current.x] + 1.0;

            if (tentativeG < gScore[ny][nx]) {
                parent[ny][nx] = {current.x, current.y};
                gScore[ny][nx] = tentativeG;

                openSet.emplace(
                    nx,
                    ny,
                    tentativeG,
                    heuristic(nx, ny, goalX, goalY));
            }
        }
    }

    std::cout << "NO PATH FOUND\n";
    return false;
}

const std::vector<std::pair<int, int>>& GridSubsystem::getPath() const {
  return m_path;
}


void GridSubsystem::printGrid() {
  for (int y = 0; y < kHeight; y++) {
    for (int x = 0; x < kWidth; x++) {
      std::cout << field2DGrid[y][x] << " ";
    }
    std::cout << "\n";
  }
  std::cout << "----------------\n";
}
