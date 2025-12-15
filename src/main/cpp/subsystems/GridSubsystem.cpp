// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GridSubsystem.h" 
#include "frc/Filesystem.h"
#include <frc/DriverStation.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <stdlib.h>
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cstdlib>


GridSubsystem::GridSubsystem() = default;

// This method will be called once per scheduler run
void GridSubsystem::Periodic() {}

void GridSubsystem::readFromFile() {
    std::ifstream inputFile(frc::filesystem::GetDeployDirectory() + "/mapFiles/map.txt");
    
    content.str("");
    content.clear();
    content << inputFile.rdbuf();
    inputFile.close();

    int row = 0;
    int col = 0;

    std::string raw = content.str();

    for (char c : raw) {
        if (c == '\n' || c == '\r') continue;

        if (c != ' ') {
            field2DGrid[row][col] = c - '0';
            col++;
        }
        

        if (col == 14) {
            col = 0;
            row++;

            if (row == 14) break;
        }


    }

    content.clear();
    content.seekp(0);
}

void GridSubsystem::findPath(int startX, int startY, int goalX, int goalY) {
    // A* Pathfinding implementation would go here
    // This is a placeholder for the actual pathfinding logic

    std::priority_queue<Node> openSet;
    std::unordered_set<Node, NodeHash, NodeEqual> closedSet;
    


    Node startNode(startX, startY, 0, 0, nullptr);
    startNode.h = std::abs(goalX - startX) + std::abs(goalY - startY);
    startNode.f = startNode.g + startNode.h;
    openSet.push(startNode);
    
    while (!openSet.empty()) {
        if (pathfound) {
            return;
        }
        Node current = openSet.top();
        openSet.pop();

        std::cout << "Processing Node: (" << current.x << ", " << current.y << ")" << std::endl;
        // std::cout << "Goal Node: (" << goalX << ", " << goalY << ")" << std::endl;
        // Check if we reached the goal
        if (current.x == goalX && current.y == goalY) {
            pathfound = true;
            // Reconstruct the path
            std::cout << "FOUNDDDDDDDD\n";
            return;
            // std::cout << "huh1\n";
            // std::cout << "Path: ";
            // Node* pathNode = &current;
            // while (pathNode != nullptr) {

            //     std::cout << "(" << pathNode->x << ", " << pathNode->y << ")";
            //     field2DGrid[pathNode->y][pathNode->x] = 2; // Mark the path on the grid
            //     pathNode = pathNode->parent;

            //     if (pathNode != nullptr) {
            //         std::cout << " -> ";
            //     }
            // }

            std::cout << std::endl;
            
            // return;
            // break;
        }
        // std::cout << "huh2\n";
        

        // Add the current node to the closed set
        closedSet.insert(current);

        // Check neighbors (up, down, left, right)
        std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (const auto& dir : directions) {
            int neighborX = current.x + dir.first;
            int neighborY = current.y + dir.second;

            // Check if the neighbor is within bounds
            if (neighborX < 0 || neighborX > 14 || neighborY < 0 || neighborY > 14) {
                continue;
            }

            // Check if the neighbor is walkable (e.g., not an obstacle)
            if (field2DGrid[neighborY][neighborX] != 0) {
                continue;
            }

            // Calculate costs for the neighbor
            double g = current.g + 1; // Assume uniform cost for moving to a neighbor
            double h = std::abs(goalX - neighborX) + std::abs(goalY - neighborY); // Manhattan distance
            double f = g + h;

            Node neighbor(neighborX, neighborY, g, h, &current);

            // Skip if the neighbor is already in the closed set
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }

            // Add the neighbor to the open set
            openSet.push(neighbor);
        }
    }
}

void GridSubsystem::printGrid() {
    

    // //publisher.Set(field2DGrid[0],196); //14*14=196
    // std::vector<double> my_doubles = {1.0, 2.0, 3.0};
    // // std::span<double> s(my_doubles); // Creates a span viewing my_doubles
    // frc::SmartDashboard::PutNumberArray("GridData", std::span(field2DGrid[0]));

    

    for (int i = 0; i < 14; i++) {
        for (int j = 0; j < 14; j++) {
            std::cout << field2DGrid[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Boom\n";

    
    // for (int r = 0; r < 14; r++) {
    //     for (int c = 0; c < 14; c++) {
    //         tab.Add(std::to_string(r) + " " + std::to_string(c),field2DGrid[r][c])
    //             .WithPosition(c, r)  // puts values in a grid layout
    //             .WithSize(1, 1);
    //     }
    // }

    
    // for (auto& a : field2DGrid) {
    //     for (auto& b : a) {
    //         std::cout << b << " ";
    //     }
    //     std::cout << std::endl;

    // }


}

