// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GridSubsystem.h" 
#include <frc/DriverStation.h>
#include <fstream>
#include <string>
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <stdlib.h>
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <frc/smartdashboard/SmartDashboard.h>


GridSubsystem::GridSubsystem() = default;

// This method will be called once per scheduler run
void GridSubsystem::Periodic() {}

void GridSubsystem::readFromFile() {
    std::string line;
    std::ifstream inputFile;
    
    

    inputFile.open("src\\main\\mapFiles\\map.txt");
    if (!inputFile.is_open()) {
        return;
    }
    int row = 0;
    
    while (std::getline(inputFile, line) && row < 14) {
        int col = 0;
        for (char c : line) {
            if (col < 14) {
                field2DGrid[row][col] = 5.0;   // convert char → int
                col++;
            }
        }
        row++;
    }
    inputFile.close();
    

}

void GridSubsystem::printGrid() {
    

    //publisher.Set(field2DGrid[0],196); //14*14=196
    std::vector<double> my_doubles = {1.0, 2.0, 3.0};
    // std::span<double> s(my_doubles); // Creates a span viewing my_doubles
    frc::SmartDashboard::PutNumberArray("GridData", std::span(field2DGrid[0]));
    std::cout << "hello\n";

    
    for (int r = 0; r < 14; r++) {
        for (int c = 0; c < 14; c++) {
            tab.Add(std::to_string(r) + " " + std::to_string(c),field2DGrid[r][c])
                .WithPosition(c, r)  // puts values in a grid layout
                .WithSize(1, 1);
        }
    }

    
    // for (auto& a : field2DGrid) {
    //     for (auto& b : a) {
    //         std::cout << b << " ";
    //     }
    //     std::cout << std::endl;

    // }


}

