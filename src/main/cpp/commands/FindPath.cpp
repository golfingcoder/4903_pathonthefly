// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <commands/FindPath.h>
#include "subsystems/GridSubsystem.h"

FindPath::FindPath(GridSubsystem* m_gridSubsystem) : m_gridSubsystem(m_gridSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_gridSubsystem);
}

// Called when the command is initially scheduled.
void FindPath::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FindPath::Execute() {
  m_gridSubsystem->findPath(1, 1, 13, 13);
}

// Called once the command ends or is interrupted.
void FindPath::End(bool interrupted) {}

// Returns true when the command should end.
bool FindPath::IsFinished() {
  return false;
}
