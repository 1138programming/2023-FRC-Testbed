// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GroupedTalons;

public class MoveTalonsWithRightJoystick extends CommandBase {
  private GroupedTalons groupedTalons;
  /** Creates a new MoveTalonWithJoystick. */
  public MoveTalonsWithRightJoystick(GroupedTalons groupedTalons) {
    this.groupedTalons = groupedTalons;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groupedTalons);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groupedTalons.moveLeft(Robot.m_robotContainer.getXboxLeftYAxis());
    groupedTalons.moveRight(Robot.m_robotContainer.getXboxRightYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
