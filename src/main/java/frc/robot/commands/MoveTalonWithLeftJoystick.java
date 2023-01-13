// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LeftTalon;

public class MoveTalonWithLeftJoystick extends CommandBase {
  private LeftTalon talon;
  /** Creates a new MoveTalonWithJoystick. */
  public MoveTalonWithLeftJoystick(LeftTalon talon) {
    this.talon = talon;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talon.move(Robot.m_robotContainer.getLogiLeftXAxis());
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
