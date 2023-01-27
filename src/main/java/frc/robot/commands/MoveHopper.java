// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import static frc.robot.Constants.*;

public class MoveHopper extends CommandBase {
  private Hopper hopper;
  private HOPPERDIRECTION direction;
  private double leftSpeed;
  private double rightSpeed;


  public enum HOPPERDIRECTION {
    UP,
    DOWN,
    LEFT,
    RIGHT
  }

  /** Creates a new MoveHopper. */
  public MoveHopper(Hopper hopper, HOPPERDIRECTION direction) {
    this.hopper = hopper;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (direction) {
      case UP: 
        leftSpeed = KHopperSpeed;
        rightSpeed = KHopperSpeed;
        break;
      case DOWN: 
        leftSpeed = -KHopperSpeed;
        rightSpeed = -KHopperSpeed;
        break;
      case LEFT: 
        leftSpeed = KHopperSpeed;
        rightSpeed = -KHopperSpeed;
        break;
      case RIGHT: 
        leftSpeed = - KHopperSpeed;
        rightSpeed = KHopperSpeed;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.moveLeft(leftSpeed);
    hopper.moveRight(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
