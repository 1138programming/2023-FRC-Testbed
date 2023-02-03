// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Piston;

public class PistonSet extends CommandBase {
  /** Creates a new PistonSet. */
  
  Piston piston;
  int pistonNum;

  public enum SETCOMMAND {
    FORWARD,
    BACKWARD,
    OFF
  }
  
  SETCOMMAND hello;
  public PistonSet(Piston piston, SETCOMMAND hello, int pistonNum) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.piston = piston;
    this.hello = hello;
    this.pistonNum = pistonNum;

    addRequirements(piston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      switch(hello) {
        case FORWARD:
          piston.setForward(pistonNum);
          break;
        case BACKWARD:
          piston.setReverse(pistonNum);
          break;
        case OFF:
          piston.setOff(pistonNum);
        
      }
    }
        
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
        
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
