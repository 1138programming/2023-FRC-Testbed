// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VictorTest;

public class moveVictor extends CommandBase {
  private VictorTest victor;
  public moveVictor(VictorTest victor) {
    this.victor = victor;
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    victor.move(0.5);    
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
