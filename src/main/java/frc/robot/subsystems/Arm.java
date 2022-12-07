// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  int distance;
  public Arm(int distance) 
  {
    this.distance = distance;
  }

  @Override
  public void periodic() {
    
  }
}
