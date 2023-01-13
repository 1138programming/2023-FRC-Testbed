// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroupedTalons extends SubsystemBase {
  TalonSRX leftMotor;
  TalonSRX rightMotor;

  /** Creates a new Testing. */
  public GroupedTalons() {
    leftMotor = new TalonSRX(KTalon3);
    rightMotor = new TalonSRX(KTalon4);
  }

  public void moveLeft(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void moveRight(double speed) {
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
