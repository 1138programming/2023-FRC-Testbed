// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  /** Creates a new Hopper. */
  public Hopper() {
    leftMotor = new CANSparkMax(KHopperLeftID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(KHopperRightID, MotorType.kBrushless);
    
  }

  public void moveLeft(double speed) {
    leftMotor.set(speed);
  }
  public void moveRight(double speed) {
    rightMotor.set(speed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
