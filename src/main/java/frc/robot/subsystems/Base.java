// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
    private TalonFX leftBackBase;
    private TalonFX rightBackBase;
    private TalonFX leftFrontBase;
    private TalonFX rightFrontBase;
  /** Creates a new Base. */
  public Base() {
        leftBackBase = new TalonFX(7);
        rightBackBase = new TalonFX(8);
        leftFrontBase = new TalonFX(9);
        rightFrontBase = new TalonFX(10);
  }
  public void move() {
      leftBackBase.set(TalonFXControlMode.PercentOutput, 0.6);
      rightBackBase.set(TalonFXControlMode.PercentOutput, 0.6);
      leftFrontBase.set(TalonFXControlMode.PercentOutput, 0.6);
      rightFrontBase.set(TalonFXControlMode.PercentOutput, 0.6);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
