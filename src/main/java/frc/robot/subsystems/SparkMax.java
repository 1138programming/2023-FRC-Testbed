// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMax extends SubsystemBase {
  CANSparkMax sparkMax;

  /** Creates a new Testing. */
  public SparkMax() {
    sparkMax = new CANSparkMax(KSparkMax, MotorType.kBrushless);
  }


  public void moveSparkMax(double speed) {
    sparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
