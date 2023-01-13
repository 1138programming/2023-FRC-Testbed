// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talon extends SubsystemBase {
  TalonSRX talon1;
  TalonSRX talon2;
  TalonSRX talon3;
  TalonSRX talon4;

  /** Creates a new Testing. */
  public Talon() {
    talon1 = new TalonSRX(KTalon1);
    talon2 = new TalonSRX(KTalon2);
    talon3 = new TalonSRX(KTalon3);
    talon4 = new TalonSRX(KTalon4);
  }

  public void moveTalon(int num, double speed) {
    switch (num) {
      case 1:
       talon1.set(ControlMode.PercentOutput, speed);
       break;
      case 2:
       talon2.set(ControlMode.PercentOutput, speed);
        break;
       case 3:
       talon3.set(ControlMode.PercentOutput, speed);
       break;
      case 4:
       talon4.set(ControlMode.PercentOutput, speed);
       break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
