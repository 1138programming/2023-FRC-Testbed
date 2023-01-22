// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonSRX right; 
  private TalonSRX left;
  private TalonSRX spaghetti;
  public Intake() {
    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    left = new TalonSRX(KLeftIntakeId);
    right = new TalonSRX(KRightIntakeId);
  }
    
  public void SpaghettiSpin (double speed){
    spaghetti.set(ControlMode.PercentOutput, speed);
  }

  public void SpaghettiStop () {
    spaghetti.set(ControlMode.PercentOutput, 0);
  }

  public void RollerSpin (double speed) {
    left.set(ControlMode.PercentOutput, speed);
    right.set(ControlMode.PercentOutput, -speed);
  }
  
  public void RollerStop () {
    left.set(ControlMode.PercentOutput, 0);
    right.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
