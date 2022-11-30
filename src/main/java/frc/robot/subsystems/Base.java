// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Base extends SubsystemBase {

private TalonFX leftMotor1;
private TalonFX leftMotor2;
private TalonFX rightMotor1;
private TalonFX rightMotor2;
    
/** Creates a new ExampleSubsystem. */
  public Base() 
  {
        leftMotor1 = new TalonFX(leftMotor1Port);
        leftMotor2 = new TalonFX(leftMotor2Port);
        rightMotor1 = new TalonFX(rightMotor1Port);
        rightMotor2 = new TalonFX(rightMotor2Port);
  }

    public void Move(float leftSpeed, float rightSpeed)
    {
        leftMotor1.set(ControlMode.PercentOutput,leftSpeed);
        leftMotor2.set(ControlMode.PercentOutput,leftSpeed);
        rightMotor1.set(ControlMode.PercentOutput,rightSpeed);
        rightMotor2.set(ControlMode.PercentOutput,rightSpeed);
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
