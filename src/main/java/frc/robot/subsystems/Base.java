// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Base extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX topleftmotor;
  private TalonFX bottomleftmotor;
  private TalonFX toprightmotor;
  private TalonFX bottomrightmotor;


  
  public Base() {
 
  private TalonFX topleftmotor = new TalonFX(leftMotor1Port);
  private TalonFX bottomleftmotor = new TalonFX(leftMotor2Port);
  private TalonFX toprightmotor = new TalonFX(2);
  private TalonFX bottomrightmotor = new TalonFX(3);
  }

  public void Move(float leftSpeed, float rightSpeed)
  {
    topleftmotor.set.(ControlMode.PercentOutput,leftSpeed);
    bottomleftmotor.set.(ControlMode.PercentOutput,leftSpeed);
    toprightmotor.set.(ControlMode.PercentOutput,rightSpeed);
    bottomrightmotor.set.(ControlMode.PercentOutput,rightSpeed);
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
