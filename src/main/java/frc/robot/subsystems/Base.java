// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.*; 
import com.ctre.phoenix.motorcontrol.ControlMode;
public class  Base extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX Right_1; 
  private TalonFX Right_2;
  private TalonFX Left_1;
  private TalonFX Left_2; 
  
  public Base() {
    Right_1 = new TalonFX(kRight_1);
    Right_2 = new TalonFX(kRight_2);
    Left_1 = new TalonFX(kLeft_1);
    Left_2 = new TalonFX(KLeft_2);
  }
  public void Move (int Left_speed, int Right_speed ) {
    Right_1.set(ControlMode.PercentOutput,Right_speed);
    Right_2.set(ControlMode.PercentOutput,Right_speed);
    Left_1.set(ControlMode.PercentOutput,Left_speed);
    Left_2.set(ControlMode.PercentOutput,Left_speed);
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

