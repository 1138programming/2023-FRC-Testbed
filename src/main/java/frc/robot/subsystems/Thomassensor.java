// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.*;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource; 

public class Thomassensor extends SubsystemBase  {
    private TalonFX Good_Motor; 
    private DigitalInput mysensor; 

    public Thomassensor() {
        Good_Motor = new TalonFX(KcoolMotor);
        
    }
    
    public void moveUp(float speed) {
        Good_Motor.set(ControlMode.PercentOutput, speed );

    }
    public boolean getSensor() {
        return !(mysensor.get()); 
    }
    public void periodic() {
        
    }
    

    
}
