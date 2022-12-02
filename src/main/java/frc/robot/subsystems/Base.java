package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Base extends SubsystemBase {
    
    private TalonFX leftMotor1;
    private TalonFX leftMotor2;
    private TalonFX rightMotor1;
    private TalonFX rightMotor2;

    public Base()
    {
        leftMotor1 = new TalonFX(leftMotorPort1);
        leftMotor2 = new TalonFX(leftMotorPort2);
        rightMotor1 = new TalonFX(leftMotorPort1);
        rightMotor2 = new TalonFX(leftMotorPort1);
    }

    public void Move(double leftSpeed, double rightSpeed)
    {
        leftMotor1.set(ControlMode.PercentOutput,leftSpeed);
        leftMotor2.set(ControlMode.PercentOutput,leftSpeed);
        rightMotor1.set(ControlMode.PercentOutput,rightSpeed);
        rightMotor2.set(ControlMode.PercentOutput,rightSpeed);
    }

    @Override
    public void periodic() {
        // will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // will be called once per scheduler run during simulation
    }
}
