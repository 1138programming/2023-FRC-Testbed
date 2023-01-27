package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Scoring extends SubsystemBase {

    //motors used:
    // 2 spark-max neos
    // 1 775 motor
    // 2 pistons / 2 or 1 550s, confirm this later.
    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private Victor victor;
    public Scoring(int deviceID, MotorType type)
    {
        motorOne = new CANSparkMax(deviceID, type);
        motorTwo = new CANSparkMax(deviceID, type);
        victor = new Victor();
    }
}
