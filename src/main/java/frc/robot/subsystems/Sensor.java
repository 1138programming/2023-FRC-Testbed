package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  edu.wpi.first.wpilibj.DigitalInput;


public class Sensor extends SubsystemBase {
    private DigitalInput sensor;
    public Sensor () {
        sensor = new DigitalInput(3);
    }

    public void periodic() {
        if (sensor.get()) {
        System.out.println("limit switch has been held down");
        }
    }
}
