package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Piston extends SubsystemBase {

    DoubleSolenoid solenoid1;
    
    public Piston () {
        solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KForwardChannel, KReverseChannel); 
    } 

    public void setOff () {
        solenoid1.set(kOff);
    }
    
    public void setForward () {
        solenoid1.set(kForward);
    }
    
    public void  setReverse () {
        solenoid1.set(kReverse);
    }

}
