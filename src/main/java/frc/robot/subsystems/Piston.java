package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Piston extends SubsystemBase {

    DoubleSolenoid solenoid1;
    DoubleSolenoid solenoid2;
    DoubleSolenoid solenoid3;
    DoubleSolenoid solenoid4;
    DoubleSolenoid solenoid5;
    
    public Piston () {
        solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KSolenoid1ForwardChannel, KSolenoid1ReverseChannel );
        solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KSolenoid2ForwardChannel, KSolenoid2ReverseChannel); 
        solenoid3 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KSolenoid3ForwardChannel, KSolenoid3ReverseChannel); 
        solenoid4 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KSolenoid4ForwardChannel, KSolenoid4ReverseChannel); 
        solenoid4 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KSolenoid5ForwardChannel, KSolenoid5ReverseChannel); 
    } 

    public void setOff () {
        solenoid1.set(kOff);
        solenoid2.set(kOff);
        solenoid3.set(kOff);
        solenoid4.set(kOff);
        solenoid5.set(kOff);
    }
    
    public void setForward () {
        solenoid1.set(kForward);
        solenoid2.set(kForward);
        solenoid3.set(kForward);
        solenoid4.set(kForward);
        solenoid5.set(kForward);
    }
    
    public void  setReverse () {
        solenoid1.set(kReverse);
        solenoid2.set(kReverse);
        solenoid3.set(kReverse);
        solenoid4.set(kReverse);
        solenoid5.set(kReverse);
    }

}
