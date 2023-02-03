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

    public void setOff (int solenoidId) {
        switch (solenoidId) {
            case 1:
                solenoid1.set(kOff);
                break;
        
            case 2:
                solenoid2.set(kOff);
                break;

            case 3:
                solenoid3.set(kOff);
                break;

            case 4:
                solenoid4.set(kOff);

                break;
            case 5:
               solenoid5.set(kOff);

                break;
            
        }
        
       
    }
    
    public void setForward (int solenoidId) {
        switch (solenoidId) {
            case 1:
                solenoid1.set(kForward);
                break;
        
            case 2:
                solenoid2.set(kForward);
                break;

            case 3:
                solenoid3.set(kForward);
                break;

            case 4:
                solenoid4.set(kForward);
                break;
            case 5:
                solenoid5.set(kForward);
                break;
        }
    }
        
    
    public void  setReverse (int solenoidId) {
        switch (solenoidId) {
            case 1: solenoid1.set(kReverse);
                break;
            case 2: 
                solenoid2.set(kReverse);
                break;
            case 3: 
                solenoid3.set(kReverse);
                break;
            case 4: 
                solenoid4.set(kReverse);
                break;
            case 5: 
                solenoid5.set(kReverse);
                break;
        }
    }

}
