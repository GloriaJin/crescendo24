package frc.robot.subsystems.Climbing;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Piston {

    final DoubleSolenoid armSolenoid; 

    public Piston(PneumaticsModuleType module, int forwardChannel, int reverseChannel){
        armSolenoid = new DoubleSolenoid(module, forwardChannel, reverseChannel);
        armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }


    public void toggleCommand(){
        armSolenoid.toggle(); 
    }
}
