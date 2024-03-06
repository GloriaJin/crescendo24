package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeGround extends Command{
    
    Timer timer = new Timer(); 

    public IntakeGround(){
        addRequirements(Intake.intake);
    }

    @Override
    public void initialize(){
       
    }

    @Override
    public void execute(){
        
        
    }

    
    public void positionIntake(){
        Intake.movePositionIntake();
    }

    public void intakeMotor(){
        Intake.setIntakePercentIntake(); 
    }

/* TODO: find setpoint / set in constants 
    @Override 
    public boolean isFinished(){
        return timer.get() > 1 && Intake.atSetpoint(); 
    }*/

    public void end(){
        Intake.setIntakePercent(0);
    }

    
}
