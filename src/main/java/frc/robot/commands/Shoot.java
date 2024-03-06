package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class Shoot extends Command{
  
    
    Timer timer = new Timer(); 

    public Shoot(){
        addRequirements(Intake.intake);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        Intake.movePositionShoot();
        
    }

    public void positionShoot(){
        Intake.movePositionShoot();
    }

    public void intakeShoot(){
        Intake.setIntakePercentShoot(); 
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
