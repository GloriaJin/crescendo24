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
        Intake.movePositionIntake();
    }
}
