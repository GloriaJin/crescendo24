package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Mecanum.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;


public class Drive extends CommandBase {

    public final Mecanum mecanum; 
    public final XboxController controller; 
    
    public Drive(Mecanum mecanum, XboxController controller){
        this.mecanum = mecanum; 
        this.controller = controller; 
        addRequirements(mecanum);
    }

    @Override
    public void execute(){
        mecanum.driveFieldOriented(controller);
    }
}
