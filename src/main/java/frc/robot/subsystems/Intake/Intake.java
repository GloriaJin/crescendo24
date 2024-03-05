package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    
    public static Intake intake; 

    private final IntakeIO io; 
    private final IntakeIOAutoLogged inputs; 

    private double targetPosition = 1; //TODO: replace with a constant for the setpoint
    public double intakePercent; 
    public double pivotPercent; 
    private final PIDController extendPid; 
    private final ArmFeedforward extendFf; 

    private final Debouncer limitSwitchDebouncer; 

    private Intake(){
        intake = this; 
        inputs = new IntakeIOAutoLogged(); 

        io = new IntakeSparkMax(inputs); 
        extendPid = new PIDController(Constants.IntakeConstants.intakeP, Constants.IntakeConstants.intakeI, Constants.IntakeConstants.intakeD);
        extendFf = new ArmFeedforward(targetPosition, pivotPercent, intakePercent); //TODO: constants for ks, kg, kv, ka
        limitSwitchDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    }

    public static void init(){
        if(intake == null) 
            new Intake(); 
    }

    public void updateInputs(){
        io.updateInputs();
        inputs.hasNote = limitSwitchDebouncer.calculate(inputs.limitSwitch);
        Logger.processInputs("Intake", inputs);
    }

    @Override
    public void periodic(){
        inputs.positionSetpoint = targetPosition; 
        inputs.pivotPercent = pivotPercent; 
        inputs.intakePercent = intakePercent; 

        if(inputs.position < targetPosition){
            io.setPivotMotor(12.0);
        }
        else if (inputs.position > targetPosition){
            io.setPivotMotor(-12.0);
        }
        

    }

    public void setIntakePercentHold(){
        intake.setIntakePercent(Constants.IntakeConstants.Percents.hold);
    
    }
    public void setIntakePercent(double percent){
        intakePercent = percent; 
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1)* 12); 
    }
    
    public double getPosition(){ return inputs.position; }

    public static void movePositionIntake(){
        intake.movePosition(Constants.IntakeConstants.Setpoints.intake);
    }

    public void movePosition(double position){
        targetPosition = position; 
        extendPid.setSetpoint(position);
    }

    private boolean hasNote(){
        return inputs.hasNote; 
    }
}
