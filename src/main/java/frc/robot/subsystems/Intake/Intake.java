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
    public static Intake pivot; 

    private final IntakeIO io; 
    private final IntakeIOAutoLogged inputs; 

    private double targetPosition = 1; //TODO: replace with a constant for the setpoint
    public double intakePercent; 
    public double pivotPercent; 
    private final PIDController extendPid; 
    private final ArmFeedforward extendFf; 

    private final Debouncer limitSwitchDebouncer; 

    public Intake(){
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
        

        if(inputs.position < targetPosition){
            io.setPivotMotor(10.0);
        }
        else if (inputs.position > targetPosition){
            io.setPivotMotor(-10.0);
        }
        

    }

    public static void setIntakePercentHold(){
        Intake.setIntakePercent(Constants.IntakeConstants.Percents.hold);
    
    }

    public static void setIntakePercentIntake(){
        Intake.setIntakePercent(Constants.IntakeConstants.Percents.intake);
    }

    public static void setIntakePercentShoot(){
        Intake.setIntakePercent(Constants.IntakeConstants.Percents.shoot);
    }

    public static void setIntakePercent(double percent){
        intake.setIntakePercentI(percent);
    }

    public void setIntakePercentI(double percent){
        intakePercent = percent; 
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1)* 12); 
    }

    public double getPosition(){
         return inputs.position; 
    }

    public static void movePositionIntake(){
        intake.movePosition(Constants.IntakeConstants.Setpoints.intake);
    }

    public void movePosition(double position){
        targetPosition = position; 
        extendPid.setSetpoint(position);
    }

  
    public static boolean hasNote(){
        return intake.hasNoteI();
    }
   

    private boolean hasNoteI(){
        return inputs.hasNote; 
    }

    public static boolean atSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atSetpoint'");
    }

    public static void movePositionShoot() {
       intake.movePosition(Constants.IntakeConstants.Setpoints.shoot);
    }
}
