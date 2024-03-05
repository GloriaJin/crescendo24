package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeSparkMax extends IntakeIO {
    

    public static boolean paralyzedPivot = false; 
    public static boolean paralyzedIntake = false; 

    private final CANSparkBase pivot; 
    private final CANSparkBase intake; 

    private final RelativeEncoder pivotEncoder; 
    private final DigitalInput limitSwitch; 

    public IntakeSparkMax(IntakeIOAutoLogged input){
        inputs = input; 
        pivot = new CANSparkMax(Constants.IntakeConstants.pivotId, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.IntakeConstants.intakeId, CANSparkBase.MotorType.kBrushless);
        
        pivotEncoder = pivot.getEncoder(); 
        pivotEncoder.setPositionConversionFactor(Constants.IntakeConstants.gearRatioPivot * 360); 
        pivotEncoder.setVelocityConversionFactor(Constants.IntakeConstants.gearRatioPivot * 360);
        pivotEncoder.setPosition(Constants.IntakeConstants.Setpoints.start);
        limitSwitch = new DigitalInput(Constants.IntakeConstants.limitSwitchId);
    
    }

    @Override
    public void updateInputs(){
        inputs.position = pivotEncoder.getPosition(); 
        inputs.velocity = pivotEncoder.getVelocity(); 
        inputs.limitSwitch = limitSwitch.get(); 
    }

     @Override
    public void setPivotMotor(double volts){
        inputs.voltsAppliedPivot = volts; 
        pivot.setVoltage(paralyzedPivot ? 0 : MathUtil.clamp(volts, -12.0, 12.0)); 
    }

    @Override
    public void setIntakeMotor(double volts){
        inputs.voltsAppliedIntake = volts; 
        intake.setVoltage(paralyzedIntake ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setPivotBrake(boolean brake){
        inputs.brakePivot = brake; 
        pivot.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
