package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public abstract class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double position; 
        public double positionSetpoint; 
        public double velocity; 
        public double pivotPercent; 
        public double intakePercent; 
        public boolean limitSwitch; 
        public boolean hasNote; 
        public double voltsAppliedPivot; 
        public double voltsAppliedIntake; 
        public boolean brakePivot; 


    }

    protected IntakeIOAutoLogged inputs; 

    public abstract void updateInputs(); 

    public abstract void setIntakeMotor(double volts);

    public abstract void setPivotMotor(double volts);

    public abstract void setPivotBrake(boolean brake);
}
