package frc.robot.subsystems.Intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
   
    @Override
    public void toLog(LogTable table){
        table.put("Position", position);
        table.put("PositionSetpoint", positionSetpoint);
        table.put("Velocity", velocity);
        table.put("PivotPercent", pivotPercent);
        table.put("IntakePercent", intakePercent);
        table.put("LimitSwitch", limitSwitch);
        table.put("HasNote", hasNote);
        table.put("VoltsAppliedPivot", voltsAppliedPivot);
        table.put("VoltsAppliedIntake", voltsAppliedIntake);
        table.put("BrakePivot", brakePivot);
    }

    @Override
    public void fromLog(LogTable table){
        position = table.get("Position", position);
        positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
        velocity = table.get("Velocity", velocity);
        pivotPercent = table.get("PivotPercent", pivotPercent);
        intakePercent = table.get("IntakePercent", intakePercent);
        limitSwitch = table.get("LimitSwitch", limitSwitch);
        hasNote = table.get("HasNote", hasNote);
        voltsAppliedPivot = table.get("VoltsAppliedPivot", voltsAppliedPivot);
        voltsAppliedIntake = table.get("VoltsAppliedIntake", voltsAppliedIntake);
        brakePivot = table.get("BrakePivot", brakePivot);
    
    }

    public IntakeIOAutoLogged clone() {
        IntakeIOAutoLogged copy = new IntakeIOAutoLogged();
        copy.position = this.position;
        copy.positionSetpoint = this.positionSetpoint;
        copy.velocity = this.velocity;
        copy.pivotPercent = this.pivotPercent; 
        copy.intakePercent = this.intakePercent;
        copy.limitSwitch = this.limitSwitch;
        copy.hasNote = this.hasNote;
        copy.voltsAppliedPivot = this.voltsAppliedPivot;
        copy.voltsAppliedIntake = this.voltsAppliedIntake;
        copy.brakePivot = this.brakePivot;
        return copy;
      }
}
