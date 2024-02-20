package frc.robot.subsystems.Mecanum;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumModuleState;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.pose.Gyro;
import frc.robot.sensor.pose.Odometry;
import frc.robot.sensor.vision.VisionAprilTag;

import frc.robot.utility.conversion.AngleUtil;
import frc.robot.utility.conversion.ObjectUtil;
import frc.robot.utility.information.StageDetector;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Mecanum extends SubsystemBase {
  //Creates new Chassis
  public WPI_VictorSPX leftFrontMotor;
  public WPI_VictorSPX leftRearMotor;
  public WPI_VictorSPX rightFrontMotor;
  public WPI_VictorSPX rightRearMotor;
  private MecanumDrive driveTrain;
  private AHRS navx; 

  public ChassisSubsystem() {
    leftFrontMotor = new WPI_VictorSPX(Constants.k_chassis.leftFrontMotorPort);
    leftRearMotor = new WPI_VictorSPX(Constants.k_chassis.leftRearMotorPort);
    rightFrontMotor = new WPI_VictorSPX(Constants.k_chassis.rightFrontMotorPort);
    rightRearMotor = new WPI_VictorSPX(Constants.k_chassis.rightRearMotorPort);
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
    driveTrain = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    navx = new AHRS(SPI.Port.kMXP);
  }

  public void driveFieldOriented(XboxController controller){
    double forward = -controller.getRawAxis(1);
    double strafe = controller.getRawAxis(0);
    double rotation = controller.getRawAxis(4);

    double navxAngle = navx.getAngle(); 
    double radianAngle = Math.toRadians(navxAngle);
    double temp = forward * Math.cos(radianAngle) + strafe * Math.sin(radianAngle);
    strafe = -forward * Math.sin(radianAngle) + strafe * Math.cos(radianAngle);
    forward = temp; 

    driveTrain.driveCartesian(strafe, forward, rotation);
  }

  private double getnavxAngle(){
   
    return navx.getAngle();
  }

  private void resetnavxAngle(){
    navx.calibrate();
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, Rotation2d navxAngle) {
    driveTrain.driveCartesian(ySpeed, xSpeed, zRotation, navxAngle);
  }

  public Object driveCartesian(double leftX, double leftY, double rightX, ChassisSubsystem m_chassis) {
    return null;
  }

  /*public static void setMaxOutput(double maxSpeed) {
    Chassis.setMaxOutput(0.5);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  

  public void driveFieldOriented(double xSpeed, double ySpeed, double zRotation) {
  }


/*
 * -------------------------------------------------------------
 * -------------------------------------------------------------
 *                    LIMELIGHT STUFF !!!
 * -------------------------------------------------------------
 * -------------------------------------------------------------
 */
  
  //Calculating Classes
  public boolean isTargetFound() 
  {
      //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

      return tv != 0;
      //ouble tv = table.getEntry("tv").getDouble(0);
  }
  
  public long AprilTagFoundID() // returns ApirlTag ID after determining target has been found
  {
    long AprilTagID = 0;
    
    if(isTargetFound())
      {
        AprilTagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0); 
      }
    
    return AprilTagID;
  }
  
  public double Estimate_Distance() 
  {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

      double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0); //vertical offset

      double angleToGoalDegrees = Constants.EstimateDistanceConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
      
      //calculate distance
      double distanceFromLimelightToGoalInches = (Constants.EstimateDistanceConstants.goalHeightInchesAmp - Constants.EstimateDistanceConstants.limelightLensHeightInches)/Math.tan(angleToGoalRadians); //return
      return distanceFromLimelightToGoalInches;
  }
  
  public static double DistanceToTimeCalculation(double distance)
  {
    double time = distance/Constants.k_chassis.inPerSecSpeed; //edit constant
    return time;
  }

  public void findCorrectSpotX(double tx){
    if(tx > 0)
      {
        new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(-tx + Constants.AmpConstants.limelightOffsetFromRobotCenter))));
      }
      else if(tx < 0)
      {
        new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.AmpConstants.limelightOffsetFromRobotCenter));
      }
  }
  public void findCorrectSpotY(double time, double distanceError){
    if(distanceError > 0) //goes forwards
      {
        new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time); //(zRotation, ySpeed, xSpeed)
      }
      else if(distanceError < 0)//goes backwards
      {
        new RunCommand(() -> driveCartesian(0, Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time); //(zRotation, ySpeed, xSpeed)
      }
  }
 
  //Driving/Positional Classes

  //Amp on Blue Alliance --> ID 6
  //add code for angle of moving arm --> loading mech
  public void TargetAmpBlue()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.AmpConstants.desiredDistanceAmp;

    if(AprilTagID == 5)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //left AprilTagSpacing (negative value)
      new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.AmpConstants.AprilTagSpacing));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      
    }
  }

  //Amp on Red Alliance --> ID 5
  //add code for angle of moving arm --> loading mech
  public void TargetAmpRed()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.AmpConstants.desiredDistanceAmp;

    if(AprilTagID == 5)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //right AprilTagSpacing (positive value)
      new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.AmpConstants.AprilTagSpacing));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      
    }
  }

    public void TargetStageChain()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.StageConstants.desiredDistanceStage;

    if(AprilTagID == 11 || AprilTagID == 12 || AprilTagID == 13 || AprilTagID == 14 || AprilTagID == 15 || AprilTagID == 16)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //right AprilTagSpacing (positive value)
      new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.StageConstants.AprilTagSpacing));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      
    }
  }
  
}
  //input ApirlTag ID into desired distance variable
/*
  public void getOnChargeStation() //automatically drive to desired distance from target
  {
      AprilTagID = AprilTagFoundID();

      double distanceError = 0;
      double drivingAdjust = 0;
      double desiredDistance = Constants.ChargeStationConstants.desiredDistanceCS;
  
      if(AprilTagID == 6 || AprilTagID == 1)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          //left AprilTagSpacing (negative value)
          new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 7 || AprilTagID == 2)
      {
          double tx = table.getEntry("tx").getDouble(0);
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          double current_distance = Estimate_Distance();
          
          distanceError = desiredDistance - current_distance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 8 || AprilTagID == 3)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          //right AprilTagSpacing (positive value)
          new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError); 
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      }
  }
  
  

  public void ShoulderLineUpLeft()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      //go left
      new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ShoulderDriveConstants.nodeSpacingFromAprilTag));
      
      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }

  public void ShoulderLineUpCenter()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }

  public void ShoulderLineUpRight()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      //go right
      new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ShoulderDriveConstants.nodeSpacingFromAprilTag));
      
      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }

  
 */