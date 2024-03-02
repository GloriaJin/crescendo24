package frc.robot.subsystems.Mecanum;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.pose.Gyro;
import frc.robot.sensor.pose.Odometry;
import frc.robot.sensor.vision.VisionAprilTag;

import frc.robot.utility.conversion.AngleUtil;
import frc.robot.utility.conversion.ObjectUtil;
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
  public PWMSparkMax leftFrontMotor;
  public PWMSparkMax leftRearMotor;
  public PWMSparkMax rightFrontMotor;
  public PWMSparkMax rightRearMotor;
/* 
  public WPI_VictorSPX leftFrontMotor; 
  public WPI_VictorSPX leftRearMotor;
  public WPI_VictorSPX rightFrontMotor;
  public WPI_VictorSPX rightRearMotor;*/

  private MecanumDrive driveTrain;
  private AHRS navx; 

  public Mecanum() {
    leftFrontMotor = new PWMSparkMax(Constants.k_chassis.leftFrontMotorPort);
    leftRearMotor = new PWMSparkMax(Constants.k_chassis.leftRearMotorPort);
    rightFrontMotor = new PWMSparkMax(Constants.k_chassis.rightFrontMotorPort);
    rightRearMotor = new PWMSparkMax(Constants.k_chassis.rightRearMotorPort);
    /*leftFrontMotor = new WPI_VictorSPX(Constants.k_chassis.leftFrontMotorPort);
    leftRearMotor = new WPI_VictorSPX(Constants.k_chassis.leftRearMotorPort);
    rightFrontMotor = new WPI_VictorSPX(Constants.k_chassis.rightFrontMotorPort);
    rightRearMotor = new WPI_VictorSPX(Constants.k_chassis.rightRearMotorPort);
    */
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



  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, Rotation2d navxAngle) {
    driveTrain.driveCartesian(ySpeed, xSpeed, zRotation, navxAngle);
  }

 

  /*public static void setMaxOutput(double maxSpeed) {
    Chassis.setMaxOutput(0.5);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  

}