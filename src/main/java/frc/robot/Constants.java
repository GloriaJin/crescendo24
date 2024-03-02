// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //time it takes in seconds btwn each code loop
  public static final double loopTime = 0.02; 
  
  public final class k_chassis {
    //Chassis Motor ports
    public final static int leftFrontMotorPort = 2;
    public final static int rightFrontMotorPort = 5;
    public final static int rightRearMotorPort = 0;
    public final static int leftRearMotorPort = 1;
    // 5 2 1 0 is technically correct but i changed it so that it would drive correctrly 
    // 2 5 0 1
    //chassis speeds
    public final static double normalDriveSpeed = 0.6;
    public final static double slowDriveSpeed = 0.5;
    public final static double normalRotationSpeed = 0.5;
    public final static double slowRotationSpeed = 0.2;
    
    public final static double inPerSecSpeed = 1; //edit

    public final static double gyro = 0;


  }

   public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public final class FieldConstants {
    public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-0.086473, 5.757474);
    public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.389722, 5.757474);

    public static Translation2d getSpeaker() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? RED_SPEAKER_POSE
            : BLUE_SPEAKER_POSE;
      } else {
        return BLUE_SPEAKER_POSE; // default to blue
      }
    }
  }
  

  public static final class Mecanum{

    public static final double wheelRadius = 0.089; //TODO: wheel radius in meters

    public static final double wheelInset = 0.112; //TODO: how far back the wheel is set in from the frame
    
    public static final double frameX = 0.688; //TODO: replace with robot frame widt in meters
    public static final double frameY = 0.822; //TODO: robot frame length in meters
    
    public static final Translation2d[] modulePositions = new Translation2d[]{
      new Translation2d( ((frameX / 2) - wheelInset), ((frameY / 2) - wheelInset)),
      new Translation2d( ((frameX / 2 ) - wheelInset), -((frameY / 2) - wheelInset)),
      new Translation2d( -((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset)),
      new Translation2d(-((frameX / 2) - wheelInset), ((frameY / 2 - wheelInset)))
    };
    
    /** gear reduction  */
    public static final double driveGearRatio = 12.75; //TODO: gear reduction from angle motors to wheels

    /** conversion from Neo encoder units to degrees */
    public static final double relativeConversion = 360.0; 

    public static final class Constraints{//TODO: replace constraints 

      /** max free speed of robot in meters / second */
      public static final double maxFreeSpeed = 4.60248; 

      /** max accel of robot in meters / second */
      public static final double maxAcceleration = 2; 

      /** max rotational velocity in deg / second */
      public static final double maxRotationSpeed = 360; 

      /** max rotational accel in rotations / second */
      public static final double maxRotationalAcceleration = 1; 
    }
  }

  //wrong
  public final class k_xbox {
    //buttons --> correct
        public static final int buttonA = 1; 
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        public static final int buttonLeftBumper = 5;
        public static final int buttonRightBumper = 6;
        public static final int buttonLeftLowerBumper = 9;
        public static final int buttonRightLowerBumper = 10;
        public static final int buttonBack = 7;
        public static final int buttonStart = 8;
        //joysticks --> incorrect
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 4; //WHO PUT 2 FOR THIS??????? IT IS 4. ROTATION WASN'T WORKING BECAUSE THIS WAS 2 AND NOT 4
        public static final int rightYAxis = 3;
    }

    public static class IntakeConstants {

      public static final int intakeID = 9; //edit
      public static final double intakeP = 0.0;
      public static final double intakeD = 0.0;
      public static final double intakeI = 0.0;
  
      //public static final int start = 360; //previously start
      //public static final int open = 0; //previously open
  
      public static final double intakeSpeed = -0.2; //edit
      public static final double outputSpeed = 0.2; //edit
        // negative = counter-clockwise (?)
        // positive = clockwise (?)

      //public static final double intakeCloseSpeed = 0.1;
      //public static final double intakeOpenSpeed = -0.13;


      public static final int leftXAxis = 0; //edit
      public static final int leftYAxis = 1; //edit
  
    }
  
    public static class ArmConstants {
      public static final double loadingStop = 0.0; //loading angle --> edit
      public static final double shootingStop = 0.0; //shooting angle --> edit
      public static final double restingStop = 0.0; //resting angle --> edit

      public static final double speed = 0.2; //edit 
    }

//Estimate Distance
public static class EstimateDistanceConstants
{
    final public static double limelightMountAngleDegrees = -45.0; // how many degrees back is your limelight rotated from perfectly vertical?
    final public static double limelightLensHeightInches = 27.5; // distance from the center of the Limelight lens to the floor
    final public static double goalHeightInchesAmp = 48.5; // bottom of AprilTag height --> Amp
    final public static double goalHeightInchesLoading = 48.125; //bottom of AprilTag height --> Loading Station
    final public static double goalHeightInchesStage = 47.5; //bottom of AprilTag height --> Stage
}

//Getting in Range
public static class GettingInRangeConstants
{
    final public static double KpDistance = -0.1; // Proportional control constant for distance
}

//Amp 
public static class AmpConstants
{
    public static double AprilTagSpacing = 66.00; //in inches
    public static double desiredDistanceAmp = 132.25; //in inches --> edit 
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches --> edit 
}

//Stage
public static class StageConstants
{
    public static double desiredDistanceStage = 16.625; //inches
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches --> edit 
    
}

}