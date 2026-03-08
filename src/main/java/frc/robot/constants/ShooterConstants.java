// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class ShooterConstants {

  /*===============Speeds===================*/
  public static final double kKickupIntake = 90.0;  //Verified 2/25/26
  public static final double kKickupOuttake = -12.0;  //Verified 2/25/26
  public static final double kSpindexerIntake = 50.0;  
  public static final double kSpindexerOuttake = -10.0;  
    public static final double kSpindexerStoppedVelocityTolerance = 2;

  /* Deadbands */
  public static final double kHoodPositionDeadband = 0.5;  //In degrees - tentative
  public static final double kTurretPositionDeadband = 0.5;  //In degrees - tentative
  public static final double kShooterVelocityDeadband = 1;  //Changed from 2 to improve initial shot accuracy
  public static final double kKickupVelocityDeadband = 20.0;
  public static final double kSpindexerVelocityDeadband = 25;



  
  /*===============Safeties==================*/
  public static final double kHoodSafeExtend = 35.0;  //In degrees.  May be able to go slightly higher
  public static final double kHoodSafeRetract = 0.0;  //In degrees.  Zero is home
  
  public static final double kTurretSafeClockwise = -90.0;  //May be able to go further.  - is clockwise
  public static final double kTurretSafeCounterClockwise = 90.0;  //May be able to go further. + is counterclockwise
  public static final double kTurretOverturn = kTurretSafeCounterClockwise - 180.0; // If the counter clockwise limit is greater than 180.0, this is the degrees past 180.0 the turret can continue to move.
  // Set to 0 if the turret cannot move past 180.0 in the counterclockwise direction.

  public static final double kShooterMaxSpeed = 100.0;
  public static final double kShooterMinSpeed = -100.0;

  public static final double kKickupMaxSpeed = 500.0;
  public static final double kKickupMinSpeed = -50.0;
  
  //==============DutyCycles====================
  public static final double kTurretZeroDutyCycle = 0;
  public static final double kKickupZeroDutyCycle = 0;
  public static final double kSpindexerZeroDutyCycle = 0;
  public static final double kShooterZeroDutyCycle = 0;

  //==============Positions=================
  public static final double kTurretZeroPosition = 0;
  public static final double kTurretClimbPosition = 0;



  //==============Gear Ratios================
  public static final double kTurretMotorPinion=8; //Gear teeth
  public static final double kTurretDrivenGear1=48;
  public static final double kTurretPinion2=0.9;  //Pitch diameter of an 18 tooth gear
  public static final double kTurretGear=9.382;  //Pitch diameter
  public static final double kTurretGearRatio = (kTurretGear / kTurretPinion2) * (kTurretDrivenGear1 / kTurretMotorPinion);


  public static final double kHoodPinion = 8;
  public static final double kHoodDrivenGear1 = 42;
  public static final double kHoodPinion2 = 1.3;  //Pitch diameter of a 26 tooth gear
  public static final double kHoodGear = 8.75;  //Pitch diameter inches
  public static final double kHoodGearRatio = (kHoodGear / kHoodPinion2) * (kHoodDrivenGear1 / kHoodPinion);

  //=================Misc========================
  public static final int kTurretZeroCurrentLimit = 0;
  
  /* Simulation Constants */
  public static final double kShooterFlywheelGearing = 1.3; // 1 output : 1 input
  public static final double kHoodGearing = 5.0;
  public static final double kTurretGearing = 62.7;
  public static final double kSpindexerGearing = 3.0;
  public static final double kKickupGearing = 5.0;

  public static final Pose2d kRedHubCenter = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueHubCenter = new Pose2d(4.622, 4.028, Rotation2d.fromDegrees(180.0));

  public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(0.1143, -0.1381), new Rotation2d(0.0));
  // 0.138112776225552 m X from center of robot.
  // -0.114300228600457 m Y from centrer of robot.

  public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap(); // First column is distance (in meters), second column is angle (in degrees).
  public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap(); // First column is distance (in meters), second column is speed (in rotations per minute).
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();  // First column is distance (in meters), second column is time (in seconds).
                                                                                                      // Time of flight is the amount of time that the ball is in the air.

  // An enumeration of shooter faults to log with DogLog.
  public static enum ShooterFault {
    HOOD_SETPOINT_OUT_OF_RANGE,
    TURRET_SETPOINT_OUT_OF_RANGE,
    SHOOTER_SETPOINT_OUT_OF_RANGE,
    KICKUP_SETPOINT_OUT_OF_RANGE,
    SPINDEXER_SETPOINT_OUT_OF_RANGE
  }

  static {
    // In degrees.
    hoodAngleMap.put(1.7272, 10.0);
    hoodAngleMap.put(2.1336, 15.0);
    hoodAngleMap.put(2.8194, 20.0);
    hoodAngleMap.put(3.3528, 25.0);
    hoodAngleMap.put(3.7338, 30.0);
    hoodAngleMap.put(4.6990, 35.0);
    hoodAngleMap.put(5.4102, 35.0);

    // In RPS.
    flywheelSpeedMap.put(1.7272, 31.0);
    flywheelSpeedMap.put(2.1336, 33.0);
    flywheelSpeedMap.put(2.8194, 36.0);
    flywheelSpeedMap.put(3.3528, 39.5);
    flywheelSpeedMap.put(3.7338, 42.0);
    flywheelSpeedMap.put(4.6990, 45.5);
    flywheelSpeedMap.put(5.4102, 50.5);

    // In seconds.
    timeOfFlightMap.put(1.7272, 1.018);
    timeOfFlightMap.put(2.1336, 1.035);
    timeOfFlightMap.put(2.8194, 1.033);
    timeOfFlightMap.put(3.3528, 1.187);
    timeOfFlightMap.put(3.7338, 1.202);
    timeOfFlightMap.put(4.6990, 1.234);
    timeOfFlightMap.put(5.4102, 1.233);
  }
}
