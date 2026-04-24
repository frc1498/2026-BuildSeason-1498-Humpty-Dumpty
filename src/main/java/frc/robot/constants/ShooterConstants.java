// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants for the shooter, including speeds for the flywheel, positions for the hood, locations for the hub, and interpolation for the shooter as a function of distance.
 */
public class ShooterConstants {

  /*===============Speeds===================*/

  /* Deadbands */
  public static final double kHoodPositionDeadband = 0.5;  //In degrees - tentative
  public static final double kShooterVelocityDeadband = 0;  //Changed from 2 to improve initial shot accuracy

  /*===============Safeties==================*/
  public static final double kHoodSafeExtend = 55.0;  //In degrees.  May be able to go slightly higher
  public static final double kHoodSafeRetract = 0.0;  //In degrees.  Zero is home
  
  public static final double kShooterMaxSpeed = 100.0;
  public static final double kShooterMinSpeed = -100.0;

  //==============DutyCycles====================
  public static final double kShooterZeroDutyCycle = 0;

  //==============Gear Ratios================
  public static final double kHoodPinion = 8;
  public static final double kHoodDrivenGear1 = 42;
  public static final double kHoodPinion2 = 1.3;  //Pitch diameter of a 26 tooth gear
  public static final double kHoodGear = 8.75;  //Pitch diameter inches
  public static final double kHoodGearRatio = (kHoodGear / kHoodPinion2) * (kHoodDrivenGear1 / kHoodPinion);

  //=================Misc========================
  
  /* Simulation Constants */
  public static final double kShooterFlywheelGearing = 1.3; // 1 output : 1 input
  public static final double kHoodGearing = 5.0;

  public static final Pose2d kRedHubCenter = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0.0));
  public static final Pose2d kBlueHubCenter = new Pose2d(4.622, 4.028, Rotation2d.fromDegrees(180.0));

  //y - 4.028, adjusted for pi's field to 4.228

  //Passing Constants
  public static final Pose2d kRedLeft = new Pose2d(14.3, 2.5, Rotation2d.fromDegrees(0));
  public static final Pose2d kRedRight = new Pose2d(14.3, 5.5, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueLeft = new Pose2d(2.2, 5.5, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kBlueRight = new Pose2d(2.2, 2.5, Rotation2d.fromDegrees(180.0));

  public static final Transform2d kRobotToShooter = new Transform2d(new Translation2d(0.1143, -0.1381), Rotation2d.fromDegrees(0.0));
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
    hoodAngleMap.put(1.5748, 5.0);
    hoodAngleMap.put(2.1336, 15.0);
    hoodAngleMap.put(2.3368, 17.0);
    hoodAngleMap.put(3.1496, 25.0);
    hoodAngleMap.put(3.9370, 28.0);
    hoodAngleMap.put(4.9276, 33.0);
    hoodAngleMap.put(5.2324, 33.0);
    hoodAngleMap.put(5.5324, 34.5);
    hoodAngleMap.put(6.2738, 37.0); //For passing shot
    hoodAngleMap.put(9.4488, 37.0); //For passing shot
    hoodAngleMap.put(12.0, 55.0);  //For passing shot
    hoodAngleMap.put(15.0, 55.0); //For passing shot

    // In RPS.
    flywheelSpeedMap.put(1.5748, 33.0);
    flywheelSpeedMap.put(2.1336, 34.0);  //Was 34.8
    flywheelSpeedMap.put(2.3368, 36.8);  //Was 
    flywheelSpeedMap.put(3.1496, 39.0);  //Was 39.5
    flywheelSpeedMap.put(3.9370, 42.5);  //Was 43
    flywheelSpeedMap.put(4.9276, 48.0);  //Was
    flywheelSpeedMap.put(5.2324, 49.0);  //Was
    flywheelSpeedMap.put(5.5324, 49.5);  //Was
    flywheelSpeedMap.put(6.2738, 52.0); //For passing shot
    flywheelSpeedMap.put(9.4488, 60.0); //For passing shot
    flywheelSpeedMap.put(12.0, 75.0); //For passing shot
    flywheelSpeedMap.put(15.0, 85.0); //For passing shot

    // In seconds.
    timeOfFlightMap.put(1.5748, 1.063);
    timeOfFlightMap.put(2.1336, 1.063);
    timeOfFlightMap.put(2.3368, 1.064);
    timeOfFlightMap.put(3.1496, 1.067);
    timeOfFlightMap.put(3.9370, 1.105);
    timeOfFlightMap.put(4.9276, 1.267);
    timeOfFlightMap.put(5.2324, 1.268);
    timeOfFlightMap.put(5.5324, 1.275);
    timeOfFlightMap.put(6.2738, 1.500); //For passing shot
    timeOfFlightMap.put(9.4488, 1.700); //For passing shot
    timeOfFlightMap.put(12.0, 1.861);  //For passing shot
    timeOfFlightMap.put(15.0, 2.3); //For passing shot
  }
}
