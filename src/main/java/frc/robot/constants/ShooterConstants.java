// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants for the shooter, including speeds for the flywheel, positions for the hood, locations for the hub, and interpolation for the shooter as a function of distance.
 */
public class ShooterConstants {

  /*===============Speeds===================*/

  /* Deadbands */
  public static final double kHoodPositionDeadband = 0.5;  //In degrees - tentative
  public static final double kShooterVelocityDeadband = 1;  //Changed from 2 to improve initial shot accuracy

  /*===============Safeties==================*/
  public static final double kHoodSafeExtend = 45.5;  //In degrees.  May be able to go slightly higher
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

  public static final Pose2d kRedHubCenter = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueHubCenter = new Pose2d(4.622, 4.028, Rotation2d.fromDegrees(180.0));

  //Passing Constants
  public static final Pose2d kRedLeft = new Pose2d(14.3, 2.5, Rotation2d.fromDegrees(0));
  public static final Pose2d kRedRight = new Pose2d(14.3, 5.5, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueLeft = new Pose2d(2.2, 5.5, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kBlueRight = new Pose2d(2.2, 2.5, Rotation2d.fromDegrees(180.0));

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
    hoodAngleMap.put(2.1336, 16.0);
    hoodAngleMap.put(2.8194, 20.5);
    hoodAngleMap.put(3.3528, 26.0);
    hoodAngleMap.put(3.7338, 31.0);
    hoodAngleMap.put(4.6990, 35.0);
    hoodAngleMap.put(5.4102, 37.0);
    hoodAngleMap.put(7.62, 45.0); //For passing shot
    hoodAngleMap.put(16.0, 45.0); //For passing shot

    // In RPS.
    flywheelSpeedMap.put(1.7272, 40.0); // Was 31
    flywheelSpeedMap.put(2.1336, 42.0); // Was 33
    flywheelSpeedMap.put(2.8194, 45.5); // Was 36
    flywheelSpeedMap.put(3.3528, 48.5); // Was 39.5
    flywheelSpeedMap.put(3.7338, 49.0); // Was 42
    flywheelSpeedMap.put(4.6990, 54.0); // Was 45.5
    flywheelSpeedMap.put(5.4102, 59.0); // Was 50.5
    flywheelSpeedMap.put(7.62, 75.0); //For passing shot
    flywheelSpeedMap.put(16.0, 75.0); //For passing shot

    // In seconds.
    timeOfFlightMap.put(1.7272, 1.018);
    timeOfFlightMap.put(2.1336, 1.035);
    timeOfFlightMap.put(2.8194, 1.033);
    timeOfFlightMap.put(3.3528, 1.187);
    timeOfFlightMap.put(3.7338, 1.202);
    timeOfFlightMap.put(4.6990, 1.234);
    timeOfFlightMap.put(5.4102, 1.233);
    timeOfFlightMap.put(7.62, 1.735); //For passing shot
    timeOfFlightMap.put(16.0, 1.735); //For passing shot
  }
}
