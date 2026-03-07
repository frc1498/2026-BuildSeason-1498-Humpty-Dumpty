// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final double kShooterVelocityDeadband = 2;
  public static final double kKickupVelocityDeadband = 20.0;
  public static final double kSpindexerVelocityDeadband = 2;

  /*===============Safeties==================*/
  public static final double kHoodSafeExtend = 30.0;  //In degrees.  May be able to go slightly higher
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

  // None of these values are currently 'real'.  We need theoretical values for testing the algorithm.
  static {
    hoodAngleMap.put(0.5, 75.0);
    hoodAngleMap.put(1.0, 65.0);
    hoodAngleMap.put(1.5, 50.0);
    hoodAngleMap.put(2.0, 45.0);
    hoodAngleMap.put(2.5, 35.0);
    hoodAngleMap.put(3.0, 35.0);
    hoodAngleMap.put(3.5, 35.0);
    hoodAngleMap.put(4.0, 35.0);

    flywheelSpeedMap.put(0.5, 50.0);
    flywheelSpeedMap.put(1.0, 55.0);
    flywheelSpeedMap.put(2.0, 62.0);
    flywheelSpeedMap.put(2.5, 68.0);
    flywheelSpeedMap.put(3.0, 75.0);
    flywheelSpeedMap.put(3.5, 88.0);
    flywheelSpeedMap.put(4.0, 100.0);

    timeOfFlightMap.put(0.5, 0.6);
    timeOfFlightMap.put(1.0, 0.63);
    timeOfFlightMap.put(1.5, 0.71);
    timeOfFlightMap.put(2.0, 0.75);
    timeOfFlightMap.put(2.5, 0.78);
    timeOfFlightMap.put(3.0, 0.84);
    timeOfFlightMap.put(3.5, 0.89);
    timeOfFlightMap.put(4.0, 0.95);
  }
}
