// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class MotorEnableConstants {
//Motor Enable/Disable

  public static final boolean kIntakeLeftMotorEnabled = true;
  public static final boolean kIntakeRightMotorEnabled = true;
  public static final boolean kHopperMotorEnabled = true;
  public static final boolean kClimbMotorEnabled = true;
  public static final boolean kTopShooterLeftMotorEnabled = true;
  public static final boolean kBottomShooterLeftMotorEnabled = true;
  public static final boolean kTopShooterRightMotorEnabled = true;
  public static final boolean kBottomShooterRightMotorEnabled = true;
  public static final boolean kFloorMotorEnabled = true;
  public static final boolean kFrontKickupMotorEnabled = true;
  public static final boolean kRearKickupMotorEnabled = true;
  public static final boolean kHoodMotorEnabled = true;

  public static enum LogLevel {
    NONE,
    FULL
  }

  public static enum TelemetryLevel {
    NONE,
    LIMITED,
    FULL,
  }
  
}
