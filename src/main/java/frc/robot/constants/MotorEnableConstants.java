// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants used to turn the motors on and off for debugging purposes.
 * Also includes values for enabling and limiting the robot logging and telemetry.
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
