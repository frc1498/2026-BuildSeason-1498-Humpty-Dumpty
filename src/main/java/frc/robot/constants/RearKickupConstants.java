// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the rear kickup, including intake and outtake speeds.
 */
public class RearKickupConstants {
  /*===============Speeds===================*/
  public static final double kRearKickupIntake = 70.0;  
  public static final double kRearKickupOuttake = -12.0;  

  /* Deadbands */
  public static final double KRearKickupDeadband = 5.0;

  /* Safeties */
  public static final double kRearKickupMaxSpeed = 100.0;
  public static final double kRearKickupMinSpeed = -50.0;

  /* Duty Cycles */
  public static final double kRearKickupZeroDutyCycle = 0.0;
  
  /* Simulation Constants */

}
