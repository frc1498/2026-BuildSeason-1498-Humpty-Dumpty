// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the front kickup, including intake and outtake speeds.
 */
public class FrontKickupConstants {

  /* Speeds */
  public static final double kFrontKickupIntake = 90.0;  
  public static final double kFrontKickupOuttake = -12.0;  

  /* Deadbands */
  public static final double KFrontKickupDeadband = 5.0;

  /* Safeties */
  public static final double kFrontKickupMaxSpeed = 100.0;
  public static final double kFrontKickupMinSpeed = -50.0;

  /* Duty Cycles */
  public static final double kFrontKickupZeroDutyCycle = 0.0;
   
  /* Simulation Constants */
  public static final double kKickupGearing = 5.0;  //This is incorrect for Humpty Dumpty
}
