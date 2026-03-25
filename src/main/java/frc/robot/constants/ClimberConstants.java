// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class ClimberConstants {
  //=====================Positions=====================
  public static final int kLiftClimbExtend = 45;
  public static final int kLiftClimbHandOff = 0;
  public static final int kLiftClimbRetract = 0;
  public static final int kLiftClimbHome = 0;

  public static final int kRotateClimbExtend = 0;
  public static final int kRotateClimbHandOff = 0;
  public static final int kRotateClimbRetract = 0;
  public static final int kRotateClimbHome = 0;
  public static final int kLiftClimbDeadband = 3;
  public static final int kRotateClimbDeadband = 0;

  //=====================Safeties======================
  public static final int kLiftClimbSafeExtend = 45;
  public static final int kLiftClimbSafeRetract = 0;

  public static final int kRotateClimbSafeExtend = 0;
  public static final int kRotateClimbSafeRetract = 0;

  //======================Sim Values===================

  //======================Poses=====================
  /* define a 2D array to hold positions for quick climb
   *        0           1
   * 0 [blue left][blue right]
   * 1 [red left][red right]
  */
  public static final Pose2d[][] quickClimbPoses = {
    //Blue
    {
      new Pose2d(1.512, 4.178, Rotation2d.fromDegrees(180)), //left
      new Pose2d(1.512, 3.310, Rotation2d.fromDegrees(180)) //right
    },
    //Red
    {
      new Pose2d(15.034, 3.882, new Rotation2d(0)), //left
      new Pose2d(15.034, 4.734, new Rotation2d(0)) //right
    }
  };
}
