package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * All constant values used for the vision co-processors.
 * Mainly co-processor names, physical coordinates of their positions on the robot, and camera settings.
 */
public class VisionConstants {

    public static class limelight {
        public static final String kName = "limelight";
        public static final double kForwardOffset = 0.3048;   // 12 inches (0.3048 meters) from the center of the robot.
        public static final double kSideOffset = -0.1047;     // 4-1/8 inches (-0.104775) from the center of the robot.
        public static final double kUpOffset = 0.2937;      // 11-9/16 inches (0.2936875 meters) from the bottom of the floor.
        public static final double kRollOffset = 0.0;          // degrees from vertical.
        public static final double kPitchOffset = 0.0;         // degrees from vertical.
        public static final double kYawOffset = 0.0;           // degrees from vertical.

        public static final Matrix<N3, N1> kMegaTag2StdDevs = VecBuilder.fill(0.5, 0.5, 9999999);   // The standard deviations suggested by Limelight.
    }

    public static class photonvision {
        public static final String kLeftName = "Left Swerve Camera";
        public static final Transform3d kRobotToLeftCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        public static final String kRightName = "Right Swerve Camera";
        public static final Transform3d kRobotToRightCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);  // FiM fields are welded.

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);  // Setting these to the same as the Limelight until they can be experimentally found.
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);
    }
}