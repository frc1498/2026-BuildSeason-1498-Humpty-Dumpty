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
    // Vision constants used between every camera.
    public static final double kMaximumRotationRate = 3.3;  // 3.3 radians per second.

    /**
     * Limelight related constants.
     */
    public static class limelight {
        public static final String kName = "limelight";
        public static final double kForwardOffset = 0.33655;   // 13 1/4 inches (0.33655 meters) from the center of the robot.
        public static final double kSideOffset = -0.05715;     // 2 1/4 inches (-0.05715) from the center of the robot. New mount shifts it 0.25 in.
        public static final double kUpOffset = 0.34925;      // 14 inches (0.3556 meters) from the bottom of the floor.
        public static final double kRollOffset = 0.0;          // degrees from vertical.
        public static final double kPitchOffset = 15.0;         // degrees from vertical. 22
        public static final double kYawOffset = 0.0;           // degrees from vertical.  Was zero
        public static final Transform3d kRobotToLimelight = new Transform3d(new Translation3d(0.3048, -0.1047, 0.2937), new Rotation3d(0,0,0));
        
        public static final int kCompPipelineIndex = 0;         // Treat index '0' as the competition index.
        public static final int kPracticePipelineIndex = 1;     // Treat index '1' as the practice index.

        public static final Matrix<N3, N1> kMegaTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);   // The standard deviations suggested by Limelight.
        public static final Matrix<N3, N1> kMegaTag2StdDevs = VecBuilder.fill(0.5, 0.5, 9999999);   // The standard deviations suggested by Limelight.
    }

    /**
     * Photonvision related constants.
     */
    public static class photonvision {
        public static final String kRightName = "swerveRightCamera";
        public static final Transform3d kRobotToRightCamera = new Transform3d(new Translation3d(0.2794, -0.31115, 0.1810), new Rotation3d(0.0, 0.2618, -1.5708));
        // 11 in. (0.279400558801118 in meters) X
        // 13 in. (-0.330200660401321 in meters) Y
        // 7-1/8 in. (0.180975361950724 in meters) Z
        // 1.5707963267949 rad (pi / 2) 90 deg
        // 15 deg roll (0.261799387799149 rad)
        // That is in the robot coordinate frame.  To the camera, that is a 15 deg positive pitch.
        public static final String kLeftName = "swerveLeftCamera";
        public static final Transform3d kRobotToLeftCamera = new Transform3d(new Translation3d(0.2794, 0.31115, 0.1810), new Rotation3d(0.0, 0.2618, 1.5708));
        // 11 in. (0.279400558801118 in meters) X
        // 12-1/8 in. (0.307975615951232 in meters) Y
        // 7-1/8 in. (0.180975361950724 in meters) Z
        // -1.5707963267949 rad (-pi / 2) -90 deg
        // 15 deg roll (0.261799387799149 rad)
        // That is in the robot coordinate frame.  To the camera, that is a 15 deg positive pitch.
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);  // FiM fields are welded.

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);  // Setting these to the same as the Limelight until they can be experimentally found.
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);

        public static final double kAmbiguityThreshold = 0.10;
        public static final int kCompPipelineIndex = 0;         // Treat index '0' as the competition index.
        public static final int kPracticePipelineIndex = 1;     // Treat index '1' as the pipeline index.

        public static enum Camera {
            SWERVE_RIGHT_CAMERA,
            SWERVE_LEFT_CAMERA
        }
    }
}