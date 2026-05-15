package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Camera {
    // Fill out with methods shared by both Limelight and Photonvision.
    // isConnected
    // setPipeline
    // getPipeline
    // setCameraTransform
    // getCameraTransform
    // takeSnapshot
    // getPoseEstimate
    // setName
    // getName

    // A record with commonalities between camera pose estimates.
    // Use so a pose estimate, from a limelight or photonvision, is treated the same by the vision subsystem.
    public static record poseEstimate (
        Pose3d pose,
        double timestamp,
        double ambiguity,
        double averageTagDistance,
        int tagCount,
        estimateType type
    ) {}

    // Denotes the type of the pose estimate.
    // Useful if you want to trust one type of estimate more than the others, or apply stricter criteria.
    public static enum estimateType {
        LIMELIGHT_MEGATAG_1,
        LIMELIGHT_MEGATAG_2,
        PHOTONVISION_MULTITAG,
        PHOTONVISION_SINGLETAG
    }

    @FunctionalInterface
    public static interface poseEstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> measurementStdDevs);
    }
}
