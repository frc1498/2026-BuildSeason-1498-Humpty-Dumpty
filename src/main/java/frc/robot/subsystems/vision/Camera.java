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

    @FunctionalInterface
    public static interface poseEstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> measurementStdDevs);
    }
}
