package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
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


    default List<poseEstimate> getLatestEstimates() {
        return null;
    }

    default String getName() {
        return "";
    }

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

    @CustomLoggerFor(Camera.class)
    public class customCameraLogger extends ClassSpecificLogger<Camera> {
        public customCameraLogger() {
            super(Camera.class);
        }

        @Override
        public void update(EpilogueBackend backend, Camera camera) {
            backend.log("Camera", camera.getName());
            for (var est : camera.getLatestEstimates()) {
                backend.log("Pose", est.pose(), Pose3d.struct);
                backend.log("Timestamp", est.timestamp());
                backend.log("Ambiguity", est.ambiguity());
                backend.log("Average Tag Distance", est.averageTagDistance());
                backend.log("Tag Count", est.tagCount());
                backend.log("Type", est.type());
            }

        }
    }

    @FunctionalInterface
    public static interface poseEstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> measurementStdDevs);
    }
}
