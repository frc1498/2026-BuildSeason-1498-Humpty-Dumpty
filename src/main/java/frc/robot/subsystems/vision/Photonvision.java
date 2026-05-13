package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants.photonvision;

public class Photonvision extends SubsystemBase implements Camera {

    private String name;
    private poseEstimateConsumer consumer;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d cameraTransform;
    private Alert cameraDisconnected;

    public Photonvision(String name, poseEstimateConsumer consumer, AprilTagFieldLayout fieldLayout, Transform3d cameraTransform) {
        this.setName(name);
        this.consumer = consumer;
        this.fieldLayout = fieldLayout;
        this.setCameraTransform(cameraTransform);

        this.camera = new PhotonCamera(this.getName());
        this.poseEstimator = new PhotonPoseEstimator(this.fieldLayout, this.getCameraTransform());
        this.cameraDisconnected = new Alert(this.getName() + " Disconnected", AlertType.kError);
    }

    public void setName(String cameraName) {
        this.name = cameraName;
    }

    public String getName() {return this.name;}

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public int getPipeline() {return camera.getPipelineIndex();}

    public void setCameraTransform(Transform3d cameraTransform) {
        this.cameraTransform = cameraTransform;
    }

    public Transform3d getCameraTransform() {return this.cameraTransform;}

    public Boolean isConnected() {
        return camera.isConnected();
    }


    private Optional<EstimatedRobotPose> processSingleResult(PhotonPipelineResult result, PhotonPoseEstimator estimator) {
        return estimator.estimateCoprocMultiTagPose(result);
    }

    private List<PhotonPipelineResult> getResults(int resultLimit) {
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        return results.subList(0, Math.min(resultLimit, results.size()));
    }

    private EstimatedRobotPose calculateEstimate(List<PhotonPipelineResult> results, PhotonPoseEstimator estimator) {
        for (var result : results) {
            result.
            if (result.multitagResult.isPresent()) {
                var wacky = this.processSingleResult(result, estimator);

            }
        }
    }

    private double distanceToTarget(PhotonTrackedTarget target) {
        return this.poseEstimator.getFieldTags().getTagPose(target.getFiducialId()).get().toPose2d().getTranslation().getDistance()
    }

    
    /**
     * Process the latest camera results from the photon camera.
     * We still need to determine the return type of this method, and how it passes the estimate back into the periodic method.
     * @param photonResults
     * @param photonEstimator
     */
    private EstimatedRobotPose processPhotonCameraResults(List<PhotonPipelineResult> photonResults, PhotonPoseEstimator photonEstimator, photonvision.Camera photonCamera) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : photonResults) {
            // Check if the pose is valid, and ignore everything if it isn't.
            if (this.isPhotonvisionResultValid(photonEstimator, result, photonvision.kEstimateTagCount)) {
                visionEst = this.processSingleResult(result, photonEstimator);
                if (visionEst.isEmpty()) {
                    // If there's no estimate from the MultiTagPose, do nothing.
                    // Leave the option to return to using the lowest estimate ambiguity.
                    //visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                }
                this.updateEstimationStdDevs(photonEstimator, visionEst, result.getTargets());

                // Only accept the vision estimate if there is one.
                visionEst.ifPresent( est -> {
                    var stddev = getEstimationStdDevs();
                    poseConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stddev);
  
                }
                });
            }
            // this.drivetrain.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds, this.getEstimationStdDevs());
        } 
    }
}
