package frc.robot.subsystems.vision;

import java.util.LinkedList;
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
    private List<poseEstimate> poseEstimates;
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

    private List<PhotonPipelineResult> getResults(int resultLimit) {
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        return results.subList(0, Math.min(resultLimit, results.size()));
    }

    public List<poseEstimate> getLatestEstimates() {
        return this.poseEstimates;
    }

    public void calculateEstimate(int resultLimit, PhotonPoseEstimator estimator) {
        this.poseEstimates = new LinkedList<poseEstimate>();
        for (var result : this.getResults(resultLimit)) {
            // If any of these checks aren't true, don't bother using the result.
            if (!this.areTagsSeen(result, 1) || !this.resultAmbiguityLimit(result, 0.1) || !this.targetArea(result, 15.0)) {
                continue;
            }
            if (result.multitagResult.isPresent()) {
                var est = estimator.estimateCoprocMultiTagPose(result);
                if (est.isPresent()) {
                    this.poseEstimates.add(new poseEstimate(
                        est.get().estimatedPose,
                        est.get().timestampSeconds,
                        result.multitagResult.get().estimatedPose.ambiguity,
                        result.getTargets().get(0).bestCameraToTarget.getTranslation().getNorm(),
                        result.getTargets().size(),
                        estimateType.PHOTONVISION_MULTITAG)
                    );
                }
            } else {
                var est = estimator.estimateLowestAmbiguityPose(result);
                if (est.isPresent()) {
                    this.poseEstimates.add(new poseEstimate(
                        est.get().estimatedPose,
                        est.get().timestampSeconds,
                        result.getBestTarget().poseAmbiguity,
                        result.getBestTarget().bestCameraToTarget.getTranslation().getNorm(),
                        1,
                        estimateType.PHOTONVISION_SINGLETAG)
                    );
                }
            }
        }
    }

    private boolean areTagsSeen(PhotonPipelineResult result, int tagLimit) {
        return this.isConnected() && result.hasTargets() && (result.getTargets().size() >= tagLimit);
    }

    private boolean resultAmbiguityLimit(PhotonPipelineResult result, double ambiguityThreshold) {
        return this.isConnected() && (result.getBestTarget().getPoseAmbiguity() <= ambiguityThreshold);
    }

    private boolean targetArea(PhotonPipelineResult result, double tagSizeThreshold) {
        return this.isConnected() && (result.getBestTarget().getArea() >= tagSizeThreshold);
    }

    private boolean targetSkew(PhotonPipelineResult result, double skewThreshold) {
        return this.isConnected() && (result.getBestTarget().getSkew() <= skewThreshold);
    }
}
