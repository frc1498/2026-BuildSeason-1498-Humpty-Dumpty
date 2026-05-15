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

public class Photonvision implements Camera {

    private String name;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d cameraTransform;
    private List<poseEstimate> poseEstimates;
    private Alert cameraDisconnected;

    /**
    * Constructor for a Photonvision camera.
    * @param name The name of the camera.
    * @param fieldLayout The layout of the AprilTags on the game field.
    * @param cameraTransform The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public Photonvision(String name, AprilTagFieldLayout fieldLayout, Transform3d cameraTransform) {
        this.setName(name);
        this.fieldLayout = fieldLayout;
        this.setCameraTransform(cameraTransform);

        this.camera = new PhotonCamera(this.getName());
        this.poseEstimator = new PhotonPoseEstimator(this.fieldLayout, this.getCameraTransform());
        this.cameraDisconnected = new Alert(this.getName() + " Disconnected", AlertType.kError);
    }

    /**
    * Sets the name of the camera.
    * @param cameraName The name of the camera, as set on the co-processor.
    */
    public void setName(String cameraName) {
        this.name = cameraName;
    }

    /**
    * Return the name of this camera.
    * @return The name of the camera, used to query the co-processor.
    */
    public String getName() {return this.name;}

    /**
    * Sets the current vision pipeline for the camera.
    * The pipeline index begins at 0.
    * @param index The index of the pipeline on the co-processor to switch to.
    */
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    /**
    * Returns the index of the vision pipeline currently used by the camera.
    * The pipeline index begins at 0.
    * @return The index of the active vision pipeline.
    */
    public int getPipeline() {return camera.getPipelineIndex();}

    /**
    * Sets the 3d camera-to-robot transform used for pose estimation.
    * This is the postion of the camera on the robot.
    * The XYZ coordinates are relative to the center of the robot.
    * The yaw, pitch, and roll are relative to the camera lens.
    * @param cameraTransform The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public void setCameraTransform(Transform3d cameraTransform) {
        this.cameraTransform = cameraTransform;
    }

    /**
    * Returns the 3d camera-to-robot transform used by the camera for pose estimation.
    * @return The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public Transform3d getCameraTransform() {return this.cameraTransform;}

    /**
    * Check if the camera is connected.
    * If false, the camera is not sending any new data.
    * @return True if the camera is sending new data via NetworkTables.
    */
    public Boolean isConnected() {
        return camera.isConnected();
    }

    /**
    * Read all unread results from the camera and return them in a list.
    * Each call of PhotonCamera.getAllUnreadResults() is limited to the last 20 results.
    * The resultLimit parameter can be used to limit the size of the list further, to speed up pose estimation.
    * @param resultLimit The amount of results from the camera to include in the list.  Range is 1 - 20.
    * @return A list of PhotonPipelineResults from the camera.
    */
    private List<PhotonPipelineResult> getResults(int resultLimit) {
        // To-Do - Clamp the resultLimit to the range of 1 - 20.
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        return results.subList(0, Math.min(resultLimit, results.size()));
    }

    /**
    * Return a list of poseEstimates.
    * The list of pose estimates should be analyzed by the vision subsystem and either accepted or rejected into the overall pose estimate.
    * A poseEstimate is a custom record in the Camera interface - An attempt at a 'co-processor agnostic' type that can be handled in the vision subsystem in one way.
    * @return A list of pose estimates for the vision subsystem to accept or reject.
    */
    public List<poseEstimate> getLatestEstimates() {
        return this.poseEstimates;
    }

    /**
    * Calculate pose estimates based on the current results from the camera.
    * If a result is accepted, it will produce one pose estimate.
    * The pose estimates are not returned by this method.
    * Use this.getLatestEstimates() to return the estimates produced by this method.
    * @param resultLimit The number of results to calculate estimates for.
    */
    public void calculateEstimate(int resultLimit) {
        this.poseEstimates = new LinkedList<poseEstimate>();
        for (var result : this.getResults(resultLimit)) {
            // If any of these checks aren't true, don't bother using the result.
            // Perform intial checks on the result to determine if a potentially useful estimate can be calculated.
            if (
                !this.areTagsSeen(result, 1) ||    // Reject any result with less than # tags in view.
                !this.resultAmbiguityLimit(result, 0.1) ||    // Reject any result where the ambiguity of any tag is greater than #.
                !this.targetArea(result, 15.0)    // Reject any result where the 'best' tag in view is less than # percent of the total image.
            ) {
                continue;    // Skip the result if any of those conditions are met.
            }
            // Check if the result is a multitag result.
            if (result.multitagResult.isPresent()) {
                var est = this.poseEstimator.estimateCoprocMultiTagPose(result);
                // Redundancy check - Confirm that an estimate was created from the result.
                if (est.isPresent()) {
                    // Start filling out the pose estimate.
                    this.poseEstimates.add(new poseEstimate(
                        est.get().estimatedPose,    // Keep this as the 3d pose estimate.
                        est.get().timestampSeconds,
                        result.multitagResult.get().estimatedPose.ambiguity,    // The ambiguity of the pose estimate, technically different than tag ambiguity.
                        result.getTargets().get(0).bestCameraToTarget.getTranslation().getNorm(),    // For our purposes, the norm is like the distance from the camera to the 'best' tag.
                        result.getTargets().size(),
                        estimateType.PHOTONVISION_MULTITAG    // Caching the type of pose estimate, in case that factors into the trust of each type in the future.
                    )
                    );
                }
            }
            // If the result isn't a multitag result but wasn't rejected from the first pass, assume it's a single tag result.
            else {
                var est = this.poseEstimator.estimateLowestAmbiguityPose(result);
                // Redundancy check - Confirm that an estimate was created from the result.
                if (est.isPresent()) {
                    this.poseEstimates.add(new poseEstimate(
                        est.get().estimatedPose,    // Keep this as the 3d pose estimate.
                        est.get().timestampSeconds,
                        result.getBestTarget().poseAmbiguity,    // This should be a single tag result anyway, so the 'best' tag is the 'only' tag.
                        result.getBestTarget().bestCameraToTarget.getTranslation().getNorm(),    // For our purposes, the norm is like the distance from the camera to the 'best' tag.
                        1,    // Automatically assume this is a single tag result.
                        estimateType.PHOTONVISION_SINGLETAG
                    )
                    );
                }
            }
        }
    }

    /**
    * Check if a result contains at least a specified amount of tags.
    * This method also checks if the camera is connected.
    * @param result The PhotonPipelineResult to check.
    * @param tagLimit The minimum amount of tags a result should contain.
    * @return True, when the camera is connected and the amount of tags viewed in the result is greater than or equal to the tagLimit.
    */
    private boolean areTagsSeen(PhotonPipelineResult result, int tagLimit) {
        return this.isConnected() && result.hasTargets() && (result.getTargets().size() >= tagLimit);
    }

    /**
    * Check if the ambiguity of the 'best' tag in a result is below or equal to a threshold.
    * This method also checks if the camera is connected.
    * @param result The PhotonPipelineResult to check.
    * @param ambiguityThreshold The maximum allowable ambiguity.
    * @return True, when the camera is connected and the ambiguity of the 'best' tag is below or equal to the threshold.
    */
    private boolean resultAmbiguityLimit(PhotonPipelineResult result, double ambiguityThreshold) {
        return this.isConnected() && (result.getBestTarget().getPoseAmbiguity() <= ambiguityThreshold);
    }

    /**
    * Check if the 'best' tag in a result takes up a larger percentage of the image than the threshold.
    * This method also checks if the camera is connected.
    * @param result The PhotonPipelineResult to check.
    * @param tagSizeThreshold The minimum percentage of the image the 'best' tag should occupy.
    * @return True, when the camera is connected and the size of the 'best' tag in the result is larger than the threshold.
    */
    private boolean targetArea(PhotonPipelineResult result, double tagSizeThreshold) {
        return this.isConnected() && (result.getBestTarget().getArea() >= tagSizeThreshold);
    }
}
