package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.LimelightHelpers;

public class Limelight implements Camera {

    private String name;
    private int imuMode;
    private double heartbeatCounter;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d cameraTransform;
    private List<poseEstimate> poseEstimates;
    public Alert cameraDisconnected;

    /**
    * Constructor for a Limelight camera.
    * @param name The name of the camera.
    * @param fieldLayout The layout of the AprilTags on the game field.
    * @param cameraTransform The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public Limelight(String name, AprilTagFieldLayout fieldLayout, Transform3d cameraTransform) {
        this.setName(name);
        this.fieldLayout = fieldLayout;
        this.setCameraTransform(cameraTransform);
        // In the constructor, set the IMU mode to 1, so the limelight IMU is seeded with the robot gyro heading.
        this.setIMUMode(1);

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
    @Override
    public String getName() {return this.name;}

    /**
    * Sets the current vision pipeline for the camera.
    * The pipeline index begins at 0.
    * @param index The index of the pipeline on the co-processor to switch to.
    */
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.getName(), index);
    }

    /**
    * Returns the index of the vision pipeline currently used by the camera.
    * The pipeline index begins at 0.
    * @return The index of the active vision pipeline.
    */
    public double getPipeline() {return LimelightHelpers.getCurrentPipelineIndex(this.getName());}

    /**
    * Sets the 3d camera-to-robot transform used for pose estimation.
    * This is the postion of the camera on the robot.
    * The XYZ coordinates are relative to the center of the robot.
    * The yaw, pitch, and roll are relative to the camera lens.
    * @param cameraTransform The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public void setCameraTransform(Transform3d cameraTransform) {
        this.cameraTransform = cameraTransform;
        LimelightHelpers.setCameraPose_RobotSpace(
            this.getName(),
            this.cameraTransform.getX(),
            this.cameraTransform.getY(),
            this.cameraTransform.getZ(),
            this.cameraTransform.getRotation().getX(),
            this.cameraTransform.getRotation().getY(),
            this.cameraTransform.getRotation().getZ()
        );
    }

    /**
    * Returns the 3d camera-to-robot transform used by the camera for pose estimation.
    * @return The coordinates of the camera (in 3d space) relative to the center of the robot.
    */
    public Transform3d getCameraTransform() {return this.cameraTransform;}

    /**
     * Command the limelight to start using its internal IMU for the pose estimate it produces.
     */
    public void setIMUMode(int IMUMode) {
        /*
         * Mode 0 - External_Only - MegaTag2 uses the yaw sent from the robot to the Limelight.
         * Mode 1 - External_Seed - The Limelight gyro is seeded with the yaw sent from the robot.
         * Mode 2 - Internal_Only - Uses the Limelight gyro only.
         * Mode 3 - Internal_MT1_Assist - Corrects the Limelight gyro with MegaTag1 estimated yaw.
         * Mode 4 - Internal_External_Assist - Corrects the Limelight gyro with the robot yaw over time.  Recommended in the Limelight documentation.
         */
        this.imuMode = IMUMode;
        LimelightHelpers.SetIMUMode(this.getName(), this.imuMode);
    }

    public int getIMUMode() {return this.imuMode;}

    public void setOrientation(double heading) {
        LimelightHelpers.SetRobotOrientation(this.getName(), heading, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Return the AprilTag field layout used by this camera.
     * @return
     */
    public AprilTagFieldLayout getFieldLayout() {return this.fieldLayout;}

    /**
    * Check if the camera is connected.
    * If false, the camera is not sending any new data.
    * @return True if the camera is sending new data via NetworkTables.
    */
    public Boolean isConnected() {
        boolean heartbeatStale = this.heartbeatCounter != LimelightHelpers.getHeartbeat(this.getName());
        this.heartbeatCounter = LimelightHelpers.getHeartbeat(this.getName());
        return heartbeatStale;
    }

    /**
     * Take a snapshot with the camera.
     * This takes a picture of the processed output.
     */
    public void takeSnapshot() {
        LimelightHelpers.triggerSnapshot(this.getName());
    }

    /**
     * Setup a limelight rewind capture.
     * @param duration - The amount of time, in seconds, to capture a recording for.  The maximum is 165 seconds.
     */
    public void takeVideo(double duration) {
        LimelightHelpers.setRewindEnabled(this.getName(), true);
        LimelightHelpers.triggerRewindCapture(this.getName(), Math.min(duration, 165.0));
    }

    /**
    * Read all unread results from the camera and return them in a list.
    * Each call of this method should have a megaTag1 result and a megaTag2 result.
    * @return A list of Limelight pose estimates.
    */
    private List<LimelightHelpers.PoseEstimate> getResults() {
        List<LimelightHelpers.PoseEstimate> results = new LinkedList<LimelightHelpers.PoseEstimate>();
        results.add(LimelightHelpers.getBotPoseEstimate_wpiBlue(this.getName()));
        results.add(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.getName()));
        return results;
    }

    /**
    * Return a list of poseEstimates.
    * The list of pose estimates should be analyzed by the vision subsystem and either accepted or rejected into the overall pose estimate.
    * A poseEstimate is a custom record in the Camera interface - An attempt at a 'co-processor agnostic' type that can be handled in the vision subsystem in one way.
    * @return A list of pose estimates for the vision subsystem to accept or reject.
    */
    @Override
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
    public void calculateEstimate() {
        this.poseEstimates = new LinkedList<poseEstimate>();
        for (var result : this.getResults()) {
            // If any of these checks aren't true, don't bother using the result.
            // Perform intial checks on the result to determine if a potentially useful estimate can be calculated.
            if (
                !this.areTagsSeen(result, 1) ||    // Reject any result with less than # tags in view.
                !this.estimateAmbiguityLimit(result, 0.1) ||    // Reject any result where the ambiguity of any tag is greater than #.
                !this.targetArea(result, 15.0)    // Reject any result where the 'best' tag in view is less than # percent of the total image.
            ) {
                continue;    // Skip the result if any of those conditions are met.
            }
            // Check if the result is a megaTag2 result.
            if (result.isMegaTag2) {
                if (true) {
                    // Start filling out the pose estimate.
                    this.poseEstimates.add(new poseEstimate(
                        new Pose3d(result.pose),    // Keep this as the 3d pose estimate.
                        result.timestampSeconds,
                        result.avgTagArea,    // The ambiguity of the pose estimate, technically different than tag ambiguity.
                        result.avgTagDist,    // For our purposes, the norm is like the distance from the camera to the 'best' tag.
                        result.tagCount,
                        estimateType.LIMELIGHT_MEGATAG_2    // Caching the type of pose estimate, in case that factors into the trust of each type in the future.
                    )
                    );
                }
            }
            // If the result isn't a multitag result but wasn't rejected from the first pass, assume it's a single tag result.
            else {
                // Redundancy check - Confirm that an estimate was created from the result.
                if (true) {
                    this.poseEstimates.add(new poseEstimate(
                        new Pose3d(result.pose),    // Keep this as the 3d pose estimate.
                        result.timestampSeconds,
                        result.avgTagArea,    // This should be a single tag result anyway, so the 'best' tag is the 'only' tag.
                        result.avgTagDist,    // For our purposes, the norm is like the distance from the camera to the 'best' tag.
                        1,    // Automatically assume this is a single tag result.
                        estimateType.LIMELIGHT_MEGATAG_1
                    )
                    );
                }
            }
        }
    }

    /**
    * Check if a megaTag estimate contains at least a specified amount of tags.
    * This method also checks if the camera is connected.
    * @param megaTagEstimate The Limelight megaTag estimate to check.
    * @param tagLimit The minimum amount of tags an estimate should contain.
    * @return True, when the camera is connected and the amount of tags viewed in the estimate is greater than or equal to the tagLimit.
    */
    private boolean areTagsSeen(LimelightHelpers.PoseEstimate megaTagEstimate, int tagLimit) {
        return this.isConnected() && (megaTagEstimate.tagCount >= tagLimit);
    }

    /**
    * Check if the highest tag ambiguity in the megaTag estimate is below or equal to a threshold.
    * This method also checks if the camera is connected.
    * @param megaTagEstimate The megaTagEstimate to check.
    * @param ambiguityThreshold The maximum allowable ambiguity.
    * @return True, when the camera is connected and the highest tag ambiguity in the estimate is below or equal to the threshold.
    */
    private boolean estimateAmbiguityLimit(LimelightHelpers.PoseEstimate megaTagEstimate, double ambiguityThreshold) {
        double highestAmbiguity = 0;
        for (var fiducial : megaTagEstimate.rawFiducials) {
            if (fiducial.ambiguity >= highestAmbiguity) {
                highestAmbiguity = fiducial.ambiguity;
            }
        }

        return this.isConnected() && (highestAmbiguity <= ambiguityThreshold);
    }

    /**
    * Check if the average tag area in the megaTag estimate takes up a larger percentage of the image than the threshold.
    * This method also checks if the camera is connected.
    * @param result The megaTagEstimate to check.
    * @param tagSizeThreshold The minimum percentage of the image the tags should occupy, on average.
    * @return True, when the camera is connected and the size of the average tag area in the result is larger than the threshold.
    */
    private boolean targetArea(LimelightHelpers.PoseEstimate megaTagEstimate, double tagSizeThreshold) {
        return this.isConnected() && (megaTagEstimate.avgTagArea >= (tagSizeThreshold / 100.0));
    }
}
