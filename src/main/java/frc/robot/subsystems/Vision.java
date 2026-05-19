package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.VisionConstants.limelight;
import frc.robot.constants.VisionConstants.photonvision;
import frc.robot.subsystems.vision.Photonvision;
import frc.robot.subsystems.vision.PhotonvisionSim;
import frc.robot.subsystems.vision.Camera.poseEstimate;
import frc.robot.subsystems.vision.Limelight;

public class Vision extends SubsystemBase {
    
    @Logged(importance = Importance.CRITICAL)
    private Photonvision leftCamera;
    @Logged(importance = Importance.CRITICAL)
    private Photonvision rightCamera;
    @Logged(importance = Importance.CRITICAL)
    private Limelight frontCamera;

    private List<poseEstimate> poseEstimates;

    public CommandSwerveDrivetrain drivetrain;
    private Supplier<SwerveDriveState> swerveStateSupplier;
    public poseEstimateConsumer poseConsumer;

    public Field2d visionField = new Field2d();

    private VisionSystemSim visionSim;
    private SimCameraProperties cameraProperties = new SimCameraProperties();

    public Matrix<N3, N1> currentStdDevs = limelight.kMegaTag2StdDevs;
    
    private Pose2d testPose = new Pose2d(5.0, 5.0, new Rotation2d(90.0));
    private double cachedRobotHeading = 0.0;
    private double cachedRobotRotationRate = 0.0;
    private boolean cachedIsRobotSlowEnough = false;
    private double testTimestamp;

    /* Logging Variables */
    @Logged(importance = Importance.CRITICAL)
    private String currentCommand = "";

    /* Subsystem Alerts */
    Alert limelightDisconnected = new Alert("Limelight Disconnected", AlertType.kError);
    Alert photonLeftDisconnected = new Alert("Photonvision Left Disconnected", AlertType.kError);
    Alert photonRightDisconnected = new Alert("Photonvision Right Disconnected", AlertType.kError);

    private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

    /**
     * Constructor.
     */
    public Vision(CommandSwerveDrivetrain drivetrain, Supplier<SwerveDriveState> swerveDriveState, poseEstimateConsumer poseConsumer, MotorEnableConstants.TelemetryLevel telemetryLevel) {
        this.drivetrain = drivetrain;
        this.swerveStateSupplier = swerveDriveState;
        
        this.poseConsumer = poseConsumer;
        //this.poseConsumer = this.drivetrain::addVisionMeasurement;

        this.telemetryLevel = telemetryLevel;

        leftCamera = new Photonvision(photonvision.kLeftName, photonvision.kTagLayout, photonvision.kRobotToLeftCamera);
        rightCamera = new Photonvision(photonvision.kRightName, photonvision.kTagLayout, photonvision.kRobotToRightCamera);
        frontCamera = new Limelight(limelight.kName, photonvision.kTagLayout, limelight.kRobotToLimelight);

        // Force all cameras to use the competition pipeline.
        leftCamera.setPipeline(photonvision.kCompPipelineIndex);
        rightCamera.setPipeline(photonvision.kCompPipelineIndex);
        frontCamera.setPipeline(limelight.kCompPipelineIndex);

        frontCamera.setOrientation(this.getRobotHeading());

        SmartDashboard.putData("Vision", this);
        SmartDashboard.putData("Vision/Pose", this.visionField);

        if (RobotBase.isSimulation()) {
            this.visionSim = new VisionSystemSim("vision");
            this.visionSim.addAprilTags(photonvision.kTagLayout);
            PhotonvisionSim leftCameraSim = new PhotonvisionSim(this.leftCamera);
            PhotonvisionSim rightCameraSim = new PhotonvisionSim(this.rightCamera);
            this.visionSim.addCamera(leftCameraSim.getCameraSim(), photonvision.kRobotToLeftCamera);
            this.visionSim.addCamera(rightCameraSim.getCameraSim(), photonvision.kRobotToRightCamera);
        }
    }

    /**
     * Returns true if the pose estimate is not 'null' and a valid target is in view.
     * 'Valid' in this case means the limelight actually sent data and sees a valid target; this method is not checking if the data makes sense.
     * @param poseEstimate - A limelight pose estimate (megaTag or megaTag2).
     * @return
     */
    private boolean isMegaTagValid(LimelightHelpers.PoseEstimate poseEstimate) {
        return (poseEstimate != null) && LimelightHelpers.getTV(limelight.kName);
    }

    /**
     * Check if the current pose ambiguity from the target is invalid.  A value of -1 is invalid.
     * @param target
     * @return True if the pose ambiguity is -1.
     */
    private boolean isResultAmbiguityInvalid(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity() == -1;
    }

    /**
     * Returns true if the average distance between visible targets and the robot is less than the distance passed into this method.
     * @param megaTagEstimate - The latest megaTag (or megaTag2) pose estimate.
     * @param distance - The maximum distance allowable between the robot and the apriltags.
     * @return True if the average tag distance is less than or equal to the distance.
     */
    private boolean isLimelightDistanceClose(LimelightHelpers.PoseEstimate megaTagEstimate, double distance) {
        return megaTagEstimate.avgTagDist <= distance;
    }

    /**
     * Returns true if the rotational velocity of the robot is less than the value passed into this method.
     * @param maximumRotationRate
     * @return
     */
    private boolean isRobotSlowEnough(double maximumRotationRate) {
        return this.cachedRobotRotationRate <= maximumRotationRate;
    }

    /**
     * Return the current robot heading, in degrees.
     * The current heading is based on the robot pose, because the pigeon yaw doesn't wrap around 0 - 360 degrees.
     */
    private double getRobotHeading() {
        return this.swerveStateSupplier.get().Pose.getRotation().getDegrees();
    }

    /**
     * Return the absolute angular velocity of the robot, in radians per second.
     * @return
     */
    private double getRobotRotationRate() {
        return Math.abs(this.swerveStateSupplier.get().Speeds.omegaRadiansPerSecond);
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic standard deviations based on number of tags, estimation strategy, and distance from the tags.
     * @param camera
     * @param estimatedPose The estimated pose to guess the standard deviations for.
     * @param targets All targets in this camera frame.
     */
    private void updateEstimationStdDevs(PhotonPoseEstimator camera, Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            this.currentStdDevs = photonvision.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = photonvision.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - See how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = camera.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                this.currentStdDevs = photonvision.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = photonvision.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs.times(1 + (avgDist * avgDist / 30));
                this.currentStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should only be used when there are targets visible.
     * @return
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return this.currentStdDevs;
    }

    /**
     * Returns a string of the name of the currently running command.
     * If no command is running, return "No Command".
     * @return
     */
    private String getCurrentCommandName() {
        return (this.getCurrentCommand() == null) ? "No Command" : this.getCurrentCommand().getName();
    }

    /**
     * Add the current test pose estimate to the drivetrain pose estimate.
     * @param drivetrain
     * @return
     */
    public Command addTestPose(Supplier<CommandSwerveDrivetrain> drivetrain) {
        return run(
            () -> {
                testTimestamp = Utils.getCurrentTimeSeconds();
                drivetrain.get().setVisionMeasurementStdDevs(limelight.kMegaTag2StdDevs);
                drivetrain.get().addVisionMeasurement(this.testPose, this.testTimestamp);
            }
        ).withName("Adding Test Pose Measurement").ignoringDisable(true);
    }

    private Command limelightVideo(DoubleSupplier duration) {
        return runOnce(
            () -> {
                this.frontCamera.takeVideo(duration.getAsDouble());
            }
        ).ignoringDisable(true).withName("limelightVideo");
    }

    public Command limelightAutonVideo() {return this.limelightVideo(() -> {return 20.0;}).withName("limelightAutonVideo");}
    public Command limelightTeleopVideo() {return this.limelightVideo(() -> {return 140.0;}).withName("limelightTeleopVideo");}

    public Command limelightSnapshot() {
        return runOnce(() -> {this.frontCamera.takeSnapshot();}).ignoringDisable(true).withName("limelightSnapshot");
    }

    private Command photonvisionSnapshot(Photonvision camera) {
        return runOnce(() -> {camera.takeSnapshot();}).ignoringDisable(true).withName("photonvisionSnapshot");
    }

    public Command leftPhotonSnapshot() {return this.photonvisionSnapshot(this.leftCamera).withName("leftPhotonSnapshot");};
    public Command rightPhotonSnapshot() {return this.photonvisionSnapshot(this.rightCamera).withName("rightPhotonSnapshot");};

    public Command allSnapshot() {
        return this.limelightSnapshot().andThen(leftPhotonSnapshot()).andThen(rightPhotonSnapshot()).withName("allSnapshot");
    }

    /**
     * Switch the limelight IMU mode  The method {@code setLimelightIMUMode()} has a description of each mode.
     * @param IMUMode - The IMU mode to switch the limelight to.
     * @return A command that changes the limelight IMU mode.
     */
    private Command switchIMUMode(int IMUMode) {
        return runOnce(() -> {this.frontCamera.setIMUMode(IMUMode);}).ignoringDisable(true);
    }

    public Command setLimelightIMUExternalOnly() {return this.switchIMUMode(0).withName("IMU Mode 0: External Only");}
    public Command setLimelightIMUExternalSeed() {return this.switchIMUMode(1).withName("IMU Mode 1: External Seed");}
    public Command setLimelightIMUInternalOnly() {return this.switchIMUMode(2).withName("IMU Mode 2: Internal Only");}
    public Command setLimelightIMUInternalMT1Assist() {return this.switchIMUMode(3).withName("IMU Mode 3: Internal MT1 Assist");}
    public Command setLimelightIMUInternalExternalAssist() {return this.switchIMUMode(4).withName("IMU Mode 4: Internal External Assist");}

    /**
     * Sets the vision pipelines of every camera on the robot.
     * Both photonvision cameras will run the same pipeline.
     * @param limelightPipelineIndex - The limelight pipeline to switch to.
     * @param photonvisionPipelineIndex - The photonvision pipeline to switch to.
     * @return - A command that sets the vision pipelines on the robot.
     */
    private Command setPipeline(int limelightPipelineIndex, int photonvisionPipelineIndex) {
        return runOnce(() -> {
            this.frontCamera.setPipeline(limelightPipelineIndex);
            this.leftCamera.setPipeline(photonvisionPipelineIndex);
            this.rightCamera.setPipeline(photonvisionPipelineIndex);
        }).ignoringDisable(true).withName("setPipeline");
    }

    public Command setCompPipeline() {return this.setPipeline(limelight.kCompPipelineIndex, photonvision.kCompPipelineIndex).withName("setCompPipeline");}
    public Command setPracticePipeline() {return this.setPipeline(limelight.kPracticePipelineIndex, photonvision.kPracticePipelineIndex).withName("setPracticePipeline");}

    public Command setLimelightPosition() {
        return runOnce(() -> {
            this.frontCamera.setCameraTransform(limelight.kRobotToLimelight);;
        }).ignoringDisable(true).withName("setLimelightPosition");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
        // That can be used to 'gate' values to log without lines of identical code.
        switch (this.telemetryLevel) {
        case FULL:
            builder.addDoubleProperty("Robot Heading", () -> {return this.cachedRobotHeading;}, null);
            builder.addDoubleProperty("Robot Rotation Rate", () -> {return this.cachedRobotRotationRate;}, null);
            builder.addBooleanProperty("Is Robot Slow Enough", () -> {return this.cachedIsRobotSlowEnough;}, null);
            builder.addDoubleProperty("Test Timestamp", () -> {return this.testTimestamp;}, null);
        case LIMITED:
            builder.addStringProperty("Command", this::getCurrentCommandName, null);
        case NONE:
            // No values!
        default:
            break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run.
        this.currentCommand = this.getCurrentCommandName();
        this.frontCamera.cameraDisconnected.set(!this.frontCamera.isConnected());
        this.leftCamera.cameraDisconnected.set(!this.leftCamera.isConnected());
        this.rightCamera.cameraDisconnected.set(!this.rightCamera.isConnected());

        // Begin each loop with an empty list of pose estimates.
        this.poseEstimates = new LinkedList<poseEstimate>();
     
        // Start by caching important values.
        // By caching these values, any other code that requires them will use the same values for the current 20 ms loop.
        this.cachedRobotHeading = this.getRobotHeading();
        this.cachedRobotRotationRate = this.getRobotRotationRate();
        this.cachedIsRobotSlowEnough = this.isRobotSlowEnough(3.3);

        // Every loop, seed the limelight IMU with the current robot heading.
        this.frontCamera.setOrientation(this.cachedRobotHeading);

        frontCamera.calculateEstimate();
        leftCamera.calculateEstimate(20);
        rightCamera.calculateEstimate(20);

        this.poseEstimates.addAll(frontCamera.getLatestEstimates());
        this.poseEstimates.addAll(leftCamera.getLatestEstimates());
        this.poseEstimates.addAll(rightCamera.getLatestEstimates());

        for(var est : this.poseEstimates) {
            // Rejection checks
            // Check for tag limit
            // check ambiguity
            // check Z height
            // check out of bounds
            // Need the zone classes for this check
            if ((est.tagCount() < photonvision.kEstimateTagCount) || (est.ambiguity() > photonvision.kAmbiguityThreshold) || (est.pose().getZ() > photonvision.kHeightLimit)) {
                continue;
            } else {
                double stddevFactor = Math.pow(est.averageTagDistance(), 2) / Math.pow(est.tagCount(), 2);
                poseConsumer.accept(est.pose().toPose2d(), est.timestamp(), photonvision.kMultiTagStdDevs.times(stddevFactor));
            }
        }
    }

    @Override
    public void simulationPeriodic() {

        this.visionSim.update(this.swerveStateSupplier.get().Pose);

        // This method will be called once per scheduler run during simulation.

        // Update the odometry to the test pose, for test purposes.
        // Add some noise to the test pose - although this is a annoying way to do it.
        // this.testPose = new Pose2d(5.0 + Math.random(), 5.0 + Math.random(), new Rotation2d(Math.random() * 180.0));
        // this.limelightField.setRobotPose(this.testPose);
        // this.testTimestamp = Utils.getCurrentTimeSeconds();

        // poseConsumer.accept(this.testPose, this.testTimestamp, limelight.kMegaTag2StdDevs);
    }

    @FunctionalInterface
    public static interface poseEstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> measurementStdDevs);
    }
}
