package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.limelight;
import frc.robot.constants.VisionConstants.photonvision;
import frc.robot.subsystems.vision.Photonvision;
import frc.robot.subsystems.vision.PhotonvisionSim;
import frc.robot.subsystems.vision.Camera.poseEstimate;

public class Vision extends SubsystemBase {
    
    @Logged
    private Photonvision leftCamera;
    @Logged
    private Photonvision rightCamera;

    private List<poseEstimate> poseEstimates;

    public CommandSwerveDrivetrain drivetrain;
    private Supplier<SwerveDriveState> swerveStateSupplier;
    public poseEstimateConsumer poseConsumer;

    private LimelightHelpers.PoseEstimate megaTag2 = new PoseEstimate();
    private LimelightHelpers.PoseEstimate megaTag = new PoseEstimate();

    public Field2d visionField = new Field2d();

    private VisionSystemSim visionSim;
    private SimCameraProperties cameraProperties = new SimCameraProperties();

    public Matrix<N3, N1> currentStdDevs = limelight.kMegaTag2StdDevs;
    
    private LimelightHelpers.PoseEstimate cachedMegaTag2 = new PoseEstimate();
    private LimelightHelpers.PoseEstimate cachedMegaTag = new PoseEstimate();
    private Pose2d testPose = new Pose2d(5.0, 5.0, new Rotation2d(90.0));
    private Pose2d leftPhotonPose = new Pose2d(0, 0, new Rotation2d(0.0));
    private Pose2d rightPhotonPose = new Pose2d(0, 0, new Rotation2d(0.0));
    private double cachedRobotHeading = 0.0;
    private double cachedRobotRotationRate = 0.0;
    private boolean cachedMegaTag2Valid = false;
    private boolean cachedMegaTagValid = false;
    private boolean cachedAreMegaTagsSeen = false;
    private boolean cachedAreMegaTag2Seen = false;
    private boolean cachedIsRobotSlowEnough = false;
    private boolean cachedIsMegaTagPoseValid = false;
    private boolean cachedIsMegaTag2PoseValid = false;
    private boolean cachedIsLeftPhotonPoseValid = false;
    private boolean cachedIsRightPhotonPoseValid = false;
    private double cachedLimelightHeartbeat = 0.0;
    private double testTimestamp;

    /* Logging Variables */
    @Logged(importance = Importance.CRITICAL)
    private String currentCommand = "";
    @Logged
    private Pose2d cachedMegaTagPose;
    @Logged
    private Pose2d cachedMegaTag2Pose;
    @Logged
    private Pose2d cachedLeftPhotonPose;
    @Logged
    private Pose2d cachedRightPhotonPose;

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

        this.setLimelightRobotPosition();
        //In the constructor, set the IMU mode to 1, so the limelight IMU is seeded with the robot gyro heading.
        this.setLimelightIMUMode(1);
        this.setLimelightPipeline(limelight.kCompPipelineIndex);    // Force the limelight to use the competition pipeline.
        LimelightHelpers.SetRobotOrientation(limelight.kName, this.getRobotHeading(), 0.0, 0.0, 0.0, 0.0, 0.0);

        leftCamera = new Photonvision(photonvision.kLeftName, photonvision.kTagLayout, photonvision.kRobotToLeftCamera);
        rightCamera = new Photonvision(photonvision.kRightName, photonvision.kTagLayout, photonvision.kRobotToRightCamera);

        // Force both photonvision cameras to use the competition pipeline.
        leftCamera.setPipeline(photonvision.kCompPipelineIndex);
        rightCamera.setPipeline(photonvision.kCompPipelineIndex);

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
     * Set the position of the Limelight relative to the center of the robot.
     * This only needs to be run during initialization.
     */
    private void setLimelightRobotPosition() {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelight.kName,
            limelight.kForwardOffset,
            limelight.kSideOffset,
            limelight.kUpOffset,
            limelight.kRollOffset,
            limelight.kPitchOffset,
            limelight.kYawOffset
        );
    }

    /**
     * Command the limelight to start using its internal IMU for the pose estimate it produces.
     */
    private void setLimelightIMUMode(int IMUMode) {
        /*
         * Mode 0 - External_Only - MegaTag2 uses the yaw sent from the robot to the Limelight.
         * Mode 1 - External_Seed - The Limelight gyro is seeded with the yaw sent from the robot.
         * Mode 2 - Internal_Only - Uses the Limelight gyro only.
         * Mode 3 - Internal_MT1_Assist - Corrects the Limelight gyro with MegaTag1 estimated yaw.
         * Mode 4 - Internal_External_Assist - Corrects the Limelight gyro with the robot yaw over time.  Recommended in the Limelight documentation.
         */
        LimelightHelpers.SetIMUMode(limelight.kName, IMUMode);
    }

    /**
     * Sets the current pipeline for the limelight.
     * @param pipelineIndex - The pipeline to switch to.
     */
    private void setLimelightPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelight.kName, pipelineIndex);
    }

    /**
     * Retrieve the limelight standard deviations from NetworkTables.
     * @return An array containing standard deviations for the megaTag and megaTag2 pose estimates.
     */
    private double[] getLimelightStdDevs() {
        return NetworkTableInstance.getDefault().getTable(limelight.kName).getEntry("stddevs").getDoubleArray(new double[12]);
    }

    /**
     * Separate the megaTag standard deviations from the whole standard deviation array and add them into a matrix.
     * @return A 3x1 matrix of standard deviations for the megaTag pose estimate.
     */
    private Matrix<N3, N1> getMegaTagStdDevs() {
        var stdDevs = this.getLimelightStdDevs();
        return VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[5]);
    }

    /**
     * Separate the megaTag2 standard deviations from the whole standard deviation array and add them into a matrix.
     * @return A 3x1 matrix of standard deviations for the megaTag2 pose estimate.
     */
    private Matrix<N3, N1> getMegaTag2StdDevs() {
        var stdDevs = this.getLimelightStdDevs();
        return VecBuilder.fill(stdDevs[6], stdDevs[7], stdDevs[11]);
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
     * Checks every target within a result and returns true if the highest measured ambiguity is less than the threshold passed into the method.
     * @param targets
     * @param ambiguityThreshold
     * @return
     */
    private boolean isResultAmbiguityBelowThreshold(List<PhotonTrackedTarget> targets, double ambiguityThreshold) {
        double highestAmbiguity = 0;
        for (var tgt : targets) {
            // For now, the ambiguity is -1 if the target is in a multi-pose estimate.
            // In that case, treat the target as 0 ambiguity.
            // A -1 would still work with the existing logic, but this is future-proofing.
            if (this.isResultAmbiguityInvalid(tgt)) {
                highestAmbiguity = 0;
            }
            else if (tgt.getPoseAmbiguity() >= highestAmbiguity) {
                highestAmbiguity = tgt.getPoseAmbiguity();
            }
        }
        return highestAmbiguity <= ambiguityThreshold;
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
     * Returns true if the latest megaTag estimate identifies at least the amount of tags passed into this method.
     * @param megaTagEstimate - A limelight pose estimate (megaTag or megaTag2)
     * @param tagCount - The amount of tags that the estimate must see for a valid estimate.
     * @return
     */
    private boolean areLimelightTagsSeen(LimelightHelpers.PoseEstimate megaTagEstimate, int tagCount) {
        return megaTagEstimate.tagCount >= tagCount;
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
     * Signifies that the latest estimated pose is valid if:
     * 1. The megaTag estimate is valid.
     * 2. At least one AprilTag was seen.
     * 3. The robot is not turning too fast.
     * @param megaTagEstimate - A limelight pose estimate (megaTag or megaTag2).
     * @return
     */
    private boolean isLimelightPoseValid(LimelightHelpers.PoseEstimate megaTagEstimate, int tagCount) {
        // 3.3 radians per second is currently 75% of our maximum rotational speed.
        return this.isMegaTagValid(megaTagEstimate) && this.areLimelightTagsSeen(megaTagEstimate, tagCount) && this.isRobotSlowEnough(VisionConstants.kMaximumRotationRate);
    }

    /**
     * Take a snapshot with the limelight.
     */
    private void takeLimelightSnapshot() {
        LimelightHelpers.triggerSnapshot(limelight.kName);
    }

    /**
     * Setup a limelight rewind capture.
     * @param duration - The amount of time, in seconds, to capture a recording for.  The maximum is 165 seconds.
     */
    private void takeLimelightVideo(double duration) {
        // This method is a want, not a need.
        LimelightHelpers.setRewindEnabled(limelight.kName, true);
        LimelightHelpers.triggerRewindCapture(limelight.kName, duration);
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
     * Return the pose component of the current megaTag estimate.
     * @return
     */
    private Pose2d getCurrentMegaTagPose() {
        return this.megaTag.pose;
    }

    /**
     * Return the pose component of the current megaTag2 estimate.
     * @return
     */
    private Pose2d getCurrentMegaTag2Pose() {
        return this.megaTag2.pose;
    }

    /**
     * Return the pose component of the current left swerve camera estimate.
     * @param camera
     * @return
     */
    private Pose2d getCurrentLeftPhotonPose() {
        return this.leftPhotonPose;
    }

    /**
     * Return the pose component of the current right swerve camera estimate.
     * @param camera
     * @return
     */
    private Pose2d getCurrentRightPhotonPose() {
        return this.rightPhotonPose;
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

    public Trigger addMegaTagPose = new Trigger(() -> {return this.cachedIsMegaTagPoseValid;});
    public Trigger addMegaTag2Pose = new Trigger(() -> {return this.cachedIsMegaTag2PoseValid;});
    public Trigger addLeftPhotonPose = new Trigger(() -> {return this.cachedIsLeftPhotonPoseValid;});
    public Trigger addRightPhotonPose = new Trigger(() -> {return this.cachedIsRightPhotonPoseValid;});

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
                this.takeLimelightVideo(duration.getAsDouble());
            }
        ).ignoringDisable(true).withName("limelightVideo");
    }

    public Command limelightAutonVideo() {return this.limelightVideo(() -> {return 20.0;}).withName("limelightAutonVideo");}
    public Command limelightTeleopVideo() {return this.limelightVideo(() -> {return 140.0;}).withName("limelightTeleopVideo");}

    public Command limelightSnapshot() {
        return runOnce(() -> {this.takeLimelightSnapshot();}).ignoringDisable(true).withName("limelightSnapshot");
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
        return runOnce(() -> {this.setLimelightIMUMode(IMUMode);}).ignoringDisable(true);
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
            this.setLimelightPipeline(limelightPipelineIndex);
            leftCamera.setPipeline(photonvisionPipelineIndex);
            rightCamera.setPipeline(photonvisionPipelineIndex);
        }).ignoringDisable(true).withName("setPipeline");
    }

    public Command setCompPipeline() {return this.setPipeline(limelight.kCompPipelineIndex, photonvision.kCompPipelineIndex).withName("setCompPipeline");}
    public Command setPracticePipeline() {return this.setPipeline(limelight.kPracticePipelineIndex, photonvision.kPracticePipelineIndex).withName("setPracticePipeline");}

    public Command setLimelightPosition() {
        return runOnce(() -> {
            this.setLimelightRobotPosition();
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
        this.limelightDisconnected.set(LimelightHelpers.getHeartbeat(limelight.kName) == this.cachedLimelightHeartbeat);
        this.photonLeftDisconnected.set(!this.leftCamera.isConnected());
        this.photonRightDisconnected.set(!this.rightCamera.isConnected());

        // Begin each loop with an empty list of pose estimates.
        this.poseEstimates = new LinkedList<poseEstimate>();
     
        // Start by caching important values.
        // By caching these values, any other code that requires them will use the same values for the current 20 ms loop.
        this.cachedRobotHeading = this.getRobotHeading();
        this.cachedRobotRotationRate = this.getRobotRotationRate();
        this.cachedIsRobotSlowEnough = this.isRobotSlowEnough(3.3);

        // Every loop, seed the limelight IMU with the current robot heading.
        LimelightHelpers.SetRobotOrientation(limelight.kName, this.cachedRobotHeading, 0.0, 0.0, 0.0, 0.0, 0.0);
        
        this.cachedMegaTag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.kName);
        this.cachedMegaTagValid = this.isMegaTagValid(this.cachedMegaTag);

        this.cachedMegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.kName);
        this.cachedMegaTag2Valid = this.isMegaTagValid(this.cachedMegaTag2);


        // Only check the number of tags and validity of the pose if the megatag is valid.
        // Only update the megaTag if the most recent megaTag is valid.
        if (this.cachedMegaTagValid) {
            this.cachedAreMegaTagsSeen = this.areLimelightTagsSeen(this.cachedMegaTag, limelight.kMegaTagCount);
            this.cachedIsMegaTagPoseValid = this.isLimelightPoseValid(this.cachedMegaTag, limelight.kMegaTagCount);
            this.megaTag = this.cachedMegaTag;
        } else {
            // If the megaTag isn't valid, obviously no tags can be seen and the pose isn't valid.
            this.cachedAreMegaTagsSeen = false;
            this.cachedIsMegaTagPoseValid = false;
        }

        // Only check the number of tags and validity of the pose if the megatag2 is valid.
        // Only update the megaTag2 if the most recent megaTag2 is valid.
        if (this.cachedMegaTag2Valid) {
            this.cachedAreMegaTag2Seen = this.areLimelightTagsSeen(this.cachedMegaTag2, limelight.kMegaTag2Count);
            this.cachedIsMegaTag2PoseValid = this.isLimelightPoseValid(this.cachedMegaTag2, limelight.kMegaTag2Count);
            this.megaTag2 = this.cachedMegaTag2;
        } else {
            // If the megaTag2 isn't valid, obviously no tags can be seen and the pose isn't valid.
            this.cachedAreMegaTag2Seen = false;
            this.cachedIsMegaTag2PoseValid = false;
        }

        if (cachedIsMegaTagPoseValid) {
            poseConsumer.accept(this.getCurrentMegaTagPose(), this.megaTag.timestampSeconds, limelight.kMegaTagStdDevs);
        }
        
        if (cachedIsMegaTag2PoseValid) {
            poseConsumer.accept(this.getCurrentMegaTag2Pose(), this.megaTag2.timestampSeconds, limelight.kMegaTag2StdDevs);
        }

        leftCamera.calculateEstimate(20);
        rightCamera.calculateEstimate(20);

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

        this.cachedLimelightHeartbeat = LimelightHelpers.getHeartbeat(limelight.kName);

        this.cachedMegaTagPose = this.getCurrentMegaTagPose();
        this.cachedMegaTag2Pose = this.getCurrentMegaTag2Pose();
        this.cachedLeftPhotonPose = this.getCurrentLeftPhotonPose();
        this.cachedRightPhotonPose = this.getCurrentRightPhotonPose();

        // Every loop, update the odometry with the current pose estimated by the limelight.
        switch (this.telemetryLevel) {
        case FULL:
            visionField.getObject("limelightMegaTagPose").setPose(this.getCurrentMegaTagPose());
            visionField.getObject("limelightMegaTag2Pose").setPose(this.getCurrentMegaTag2Pose());
            visionField.getObject("photonLeftPose").setPose(this.getCurrentLeftPhotonPose());
            visionField.getObject("photonRightPose").setPose(this.getCurrentRightPhotonPose());
        case LIMITED:
        case NONE:
            // No values!
        default:
            break;
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
