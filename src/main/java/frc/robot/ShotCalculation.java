package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.ShooterConstants;

public class ShotCalculation {
    private static ShotCalculation instance;

    private int convergeLimit = 20;
    private double phaseDelay = 0.03;
    private Pose2d poseEstimate;
    private double targetDistance;
    private Pose2d turretPoseEstimate;
    private double turretVelocityX;
    private double turretVelocityY;
    private ChassisSpeeds turretSpeedFieldRelative;
    private Pose2d virtualTarget;
    private double previewTargetDistance;
    private double offsetX;
    private double offsetY;
    private double timeOfFlight;

    /**
     * Return the instance of this class.
     * @return
     */
    public static ShotCalculation getInstance() {
        if (instance == null) {instance = new ShotCalculation();};
        return instance;
    }

    /**
     * Return the distance between the robot pose and the target pose.
     * @param robotPose
     * @param targetPose
     * @return
     */
    public double getTargetDistance(Pose2d robotPose, Pose2d targetPose) {
        return targetPose.getTranslation().getDistance(robotPose.getTranslation());
    }
    
    /**
     * Converts a robot relative chassis speed to a field relative chassis speed.
     * This is easy to write in-line, but this method is public.
     * @param robotRelativeSpeeds
     * @param robotAngle
     * @return
     */
    public ChassisSpeeds toFieldRelative(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotAngle);
    }

    /**
     * Return the current estimated virtual target.
     * @return
     */
    public Pose2d getVirtualTarget() {
        return this.virtualTarget;
    }

    /**
     * Calculate the virtual target based on the current speed.
     * @param robotSpeeds
     * @param timeOfFlight
     * @param targetPose
     * @return
     */
    public double getDistanceToVirtualTarget(ChassisSpeeds robotSpeeds, Pose2d robotPose, Pose2d targetPose) {

        // As much as I can tell, this is a 'prediction' so the mechanisms (turret, hood, flywheel) will track closer to what the target will be when this code is run.
        // Starting with 0.03, but I'm willing to move it to 0.02 (one loop iteration), or increase it as a fudge-factor.
        poseEstimate = robotPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * this.phaseDelay,
                robotSpeeds.vyMetersPerSecond * this.phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * this.phaseDelay
            )
        );

        turretPoseEstimate = robotPose.transformBy(ShooterConstants.kRobotToTurret);
        turretSpeedFieldRelative = this.toFieldRelative(robotSpeeds, poseEstimate.getRotation());

        // Determine the field relative velocity of the turret.  This is mostly equal to the robot velocity, with some adjustment based on the current rotational speed of the robot.
        // Part one is just the X (or Y) component of the field relative robot speed.
        // Part two - an application of the rotation matrix, broken out into the x and y components and multiplied by a constant (the magnitude of the rotational speed).
        // The X and Y terms from the rotation matrix are swapped, because which axis are conventionally regarded as X and Y are swapped in the robot relative frame.
        // At least, that's what I think.

        // I'm breaking this into multiple lines, just because it's a little more readable.
        turretVelocityX = turretSpeedFieldRelative.vxMetersPerSecond + 
            turretSpeedFieldRelative.omegaRadiansPerSecond * (
                ShooterConstants.kRobotToTurret.getY() * Math.cos(poseEstimate.getRotation().getRadians()) - 
                ShooterConstants.kRobotToTurret.getX() * Math.sin(poseEstimate.getRotation().getRadians())
            );

        turretVelocityY = turretSpeedFieldRelative.vyMetersPerSecond + 
            turretSpeedFieldRelative.omegaRadiansPerSecond * (
                ShooterConstants.kRobotToTurret.getY() * Math.sin(poseEstimate.getRotation().getRadians()) - 
                ShooterConstants.kRobotToTurret.getX() * Math.cos(poseEstimate.getRotation().getRadians())
            );
        
        // Initialize these variables before iterating.
        this.virtualTarget = targetPose;
        this.previewTargetDistance = this.getTargetDistance(turretPoseEstimate, virtualTarget);

        // Set the converge limit at the top of the class, to make it quick to change.
        for (int i = 0; i < convergeLimit; i++) {
            timeOfFlight = ShooterConstants.timeOfFlightMap.get(previewTargetDistance);

            this.offsetX = turretVelocityX * timeOfFlight;
            this.offsetY = turretVelocityY * timeOfFlight;

            this.virtualTarget = new Pose2d(targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY)),targetPose.getRotation());
            this.previewTargetDistance = this.getTargetDistance(turretPoseEstimate, virtualTarget);
        }

        return this.previewTargetDistance;
    }

    
}
