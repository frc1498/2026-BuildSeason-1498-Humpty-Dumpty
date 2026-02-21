package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCalculation {
    private static ShotCalculation instance;

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
     * Calculate the virtual target based on the current speed.
     * @param robotSpeeds
     * @param timeOfFlight
     * @param targetPose
     * @return
     */
    public Pose2d getVirtualTarget(ChassisSpeeds robotSpeeds, Pose2d robotPose, double timeOfFlight, Pose2d targetPose) {
        double speedXComponent = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation()).vxMetersPerSecond;    // robotSpeeds.vxMetersPerSecond;
        double speedYComponent = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation()).vyMetersPerSecond;    // robotSpeeds.vyMetersPerSecond;
        
        double offsetX = speedXComponent * timeOfFlight;
        double offsetY = speedYComponent * timeOfFlight;
        return new Pose2d(targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY)), targetPose.getRotation());
    }

    
}
