package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.FrontKickup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.RearKickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.MatchInfo;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Move {

    public Hopper hopper;
    public Intake intake;
    public Shooter shooter;
    public CommandSwerveDrivetrain drivetrain;
    public RearKickup rearKickup;
    public FrontKickup frontKickup;
    public Floor floor;
    public Hood hood;
    public SwerveRequest.FieldCentricFacingAngle driveFacingAngle;


    public Move(Hopper hopper, Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain, FrontKickup frontKickup, RearKickup rearKickup, Floor floor, Hood hood, SwerveRequest.FieldCentricFacingAngle driveFacingAngle) {
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.rearKickup = rearKickup;
        this.frontKickup = frontKickup;
        this.floor = floor;
        this.hood = hood;
        this.driveFacingAngle = driveFacingAngle;
    }

    //==========================================================
    //=====================Configuration========================
    //==========================================================

    //==========================================================
    //======================Private=============================
    //==========================================================
    private int getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return 0;
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red) {
                return 1;
            }
        }
        return 0;
    }

    //=============================Misc==========================================


    //==========================================================
    //=====================Commands=============================
    //==========================================================

    /**
     * Put all of the motors (except the drivetrain) into coast mode, so they can be easily moved into position.
     * @return A command that sets all of the motors into coast mode.
     */
    public Command coastAllMotors() {
        return Commands.parallel(
            hopper.setHopperCoast(),
            intake.setIntakeCoast(),
            shooter.setShooterCoast(),
            frontKickup.setFrontKickupCoast(),
            rearKickup.setRearKickupCoast(),
            floor.setFloorCoast()
        ).ignoringDisable(true).withName("coastAllMotors");
    }

    /**
     * Put all of the motors (except for the drivetrain) into their original neutral mode.
     * @return A command that sets all of the motors into their original neutral mode.
     */
    public Command resetAllMotorsNeutral() {
        return Commands. parallel(
            hopper.resetHopperMotorNeutral(),
            intake.resetIntakeMotorsNeutral(),
            shooter.resetShooterMotorsNeutral(),
            frontKickup.resetFrontKickupMotorNeutral(),
            rearKickup.resetRearKickupMotorNeutral(),
            floor.resetFloorMotorNeutral()           
        ).ignoringDisable(true).withName("resetAllMotorsNeutral");
    }

    /* Drive */
    public Command pathplannerAim() {
        return Commands.runOnce(
            () -> {PPHolonomicDriveController.overrideRotationFeedback(
                () -> {
                    Rotation2d perspectiveCorrection = drivetrain.getStateCopy().Pose.getRotation().rotateBy(MatchInfo.getInstance().getAlliancePerspective());
                    return driveFacingAngle.HeadingController.calculate(perspectiveCorrection.getRadians(), shooter.robotTarget().get().getRadians(), Utils.getCurrentTimeSeconds());
                });}
        ).withName("pathplannerAim");
    }

    public Command releasePathplannerAim() {
        return Commands.runOnce(() -> {PPHolonomicDriveController.clearRotationFeedbackOverride();}).withName("releasePathplannerAim");
    }

    //==============================Hood====================================

    public Command hoodUp() {
        return hood.hood30().withName("hoodUp");
    }

    public Command hoodDown() {
        return hood.hood0().withName("hoodDown");
    }

    //================================Hopper================================
    
    public Command hopperRetract() {  //Reviewed 2/21/26 should work now
        return Commands.deadline(hopper.hopperRetract(),intake.intakeSuck()).
        andThen(intake.intakeStop()).withName("hopperRetract");
    }

    public Command hopperExtend() {  //Reviewed 2/21/26 should work now
        //return Commands.parallel(hopper.hopperExtend(),intake.intakeStop());
        return hopper.hopperExtend().withName("hopperExtend");
    }

    public Command hopperMid() {
        return Commands.parallel(hopper.hopperMidPosition(),intake.intakeSuck()).withName("hopperMid");
    }

    public Command setHopperZeroPosition() {
        return hopper.setHopperZero().withName("setHopperZeroPosition");
    }

    public Command agitateHopper() {
        return hopper.agitate().alongWith(intake.intakeSuck()).withName("agitateHopper");
    }

    public Command zeroHopper() {
        return hopper.zeroHopper().withName("zeroHopper");
    }

    public Command slowHopperRetract() {
        return (Commands.parallel(hopper.slowRetract(),intake.intakeSuck()).withName("slowHopperRetract"));

    }

    //==============================Shoot========================================
    public Command stopShoot() {
        return Commands.sequence(Commands.parallel(floor.stopFloor(), rearKickup.stopRearKickup(),frontKickup.stopFrontKickup()), shooter.stopShoot(), hood.hood0()).withName("stopShoot");       
    }

    public Command startShootStatic() {
       return  Commands.sequence(hood.hood20(), shooter.startShootStatic()).andThen
            (Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup()), floor.forwardFloor()).withName("startShootStatic");   
    }

    public Command startAutoShoot() {
        return Commands.sequence(shooter.autoShoot(), hood.autoHood())
            .until(shooter.isShooterAtVelocity)
            .andThen(Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup(), floor.forwardFloor())).withName("startAutoShoot");
    }

    public Command startWhileMoveShoot() {   
        return Commands.parallel(Commands.repeatingSequence(shooter.whileMoveShoot(), hood.whileMoveHood()),
            Commands.waitUntil(shooter.isShooterAtVelocity)
                .andThen(Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup(), floor.forwardFloor()))).withName("startWhileMoveShoot");
    }

    public Command startDistanceBasedShot() {
        return Commands.parallel(
            Commands.repeatingSequence(shooter.whileMoveShoot(), hood.whileMoveHood()),
            Commands.waitUntil(shooter.isShooterAtVelocity).andThen(Commands.parallel(
                frontKickup.forwardFrontKickup(),
                rearKickup.forwardRearKickup(),
                floor.forwardFloor()
                )
            )
        ).withName("startDistanceBasedShot");
    }

    public Command hood30(){
        return hood.hood30().withName("hood30");
    }

    public Command hood0(){
        return hood.hood0().withName("hood0");
    }

    public Command setTargetToAllianceCornerRight() {
        return Commands.runOnce(() -> {MatchInfo.getInstance().setTargetAllianceCornerRight();}).withName("setTargetToAllianceCornerRight");
        // return shooter.setTargetToAllianceCornerRight();
    }

    public Command setTargetToAllianceCornerLeft() {
        return Commands.runOnce(() -> {MatchInfo.getInstance().setTargetAllianceCornerLeft();}).withName("setTargetToAllianceCornerLeft");
        // return shooter.setTargetToAllianceCornerLeft();
    }

    public Command setTargetToAllianceHub() {
        return Commands.runOnce(() -> {MatchInfo.getInstance().setTargetAllianceHub();}).withName("setTargetToAllianceHub");
        // return shooter.setTargetToAllianceHub();
    }

    //==============================Intake=======================================
    public Command reverseIntake() { 
        return Commands.sequence(hopper.hopperExtend(),Commands.parallel(intake.intakeSpit(),floor.reverseFloor())).withName("reverseIntake");
    }

    public Command intake() {
        return Commands.parallel(hopper.hopperExtend(),intake.intakeSuck()).withName("intake");
    }

    public Command stopIntake() {
        return intake.intakeStop().withName("stopIntake");
    }

    public Command nakedIntake() {
        return intake.intakeSuck().withName("nakedIntake");
    }

    //==============================Floor======================================
    public Command stopFloor() {
        return floor.stopFloor().withName("stopFloor");
    }

    //======================================================
    //========================Triggers======================
    //======================================================

}
