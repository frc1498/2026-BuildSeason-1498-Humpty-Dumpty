package frc.robot.commands;

import java.util.function.Supplier;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShooterConstants;


public class Move {

    public Hopper hopper;
    public Climber climber;
    public Intake intake;
    public Shooter shooter;
    public CommandSwerveDrivetrain drivetrain;
    public Kickup kickup;
    public Spindexer spindexer;


    public Move(Climber climber, Hopper hopper, Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain, Kickup kickup, Spindexer spindexer) {
        this.climber = climber;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.kickup = kickup;
        this.spindexer = spindexer;
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

    //================================Hopper================================
    
    public Command hopperRetract() {  //Reviewed 2/21/26 should work now
        return Commands.deadline(hopper.hopperRetract(),intake.intakeSuck()).
        andThen(intake.intakeStop());
    }

    public Command hopperExtend() {  //Reviewed 2/21/26 should work now
        return Commands.parallel(hopper.hopperExtend(),intake.intakeStop());
    }

    public Command hopperMid() {
        return Commands.parallel(hopper.hopperMidPosition(),intake.intakeSuck());
    }

    public Command setHopperZeroPosition() {
        return hopper.setHopperZero();
    }

    public Command agitateHopper() {
        return hopper.agitate().alongWith(intake.intakeSuck());
    }

    //==================================Climb====================================
    
    public Command zeroClimb() {  
        return Commands.sequence(climber.zeroRoutine(),shooter.turret0());
    }

    public Command stopClimb() {
        return Commands.sequence(
            // climber.rotateClimbStop(),
            climber.liftClimbStop()
        );
    }
        
    public Command quickClimbRight() {//Drives to the right climb location
        return drivetrain.pathPlannerToPose(() -> {
            int allianceIndex = getAlliance();
            return ClimberConstants.quickClimbPoses[allianceIndex][1];
        });
    }
    
    public Command quickClimbLeft() {//Drives to the left climb location
        return drivetrain.pathPlannerToPose(() -> {
            int allianceIndex = getAlliance();
            return ClimberConstants.quickClimbPoses[allianceIndex][0];
        });
    }

    public Command climbExtend() {
        return Commands.sequence(shooter.stopShoot(),kickup.stopKickup(),spindexer.stopSpindexer(),shooter.turretCounterClockwise45(),climber.liftClimbExtend());
    }

    public Command climbRetract() {
        return (climber.liftClimbRetract().withTimeout(3)).andThen(climber.liftClimbStop());
    }

    //==============================Shoot========================================

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopShoot() {
        //return shooter.stopShoot();  
        return Commands.sequence(spindexer.stopSpindexer(),kickup.stopKickup(),shooter.stopShoot(), shooter.hood0(), shooter.turret0());       
    }

    /*
    public Command startShootFast() {
        //return shooter.startShootFast();  
        return Commands.sequence(shooter.hood30(),shooter.startShootFast()).andThen
                (Commands.sequence(kickup.forwardKickup(),
                spindexer.forwardSpindexer()));
    }
    */

    public Command startShootStatic() {
       return Commands.sequence(shooter.hood30(),shooter.turret0(), shooter.startShootStatic()).andThen
            (kickup.forwardKickup(), spindexer.forwardSpindexer());
    }

    public Command startAutoShoot() {
        return Commands.sequence(shooter.autoShoot(), shooter.autoHood(), shooter.autoTurret())
            .until(shooter.isShooterAtVelocity)
            .andThen(Commands.parallel(kickup.forwardKickup(), spindexer.forwardSpindexer()));
            //Commands.parallel(spindexer.forwardSpindexer(), hopper.agitate().alongWith(intake.intakeSuck())));
    }
    
    public Command startWhileMoveShoot() {
        
        return Commands.parallel(Commands.repeatingSequence(shooter.whileMoveShoot(), shooter.whileMoveHood(), shooter.whileMoveTurret()),
                Commands.waitUntil(shooter.isShooterAtVelocity).andThen(Commands.parallel(kickup.forwardKickup(), spindexer.forwardSpindexer())));

        /*
        return Commands.parallel(
            Commands.repeatingSequence(shooter.whileMoveShoot(), shooter.whileMoveHood(), shooter.whileMoveTurret()),
            Commands.waitUntil(shooter.isShooterAtVelocity).andThen(kickup.forwardKickup(), spindexer.forwardSpindexer(), hopper.agitate().alongWith(intake.intakeSuck()))
        );
        */
    }

    public Command turretClockWise45Degrees(){
        return shooter.turretClockwise45();
    }

    public Command turret0Degrees(){
        return shooter.turret0();
    }

    public Command turretCounterClockwise45Degrees() {
        return shooter.turretCounterClockwise45();
    }

    public Command hood30(){
        return shooter.hood30();
    }

    public Command hood0(){
        return shooter.hood0();
    }

    public Command setTargetToAllianceCornerRight() {
        return shooter.setTargetToAllianceCornerRight();
    }

    public Command setTargetToAllianceCornerLeft() {
        return shooter.setTargetToAllianceCornerLeft();
    }

    public Command setTargetToAllianceHub() {
        return shooter.setTargetToAllianceHub();
    }

    //==============================Intake=======================================
    public Command reverseIntake() { 
        return intake.intakeSpit();
    }

    public Command intake() {
        return Commands.parallel(hopper.hopperExtend(),intake.intakeSuck());
    }

    public Command stopIntake() {
        return Commands.parallel(intake.intakeStop());
    }

    //================================Misc=======================================
    public Command setDSAttachedLatchTrue() {
        return climber.setDSAttachedLatchTrue();
    }

    //======================================================
    //========================Triggers======================
    //======================================================


}