package frc.robot.commands;

import java.util.function.Supplier;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShooterConstants;


public class Move {

    public Hopper hopper;
    public Climber climber;
    public Intake intake;
    public Shooter shooter;
    public CommandSwerveDrivetrain drivetrain;

    public Move(Climber climber, Hopper hopper, Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        this.climber = climber;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
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
        return Commands.sequence(shooter.stopShoot(),shooter.stopKickup(),shooter.stopSpindexer(),shooter.turretCounterClockwise45(),climber.liftClimbExtend());
    }

    public Command climbRetract() {
        return (climber.liftClimbRetract().withTimeout(3)).andThen(climber.liftClimbStop());
    }

    //==============================Shoot========================================

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopShoot() {
        //return shooter.stopShoot();  
        return Commands.sequence(shooter.stopSpindexer(),shooter.stopKickup(),shooter.stopShoot(), 
        Commands.parallel(hopper.hopperExtend(), intake.intakeStop(),shooter.hood0()));       
    }

    public Command startShootFast() {
        //return shooter.startShootFast();  
        return Commands.sequence(shooter.hood30(),shooter.startShootFast()).andThen
                (Commands.sequence(shooter.forwardKickup(),
                shooter.forwardSpindexer()));
    }

    public Command startShootMedium() {
       return Commands.sequence(shooter.hood30(),
            shooter.startShootMedium()).andThen
            (shooter.forwardKickup(),
            shooter.forwardSpindexer()).andThen(
            hopper.agitate().alongWith(intake.intakeSuck()));
    }

    public Command startAutoShoot() {
        return Commands.sequence(shooter.autoShoot(), shooter.autoHood(), shooter.autoTurret())
            .until(shooter.isShooterAtVelocity)
            .andThen(shooter.forwardKickup(), Commands.parallel(shooter.forwardSpindexer(), hopper.agitate().alongWith(intake.intakeSuck())));
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

    //======================================================
    //========================Triggers======================
    //======================================================

}