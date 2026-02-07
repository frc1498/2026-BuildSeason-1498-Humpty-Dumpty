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


public class Move {

    public Hopper hopper;
    public Climber climber;
    public Intake intake;
    public Shooter shooter;
    public CommandSwerveDrivetrain drivetrain;
    public Optional<Alliance> ally;

    public Move(Climber climber, Hopper hopper, Intake intake, Shooter shooter) {
        this.hopper = hopper;
        this.climber = climber;
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
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {
                return 0;
            }
            else if (ally.get() == Alliance.Red) {
                return 1;
            }
        }
        return 0;
    }

    //==========================================================
    //=====================Commands=============================
    //==========================================================

    public Command climbSequence() {
        return Commands.parallel(hopper.hopperRetract(),climber.liftClimbExtend()/*, climber.rotateClimb1Extend(), climber.rotateClimb2Extend()*/).andThen(  //Verify hooks extended and hopper retracted
            climber.liftClimbPastHandoff()).andThen( //Lift pulls down to rotate hooks  
            climber.rotateClimbEngaged()).andThen( //Rotate hooks are engaged
            Commands.parallel(climber.liftClimbExtend()/*,climber.rotateClimb1Retract(),climber.rotateClimb2Retract()*/)).andThen(  //Lift extends to climb ready while rotate retracts
            climber.liftClimbPastHandoff(),climber.rotateClimb1Extend(),climber.rotateClimb2Extend()).andThen( //Lift pulls down while rotate hooks return upward 
            climber.rotateClimbEngaged()).andThen( //Rotate hooks are engaged
            Commands.parallel(climber.liftClimbExtend()/*,climber.rotateClimb1Retract(),climber.rotateClimb2Retract()*/)).andThen(  //Lift extends to climb ready while rotate retracts
            climber.liftClimbPastHandoff(),climber.rotateClimb1Extend(),climber.rotateClimb2Extend()).andThen( //Lift pulls down while rotate hooks return upward             
            );
    }

    public Command stopClimb() {
        return Commands.parallel(climber.rotateClimb1Stop()/*,climber.rotateClimb2Stop(),climber.liftClimbStop()*/);
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
    

    //================================Hopper================================
    public Command emptyHopper() {
        return Commands.parallel(shooter.slowShoot()/*,shooter.turretSlowShootPosition()*/).andThen(
            shooter.forwardSpindexer(),shooter.forwardKickup());
    }    

    public Command stopEmptyHopper () {
        return Commands.parallel(shooter.stopShoot()/*, shooter.stopSpindexer(), shooter.stopKickup()*/);
    }

    public Command hopperRetract(){
        return hopper.hopperRetract();
    }

    public Command hopperExtend(){
        return hopper.hopperExtend();
    }

    //==============================Spin and Kick================================
    public Command reverseSpinAndKick() {  //Need to make sure they are stopped first
        return Commands.parallel(shooter.reverseSpindexer()/*,shooter.reverseKickup()*/);
    }

    public Command stopSpinAndKick(){
        return Commands.parallel(shooter.stopShoot()/*,shooter.stopSpindexer(),shooter.stopKickup()*/);
    }

    //==================================Climb====================================
    public Command primeClimb() {
        return Commands.parallel(hopper.hopperRetract(),intake.intakeStop()).andThen(
        Commands.parallel(climber.releasePins()/*, climber.rotateClimb1Extend(),climber.rotateClimb2Extend(),climber.liftClimbExtend()*/));
    }

    public Command homeClimb() {  
        return shooter.turretClimbPosition().andThen(
            Commands.parallel(climber.rotateClimb1Home()/*, climber.rotateClimb2Home(), climber.liftClimbHome()*/).
        until(climber.isClimberHome));
    }

    public Command stopShoot() {
        return Commands.parallel(shooter.stopSpindexer()/*, shooter.stopKickup()*/).andThen(
            shooter.stopShoot());
    }

    public Command startShoot() {
        return (shooter.startShoot());
    }

    public Command stopOrReverseIntake() { //Hopper needs to be extended first
        return (intake.intakeSpit());
    }

    public Command intake() { //Hopper needs to be extended first
        return (intake.intakeSuck());
    }

    //Shoot - this should act as a request to shoot
    //Stop Shooting?
    //Reverse Shooter, Kickup, Spindexer
    //Zero Hopper, Turret, Climber - should this be all together?
    //Intake while
    //Reverse Intake while
    //Level 3 climb ready onTrue this brings everything to position and releases the hooks
    //Level 3 climb while
    //Quick climb whileTrue we don't need to release the hooks.  This can just move the hoop up to max height, and when released it drops it back down to some height - nope, can't hold and drive
    //Home Climb onTrue  Problem - how do we safely home the fold up hooks on the rotate?  We can't hold them while homing.  Need to have pins we can put in the hooks
    //Homing needs to be a very low power!!!!! Can't use the normal home command.
    //Should hopper tract while shooting to help feed balls, and what would the trigger be.  Through beam for balls?

    //======================================================
    //========================Triggers======================
    //======================================================

}