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

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command emptyHopper() {
        return Commands.sequence(
            shooter.slowShoot(),
            shooter.turretSlowShootPosition(),
            shooter.forwardSpindexer(), 
            shooter.forwardKickup()
        );
    }   

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopEmptyHopper () {
        return Commands.sequence(
            shooter.stopShoot(),
            shooter.stopSpindexer(),
            shooter.stopKickup()
        );
    }

    public Command hopperRetract() {
        return hopper.hopperRetract();
    }

    public Command hopperExtend() {
        return hopper.hopperExtend();
    }

    //==============================Spin and Kick================================

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.    
    public Command reverseSpinAndKick() {  //Need to make sure they are stopped first
        return Commands.sequence(
            shooter.reverseSpindexer(),
            shooter.reverseKickup()
        );
    }

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopSpinAndKick() {
        return Commands.sequence(
            shooter.stopSpindexer(),
            shooter.stopKickup()
        );
    }

    //==================================Climb====================================

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command primeClimb() {
        return Commands.parallel(hopper.hopperRetract(),intake.intakeStop()).andThen(
        Commands.sequence(
            climber.retractPins(),
            climber.rotateClimbExtend(),
            climber.liftClimbExtend()
        ).andThen(
            climber.extendPins()
        )
        );
    }
    
    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command homeClimb() {  
        return shooter.turretClimbPosition().andThen(
            Commands.sequence(
                climber.retractPins(),
                climber.rotateClimbHome(),
                climber.liftClimbHome()
            ).
        until(climber.isClimberHome));
    }

 public Command climbSequence() {
    //============================================Climb 1================================================
    return Commands.parallel(hopper.hopperRetract(),
        Commands.sequence(
            climber.liftClimbExtend(),
            climber.rotateClimbExtend()
        )).andThen(  //Verify hooks extended and hopper retracted
            climber.liftClimbHandoff()).andThen( //Lift pulls down to rotate hooks  
            climber.rotateClimbHandoff()).andThen( //Rotate hooks are engaged
    //============================================Climb 2================================================
        Commands.sequence(climber.liftClimbExtend(),climber.rotateClimbRetract())).andThen(  //Lift extends to climb ready while rotate retracts
        climber.liftClimbHandoff(),climber.rotateClimbExtend()).andThen( //Lift pulls down while rotate hooks return upward 
        climber.rotateClimbHandoff()).andThen( //Rotate hooks are engaged
    //============================================climb 3================================================
        Commands.sequence(climber.liftClimbExtend(),climber.rotateClimbRetract())).andThen(  //Lift extends to climb ready while rotate retracts
        climber.liftClimbRetract(),climber.rotateClimbExtend()); //Lift pulls down while rotate hooks return upward                 
    }

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopClimb() {
        return Commands.sequence(
            climber.rotateClimbStop(),
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

    //==============================Shoot========================================

    // Commands of the same subsystem cannot be run in parallel (resource conflict).
    // Switching it to a command sequence.
    public Command stopShoot() {
        return Commands.sequence(
            shooter.stopSpindexer(),
            shooter.stopKickup()
        ).andThen(
            shooter.stopShoot()
        );
    }

    public Command startShoot() {
        return shooter.startShoot();
    }

    //==============================Intake=======================================
    public Command stopOrReverseIntake() { 
        return Commands.parallel(hopper.hopperExtend(),intake.intakeSpit());
    }

    public Command intake() {
        return Commands.parallel(hopper.hopperExtend(),intake.intakeSuck());
    }

    //======================================================
    //========================Triggers======================
    //======================================================

}