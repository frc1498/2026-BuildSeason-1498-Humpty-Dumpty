package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.FrontKickup;
import frc.robot.subsystems.RearKickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Move {

    public Hopper hopper;
    public Intake intake;
    public Shooter shooter;
    public CommandSwerveDrivetrain drivetrain;
    public RearKickup rearKickup;
    public FrontKickup frontKickup;
    public Floor floor;


    public Move(Hopper hopper, Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain, FrontKickup frontKickup, RearKickup rearKickup, Floor floor) {
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.rearKickup = rearKickup;
        this.frontKickup = frontKickup;
        this.floor = floor;
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

    //==============================Hood====================================

    public Command hoodUp() {
        return shooter.hood30();
    }

    public Command hoodDown() {
        return shooter.hood0();
    }

    //================================Hopper================================
    
    public Command hopperRetract() {  //Reviewed 2/21/26 should work now
        /*
        return Commands.deadline(hopper.hopperRetract(),intake.intakeSuck()).
        andThen(intake.intakeStop());
        */
        return hopper.hopperRetract();
    }

    public Command hopperExtend() {  //Reviewed 2/21/26 should work now
        //return Commands.parallel(hopper.hopperExtend(),intake.intakeStop());
        return hopper.hopperExtend();
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

    //==============================Shoot========================================
    public Command stopShoot() {
        return Commands.sequence(floor.stopFloor(),Commands.parallel(rearKickup.stopRearKickup(),frontKickup.stopFrontKickup()), shooter.stopShoot(), shooter.hood0());       
    }

    public Command startShootStatic() {
       return  Commands.sequence(shooter.hood30(), shooter.startShootStatic()).andThen
            (Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup()), floor.forwardFloor());   
    }

    public Command startAutoShoot() {
        return Commands.sequence(shooter.autoShoot(), shooter.autoHood())
            .until(shooter.isShooterAtVelocity)
            .andThen(Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup(), floor.forwardFloor()));
            // Commands.parallel(spindexer.forwardSpindexer(), hopper.agitate().alongWith(intake.intakeSuck())));
    }

    public Command startWhileMoveShoot() {   
        return Commands.parallel(Commands.repeatingSequence(shooter.whileMoveShoot(), shooter.whileMoveHood()),
            Commands.waitUntil(shooter.isShooterAtVelocity)
                .andThen(Commands.parallel(frontKickup.forwardFrontKickup(), rearKickup.forwardRearKickup(), floor.forwardFloor())));
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
        return intake.intakeStop();
    }

    public Command nakedIntake() {
        return intake.intakeSuck();
    }

    //======================================================
    //========================Triggers======================
    //======================================================

}
