// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.config.ClimberConfig;
import frc.robot.config.HopperConfig;
import frc.robot.config.IntakeConfig;
import frc.robot.config.ShooterConfig;
import frc.robot.commands.Move;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    //=======================================================================
    //=======================Assign Subsystem Names==========================
    //======================================================================= 
    public final ClimberConfig climberConfig = new ClimberConfig();
    public Climber climber = new Climber(climberConfig);

    public HopperConfig hopperConfig = new HopperConfig();
    public Hopper hopper = new Hopper(hopperConfig);

    public IntakeConfig intakeConfig = new IntakeConfig();
    public Intake intake = new Intake(intakeConfig);

    public File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    public Selector autonSelect = new Selector(autonFolder, ".auto", "Auton Selector");
    public Command selectedAuton;
    public ArrayList<Command> autonCommands = new ArrayList<Command>();

    //Gamepad assignment
    //Instantiate 
    private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    //private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorControllerPort);
    // private final CommandXboxController developer = new CommandXboxController(ControllerConstants.kDeveloperControllerPort);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Vision vision = new Vision(drivetrain, drivetrain::getStateCopy, drivetrain::addVisionMeasurement);

    public ShooterConfig shooterConfig = new ShooterConfig();
    public Shooter shooter = new Shooter(shooterConfig, drivetrain::getStateCopy);

    public final Move move = new Move(climber,hopper,intake,shooter,drivetrain);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Create DogLog
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        //DogLog.setPdh(new PowerDistribution());     // allows battery and pdp logging
        DogLog.log("ExampleLog", "Hello world!");   // test log item
        // Configure the trigger bindings
        configureBindings();
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //shooter.setDefaultCommand(shooter.setShooterOutputs());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        this.DSAttached.onTrue(autonSelect.filterList(() -> {return DriverStation.getAlliance().get().toString();})
            .andThen(() -> {this.autonCommands = this.loadAllAutonomous(autonSelect.currentList());}).ignoringDisable(true)
            .andThen(() -> {this.selectedAuton = autonCommands.get(autonSelect.currentIndex().get());}).ignoringDisable(true));

        // Add the limelight pose estimate to the drivetrain estimate.
        //vision.addLimelightPose.whileTrue(vision.addMegaTag2(() -> {return drivetrain;}));
      
        //===================================================
        //===================Driver Commands=================
        //===================================================

        //Driver POV Up - Climb Hold (quick or otherwise, context sensitive)
        /*
        driver.povUp().and(RobotModeTriggers.disabled()).onTrue(autonSelect.increment().andThen(() -> {this.selectedAuton = this.autonCommands.get(this.autonSelect.currentIndex().get());}).ignoringDisable(true));
        driver.povUp().and(climber.isClimberReadyToClimb).onTrue(move.climbSequence());
        */

        //Driver POV Down - Climb Stop
        /*driver.povDown().and(RobotModeTriggers.disabled()).onTrue(autonSelect.decrement().andThen(() -> {this.selectedAuton = this.autonCommands.get(this.autonSelect.currentIndex().get());}).ignoringDisable(true));
        driver.povDown().onTrue(move.stopClimb());
        */

        // Running this as 'whileTrue' because otherwise the default command of the tracking shot will take over (I think).
        //driver.povLeft().whileTrue(shooter.setTuningShooterOutputs());

        //Driver POV Left: Autodrive Quick Climb Left
        //driver.povLeft().onTrue(move.quickClimbLeft());

        //Driver POV Right: Autodrive Quick Climb Right
        //driver.povRight().onTrue(move.quickClimbRight());

        //Driver X button: Empty Hopper / Slow shot / Hopper In
        //driver.x().whileTrue(move.emptyHopper());

        //Driver Y button: Reverse spindexer and kickup
        /*
        driver.b().whileTrue(move.stopSpinAndKick().andThen(move.reverseSpinAndKick())).onFalse(
            move.stopSpinAndKick());
        */

        //Driver A button: Prime Climb - pulls hopper in and goes to slow shoot and readies hooks
        //driver.a().onTrue(move.primeClimb());

        //Driver B button: Hopper In/Out
        //driver.y().whileTrue(move.hopperRetract()).onFalse(move.hopperExtend());

        //Driver RTrigger: Shoot off
        //driver.rightTrigger(0.1).onTrue(move.stopShoot());



        //Driver Select: Zero drivetrain
        //driver.start().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()));

        //Driver Start: Home the Climb System (low current, will break hooks!)
        //driver.start().whileTrue(move.homeClimb());

        //======================Current Driving Commands=======================
        //Driver RBumper: Shoot  
        //driver.rightBumper().onTrue(move.startShootMedium()).OnFalse(move.stopShoot());

        //Driver LTrigger: Intake Reverse - Check 2/26/26 ready for testing
        //driver.leftTrigger(0.1).whileTrue(move.reverseIntake()).onFalse(move.stopIntake());

        //Driver LBumper Intake on  -Checked 2/26/26 ready for testing
        //driver.leftBumper().onTrue(move.intake()).onFalse(move.stopIntake());

        driver.rightBumper().whileTrue(move.startShootMedium()).onFalse(move.stopShoot());

        driver.leftBumper().onTrue(move.intake()).onFalse(move.stopIntake());
        driver.leftTrigger(0.1).whileTrue(move.reverseIntake()).onFalse(move.stopIntake());

        driver.a().onTrue(move.hopperMid()).onFalse(move.hopperExtend());

        //===================================================
        //==================Operator Commands================
        //===================================================

        //Operator POV Up
        //operator.povUp()

        //Operator POV Down
        //operator.povDown()

        //Operator POV Left
        //operator.povLeft()

        //Operator POV Right
        //operator.povRight()

        //Operator X button
        //operator.X()

        //Operator Y button
        //operator.y()

        //Operator A button
        //operator.a()

        //Operator B button
        //operator.b()

        //Operator RTrigger
        //operator.rightTrigger(0.1)

        //Operator RBumper 
        //operator.rightBumper()

        //Operator LTrigger
        //operator.leftTrigger(0.1)

        //Operator LBumper
        //operator.leftBumper()

        //Operator Select
        //operator.

        //Operator Start
        //operator.START().

        //===================================================
        //==================Developer Commands===============
        //===================================================
        //These are tested and work
        //developer.start().onTrue(hopper.hopperExtend());
        //developer.back().onTrue(hopper.hopperRetract());

        //developer.x().onTrue(move.turretCounterClockwise45Degrees());
        //developer.y().onTrue(move.turret0Degrees());
        //developer.b().onTrue(move.turretClockWise45Degrees());

        //developer.y().onTrue(move.hood30());
        //developer.a().onTrue(move.hood0());

        //developer.y().onTrue(move.startShootFast());
        //developer.x().onTrue(move.startShootMedium());
        //developer.a().onTrue(move.stopShoot());

        //developer.rightBumper().whileTrue(move.startShootMedium()).onFalse(move.stopShoot());

        //developer.leftBumper().onTrue(move.intake()).onFalse(move.stopIntake());
        //developer.leftTrigger(0.1).whileTrue(move.reverseIntake()).onFalse(move.stopIntake());

        //Working on these
        //developer.y().onTrue(move.climbExtend());
        //developer.a().onTrue(move.climbRetract());

        //developer.x().onTrue(move.primeClimb());

        //developer.b().onTrue(climber.zeroRoutine());  //Which button is the back button!!!!

        // developer.leftBumper().and(hopper.isHopperExtended).whileTrue(intake.intakeSuck()).onFalse(intake.intakeStop());
        // developer.rightBumper().and(hopper.isHopperExtended).whileTrue(intake.intakeSpit()).onFalse(intake.intakeStop());
        // hopper.isHopperExtended.whileFalse(intake.intakeStop());
        // Running this as 'whileTrue' because otherwise the default command of the tracking shot will take over (I think).
        //developer.povUp().toggleOnTrue(shooter.setTuningShooterOutputs());

        // A test of the sysID functionality.
        //developer.back().and(developer.a()).whileTrue(shooter.sysIdKickupDynamic(Direction.kForward));
        //developer.back().and(developer.b()).whileTrue(shooter.sysIdKickupDynamic(Direction.kReverse));
        //developer.start().and(developer.x()).whileTrue(shooter.sysIdKickupQuasistatic(Direction.kForward));
        //developer.start().and(developer.y()).whileTrue(shooter.sysIdKickupQuasistatic(Direction.kReverse));
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return selectedAuton;
        /*
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
        */
    }

    /**
     * Takes a list of PathPlanner auton file names and returns a list of PathPlanner commands based on the list.
     * @param autonList
     * @return
     */
    public ArrayList<Command> loadAllAutonomous(Supplier<ArrayList<String>> autonList) {
        ArrayList<Command> commandList = new ArrayList<Command>();
        for (var i : autonList.get()) {
            commandList.add(new PathPlannerAuto(i));
        }

        return commandList;
    }

    // Use these triggers to determine when to filter the list of autons.
    public Trigger DSAttached = new Trigger(DriverStation::isDSAttached);
    public Trigger alliancePresent = new Trigger(() -> {return DriverStation.getAlliance().isPresent();});
}
