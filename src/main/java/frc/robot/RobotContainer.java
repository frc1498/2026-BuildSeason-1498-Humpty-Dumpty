// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    public Climber climber = new Climber(climberConfig, MotorEnableConstants.TelemetryLevel.LIMITED);

    public HopperConfig hopperConfig = new HopperConfig();
    public Hopper hopper = new Hopper(hopperConfig, MotorEnableConstants.TelemetryLevel.LIMITED);

    public IntakeConfig intakeConfig = new IntakeConfig();
    public Intake intake = new Intake(intakeConfig, MotorEnableConstants.TelemetryLevel.LIMITED);

    public File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    public Selector autonSelect = new Selector(autonFolder, ".auto", "Auton Selector", MotorEnableConstants.TelemetryLevel.LIMITED);
    public PathPlannerAuto selectedAuton;
    public ArrayList<PathPlannerAuto> autonCommands = new ArrayList<PathPlannerAuto>();

    //Gamepad assignment
    //Instantiate 
    private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorControllerPort);
    // private final CommandXboxController developer = new CommandXboxController(ControllerConstants.kDeveloperControllerPort);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean DSLatch = false;
    private double precisionDampenerTranslation = 1.0; //Translation Speed Limiter
    private double precisionDampenerRotation = 1.0; //Rotation Speed Limiter
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.001).withRotationalDeadband(MaxAngularRate * 0.001) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Vision vision = new Vision(drivetrain, drivetrain::getStateCopy, drivetrain::addVisionMeasurement, MotorEnableConstants.TelemetryLevel.LIMITED);

    public ShooterConfig shooterConfig = new ShooterConfig();
    public Shooter shooter = new Shooter(shooterConfig, drivetrain::getStateCopy, MotorEnableConstants.TelemetryLevel.LIMITED);
    // Because I'm lazy, I'm leaving the configurations for the kickup and spindexer motors in the shooter config.
    // We'll just pass the shooter config into the kickup and spindexer subsystems to use the already in-place configurations.
    public Kickup kickup = new Kickup(shooterConfig, MotorEnableConstants.TelemetryLevel.LIMITED);
    public Spindexer spindexer = new Spindexer(shooterConfig, MotorEnableConstants.TelemetryLevel.LIMITED);

    public final Move move = new Move(climber,hopper,intake,shooter,drivetrain,kickup,spindexer);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Create DogLog - Temporarily disabled to stop loop overruns
        //DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        //DogLog.setOptions(new DogLogOptions().withCaptureConsole(false));
        // Configure the trigger bindings
        configureBindings();
        registerAutoCommands();
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
                drive.withVelocityX(-(Math.pow(driver.getLeftY() * precisionDampenerTranslation,3)) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-(Math.pow(driver.getLeftX() * precisionDampenerTranslation,3)) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * precisionDampenerRotation) // Drive counterclockwise with negative X (left)
            )
        );

        //shooter.setDefaultCommand(shooter.setShooterOutputs());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.povUp().or(driver.povDown()).and(this.DSAttached).and(this.getDSLatch.negate()).onTrue(autonSelect.filterList(() -> {return DriverStation.getAlliance().get().toString();}));

        /*
        this.DSAttached.onTrue(autonSelect.filterList(() -> {return DriverStation.getAlliance().get().toString();})
            .andThen(() -> {this.autonCommands = this.loadAllAutonomous(autonSelect.currentList());}).ignoringDisable(true)
            .andThen(() -> {this.selectedAuton = autonCommands.get(autonSelect.currentIndex().get());}).ignoringDisable(true));
        */

         this.DSAttached.and(this.alliancePresent).and(this.getDSLatch.negate()).whileTrue(autonSelect.filterList(() -> {return DriverStation.getAlliance().get().toString();})
            .andThen(() -> {this.autonCommands = this.loadAllAutonomous(autonSelect.currentList());}).ignoringDisable(true)
            .andThen(() -> {this.selectedAuton = autonCommands.get(autonSelect.currentIndex().get());}).ignoringDisable(true)
            .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(this.selectedAuton.getStartingPose())).ignoringDisable(true))
            .andThen(this.setLatch()).ignoringDisable(true));

        // Add the limelight pose estimate to the drivetrain estimate.
        //vision.addLimelightPose.whileTrue(vision.addMegaTag2(() -> {return drivetrain;}));
      
        //===================================================
        //===================Driver Commands=================
        //===================================================
        //Driver POV Up/Down - Auton Select (only in disabled)
        driver.povUp().and(this.getDSLatch).and(RobotModeTriggers.disabled()).onTrue(autonSelect.increment().andThen(() -> {this.selectedAuton = this.autonCommands.get(this.autonSelect.currentIndex().get());}).andThen(drivetrain.runOnce(() -> drivetrain.resetPose(this.selectedAuton.getStartingPose())).ignoringDisable(true)).ignoringDisable(true));
        driver.povDown().and(this.getDSLatch).and(RobotModeTriggers.disabled()).onTrue(autonSelect.decrement().andThen(() -> {this.selectedAuton = this.autonCommands.get(this.autonSelect.currentIndex().get());}).andThen(drivetrain.runOnce(() -> drivetrain.resetPose(this.selectedAuton.getStartingPose())).ignoringDisable(true)).ignoringDisable(true));
        
        //Driver RTrigger: Intake
        driver.rightTrigger(0.1).onTrue(move.intake()).onFalse(move.stopIntake());

        //Driver RBumper hopperRetract
        driver.rightBumper().onTrue(move.hopperRetract());

        //Driver LBumper climb
        driver.leftBumper().onTrue(move.climbRetract());
        
        //Driver left trigger: Shoot
        driver.leftTrigger(0.1).whileTrue(Commands.sequence(move.setTargetToAllianceHub()).
        andThen(Commands.parallel(setShootOnMoveSpeed(),move.startWhileMoveShoot())))
        .onFalse(Commands.parallel(setNormalMoveSpeed(),move.stopShoot()));

        //Driver back: Zero drivetrain
        //driver.back().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()));

        //Driver x: Zero the Climb System
        driver.x().onTrue(move.zeroClimb());

        //Driver start: zero gyro
        driver.start().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()));

        //Driver a: Climb extend
        driver.a().onTrue(move.climbExtend());
  
        //driver.b Reverse Intake
        driver.b().whileTrue(move.reverseIntake()).onFalse(move.stopIntake());

        //Driver y: Zero Hopper position
        driver.y().onTrue(move.setHopperZeroPosition());

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
        /*
        operator.x().whileTrue(Commands.sequence(move.setTargetToAllianceCornerLeft(),
            Commands.parallel(setShootOnMoveSpeed(),move.startWhileMoveShoot())))
            .onFalse(Commands.sequence(setNormalMoveSpee(),move.setTargetToAllianceHub(),move.stopShoot()));

        //Operator B button
        operator.b().whileTrue(Commands.sequence(move.setTargetToAllianceCornerRight(),
            Commands.parallel(setShootOnMoveSpeed(),move.startWhileMoveShoot())))
            .onFalse(Commands.sequence(setNormalMoveSpeed(),move.setTargetToAllianceHub(),move.stopShoot()));
        */

        //Operator A button
        //operator.a()

        //Operator RTrigger: Static pass button
        operator.rightTrigger(0.1).whileTrue(move.startShootStatic()).onFalse(move.stopShoot());

        //Operator RBumper 
        //operator.rightBumper()

        //Operator LTrigger
        //operator.leftTrigger(0.1)

        //Operator LBumper
        //operator.leftBumper()

        //Operator y button: Agitate manually
        operator.y().onTrue(move.hopperRetract()).onFalse(move.hopperExtend());

        //Operator Start
        //operator.START().

        //===================================================
        //=================Misc Commands=====================
        //===================================================
        
        //Auto agitate when shooting and not intaking
        driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(move.agitateHopper());
        
        //===================================================
        //==================Developer Commands===============
        //===================================================        
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

    }

    public void registerAutoCommands(){
        NamedCommands.registerCommand("intake", move.intake());
        NamedCommands.registerCommand("stopIntake", move.stopIntake());
        NamedCommands.registerCommand("shoot", move.startWhileMoveShoot());
        NamedCommands.registerCommand("stopShoot", move.stopShoot());
        NamedCommands.registerCommand("extendHopper", move.hopperExtend());
        NamedCommands.registerCommand("retractHopperMid", move.hopperMid());
    }

    /**
     * Takes a list of PathPlanner auton file names and returns a list of PathPlanner commands based on the list.
     * @param autonList
     * @return
     */
    public ArrayList<PathPlannerAuto> loadAllAutonomous(Supplier<ArrayList<String>> autonList) {
        ArrayList<PathPlannerAuto> commandList = new ArrayList<PathPlannerAuto>();
        for (var i : autonList.get()) {
            commandList.add(new PathPlannerAuto(i));
        }

        return commandList;
    }

    public Command setLatch() {return Commands.runOnce(() -> {this.DSLatch = true;});}

    public Command setShootOnMoveSpeed () {return Commands.runOnce(() -> {
        this.precisionDampenerTranslation = 0.8;
        this.precisionDampenerRotation = 0.5;}
        );
    }

    public Command setNormalMoveSpeed () {return Commands.runOnce(()-> {
        this.precisionDampenerTranslation = 1.0;
        this.precisionDampenerRotation = 1.0;}
        );
    }


    // Use these triggers to determine when to filter the list of autons.
    public Trigger DSAttached = new Trigger(DriverStation::isDSAttached);
    public Trigger getDSLatch = new Trigger(() -> {return this.DSLatch;});
    public Trigger alliancePresent = new Trigger(() -> {return DriverStation.getAlliance().isPresent();});
}
