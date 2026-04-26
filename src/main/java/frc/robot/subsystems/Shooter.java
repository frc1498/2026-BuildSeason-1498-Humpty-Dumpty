// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.MatchInfo;
import frc.robot.ShotCalculation;
import frc.robot.config.ShooterConfig;
import frc.robot.sim.ShooterSim;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The shooter subsystem.  Contains the flywheel, turret, hood adjustment, spindexer, and ball kickup.
 */
public class Shooter extends SubsystemBase {

  /* Variables */

  private TalonFX shooterTopLeftMotor;   // Motor type definition
  private TalonFX shooterBottomLeftMotor;
  private TalonFX shooterTopRightMotor;   // Motor type definition
  private TalonFX shooterBottomRightMotor;

  private VelocityVoltage shooterMotorMode;   // Motor control type definition

  private ShooterConfig shooterConfig;  // Create an object of type shooter subsystem config used to configure motors

  public Pose2d currentTarget;
  public Pose2d hubTarget;

  private double desiredShooterVelocity;

  private boolean shooterAtVelocity;
  private boolean readyToFire;

  private Supplier<SwerveDriveState> swerveStateSupplier;
  private SwerveDriveState swerveState;

  private double currentShooterVelocity;

  private ShooterSim sim;
  private TalonFXSimState shooterLeftMotorSim;
  private TalonFXSimState shooterRightMotorSim;
  
  public DutyCycleOut shooterDutyCycle;

  private double simTime;

  private LinearFilter velocityFilter = LinearFilter.movingAverage(3);

  /* Logging Variables */
  @Logged(importance = Importance.CRITICAL)
  private String currentCommand = "";
  @Logged
  public Pose2d targetLocation = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0)); //Default it to blue
  @Logged
  private double distanceToTarget;
  @Logged
  private double distanceToVirtualTarget;
  @Logged
  private double virtualFlywheelVelocity;
  @Logged
  private double whileMoveFlywheelVelocity;
  @Logged
  private Rotation2d whileMoveAngle = new Rotation2d(0.0);
  @Logged
  private Rotation2d targetOffset = new Rotation2d(0.0);

  /* Subsystem Alerts */
  Alert shooterTopLeftMotorDisconnected = new Alert("Shooter Top Left Motor Disconnected", AlertType.kError);
  Alert shooterBottomLeftMotorDisconnected = new Alert("Shooter Bottom Left Motor Disconnected", AlertType.kError);
  Alert shooterTopRightMotorDisconnected = new Alert("Shooter Top Right Motor Disconnected", AlertType.kError);
  Alert shooterBottomRightMotorDisconnected = new Alert("Shooter Bottom Right Motor Disconnected", AlertType.kError);  

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  private String allianceColor = "Blue";

  private boolean requestShoot;

  public Field2d targetingField = new Field2d();

  /**
   * Creates a new instance of the shooter subsystem.
   * @param config - The motor configurations for all motors in the subsystem.
   * @param swerveDriveState - A supplier of the current swerve drive state from the drivetrain subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Shooter(ShooterConfig config, Supplier<SwerveDriveState> swerveDriveState, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;
    this.swerveStateSupplier = swerveDriveState;
    this.shooterConfig = config;

    this.shooterTopLeftMotor = new TalonFX(ShooterConfig.kShooterTopLeftMotorCANID, MotorEnableConstants.canivore);    // Create the first shooter motor.
    this.configureMechanism(this.shooterTopLeftMotor, this.shooterConfig.shooterTopLeftMotorConfig);

    this.shooterBottomLeftMotor = new TalonFX(ShooterConfig.kShooterBottomLeftMotorCANID, MotorEnableConstants.canivore);
    this.configureMechanism(this.shooterBottomLeftMotor, this.shooterConfig.shooterBottomLeftMotorConfig);

    this.shooterTopRightMotor = new TalonFX(ShooterConfig.kShooterTopRightMotorCANID, MotorEnableConstants.canivore);    // Create the second shooter motor.                                  
    this.configureMechanism(this.shooterTopRightMotor, this.shooterConfig.shooterTopRightMotorConfig);

    this.shooterBottomRightMotor = new TalonFX(ShooterConfig.kShooterBottomRightMotorCANID, MotorEnableConstants.canivore);
    this.configureMechanism(this.shooterBottomRightMotor, this.shooterConfig.shooterBottomRightMotorConfig);

    this.shooterMotorMode = new VelocityVoltage(0); // Set the control mode for both shooter motors.

    this.shooterLeftMotorSim = this.shooterTopLeftMotor.getSimState();
    this.shooterRightMotorSim = this.shooterTopRightMotor.getSimState();

    this.sim = new ShooterSim(
      this.shooterConfig,
      this.shooterLeftMotorSim,
      this.shooterRightMotorSim
    );

    // Publish subsystem data to SmartDashboard.
    SmartDashboard.putData("Shooter", this);
    //SmartDashboard.putData("Shooter/Pose", this.targetingField);
    //SmartDashboard.putData("Shooter/Sim", this.sim.getVis());

    this.shooterDutyCycle = new DutyCycleOut(0.0);

    this.requestShoot = false;

    this.desiredShooterVelocity = -10;

    this.whileMoveAngle = new Rotation2d(0.0);

  }

  /**
   * Apply the configuration to the motor.  This will attempt to re-apply the configuration if unsuccessful, up to 5 times.
   * @param mechanism - The TalonFX object (motor) to apply the configuration to.
   * @param config - The set of configurations to apply.
   */
  public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config) {
    // Start Configuring the motor with the supplied configuration.
    StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

    // Attempt to apply the configuration 5 times.  Immediately stop if the configuration was successful.
    for(int i = 0; i < 5; ++i) {
      mechanismStatus = mechanism.getConfigurator().apply(config);
      if (mechanismStatus.isOK()) {break;}
    }

    // If the configuration was still not successful, print an error to the console.
    if (!mechanismStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
    }
  }

  /* Private Methods */

  /**
   * Updates the neutral mode of the motor.  Useful for setting the motors to coast to make mechanisms easier to move by hand.
   * @param mechanism - The TalonFX motor to apply the neutral mode to.
   * @param mode - The new neutral mode of the motor.  Either Brake or Coast.
   */
  private void setMotorNeutralMode(TalonFX mechanism, NeutralModeValue mode) {
    var motorConfig = new MotorOutputConfigs();
    var currentConfigurator = mechanism.getConfigurator();
    currentConfigurator.refresh(motorConfig);
    motorConfig.NeutralMode = mode;
    currentConfigurator.apply(motorConfig);
  }

  /* Hood Private Methods */

  /**
   * Put both shooter motors in an 'idle' state.
   * Set the control mode to DutyCycleOut and set the output to zero.
   */
  private void stopShooting() {
    this.shooterTopLeftMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.shooterBottomLeftMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.shooterTopRightMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.shooterBottomRightMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.desiredShooterVelocity = 0;
  }

  /**
   * Returns true if the current setpoint is within the range of minimum and maximum parameters.
   * @param currentSetpoint - The current setpoint to check.
   * @param minimum - The minimum value allowed.
   * @param maximum - The maximum value allowed.
   * @return True if the current setpoint is greater than the minimum and less than the maximum.
   */
  private boolean isSetpointWithinSafetyRange(double currentSetpoint, double minimum, double maximum) {
    return ((currentSetpoint >= minimum) && (currentSetpoint <= maximum));
  }

  /* Shooter Private Methods */

  /**
   * Set the velocity of both shooter motors.
   * @param velocity - The desired velocity of the shooter motors, in rotations per second.
   */
  private void setShooterVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredShooterVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kTopShooterLeftMotorEnabled) {
      this.shooterTopLeftMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    } 
    if (MotorEnableConstants.kTopShooterRightMotorEnabled) {
      this.shooterTopRightMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    }
    if (MotorEnableConstants.kBottomShooterLeftMotorEnabled) {
      this.shooterBottomLeftMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    }
    if (MotorEnableConstants.kBottomShooterRightMotorEnabled) {
      this.shooterBottomRightMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));  
    }
  }

  /**
   * Return the current velocity of the first shooter motor.
   * @return - The current velocity of the first shooter motor, in rotations per second.
   */
  private double getShooterVelocity() {
    double velocity = velocityFilter.calculate(this.shooterTopRightMotor.getVelocity().getValueAsDouble());
    return velocity;
  }

  private void setTargetAllianceCornerRight() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      targetLocation = ShooterConstants.kRedRight;
    } else if (allianceColor == "Blue") {
      targetLocation = ShooterConstants.kBlueRight;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
  }

  private void setTargetAllianceCornerLeft() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      targetLocation = ShooterConstants.kRedLeft;
    } else if (allianceColor == "Blue") {
      targetLocation = ShooterConstants.kBlueLeft;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
  }

  private void setTargetAllianceHub() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      targetLocation = ShooterConstants.kRedHubCenter;
    } else if (allianceColor == "Blue") {
      targetLocation = ShooterConstants.kBlueHubCenter;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
  }

  /* Misc. Private Methods */

  /**
   * Checks if a current variable is within a deadband to the setpoint.
   * @param setpoint - The setpoint the current variable should be at.
   * @param current - The current variable to check (process variable).
   * @param deadBand - The allowable deadband (+ and -) from the setpoint.
   * @return True if the current value is at the setpoint, plus or minus the deadband.
   */
  private boolean atSetpoint(double setpoint, double current, double deadBand) {
    return ((current >= (setpoint - deadBand)) && (current <= (setpoint + deadBand)));
  }  

  /**
   * Returns a string of the name of the currently running command.
   * If no command is running, return "No Command".
   * @return A string with the name of the currently running command.
   */
  private String getCurrentCommandName() {
    /*
    if (this.getCurrentCommand() == null) {
      return "No Command";
    }
    else {
      return this.getCurrentCommand().getName();
    }
    */
    // Refactoring this method with a ternary operator.
    return (this.getCurrentCommand() == null) ? "No Command" : this.getCurrentCommand().getName();
  }

  /**
   * Logs variables from the subsystem via DogLog.  The amount of variables logged can be controlled with the logLevel parameter.
   * @param logLevel - The level of logging to enable.
   */
  private void log(MotorEnableConstants.LogLevel logLevel) {
    switch (logLevel) {
      case NONE:
        break;
      case FULL:
        break;
      default:
        break;
    }
  }

  /* Public Methods */

  private void requestStopShooting() {this.requestShoot = false;}
  private void requestStartShooting() {this.requestShoot = true;}

  private boolean isShooterAtSpeed() {  //Modified to make sure we are above speed only.
    return ((this.getShooterVelocity() > (this.desiredShooterVelocity - ShooterConstants.kShooterVelocityDeadband)));
  }

  /**
   * Set the neutral mode of the shooter motors to coast.
   * @return A command that sets the neutral mode of the shooter motors to coast.
   */
  public Command setShooterCoast() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.shooterTopLeftMotor, NeutralModeValue.Coast);
      this.setMotorNeutralMode(this.shooterBottomLeftMotor, NeutralModeValue.Coast);
      this.setMotorNeutralMode(this.shooterTopRightMotor, NeutralModeValue.Coast);
      this.setMotorNeutralMode(this.shooterBottomRightMotor, NeutralModeValue.Coast);
    });
  }

  /**
   * Reset the neutral mode of the shooter motors to the initial code configuration.
   * @return A command that resets the neutral mode of the shooter motors.
   */
  public Command resetShooterMotorsNeutral() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.shooterTopLeftMotor, this.shooterConfig.shooterTopLeftMotorConfig.MotorOutput.NeutralMode);
      this.setMotorNeutralMode(this.shooterBottomLeftMotor, this.shooterConfig.shooterBottomLeftMotorConfig.MotorOutput.NeutralMode);
      this.setMotorNeutralMode(this.shooterTopRightMotor, this.shooterConfig.shooterTopRightMotorConfig.MotorOutput.NeutralMode);
      this.setMotorNeutralMode(this.shooterBottomRightMotor, this.shooterConfig.shooterBottomRightMotorConfig.MotorOutput.NeutralMode);
    });
  }

  /* Public Shoot Commands */
  public Command requestStopShoot() {return runOnce(() -> {this.requestStopShooting();});}
  public Command requestStartShoot() {return runOnce(() -> {this.requestStartShooting();});}
  public Command stopShoot() {return runOnce(() -> {this.stopShooting();});}
  private Command setShooter(double velocity) {return run(() -> {this.setShooterVelocity(velocity);});}
  public Command newStartShootStatic() {return this.setShooter(50.0).until(this.isShooterAtVelocity).withName("startShotStatic");}
  public Command newStartShootFast() {return this.setShooter(80.0).until(this.isShooterAtVelocity).withName("startShootFast");}

  //This is used for the static shot in case vision brakes
  public Command startShootStatic(){
    return run(() -> {this.setShooterVelocity(41.5);}).until(isShooterAtVelocity); //Was 70
  }

  //Just testing long shots.  This has no particular point aside from that.
  public Command startShootFast(){
    return run(() -> {this.setShooterVelocity(80);}).until(isShooterAtVelocity);
  }

  /**
   * Set the shooter velocity based on the distance to the hub.
   * @return
   */
  public Command autoShoot() {
    return runOnce(() -> {this.setShooterVelocity(this.virtualFlywheelVelocity);});
  }

  /**
   * Set the shooter velocity based on the distance to the estimated hub for the shoot while move.
   * @return
   */
  public Command whileMoveShoot() {
    return runOnce(() -> {this.setShooterVelocity(this.whileMoveFlywheelVelocity);});
  }

  public Command setTargetToAllianceCornerRight() {
    return runOnce(() -> {this.setTargetAllianceCornerRight();});
  }

  public Command setTargetToAllianceCornerLeft() {
    return runOnce(() -> {this.setTargetAllianceCornerLeft();});
  }

  public Command setTargetToAllianceHub() {
    return runOnce(() -> {this.setTargetAllianceHub();});
  }

  private String getAlliance() {
    return DriverStation.getAlliance().get().toString();

  }

  public Command getOurAlliance () {
    return runOnce(() -> {getAlliance();});
  }

  public Supplier<Rotation2d> robotTarget() {
    return () -> {return this.whileMoveAngle.plus(this.targetOffset);};
  }

  //======================Triggers=========================
  public Trigger isShooterAtVelocity = new Trigger(() -> {return isShooterAtSpeed();});
  public Trigger isReadyToFire = new Trigger(() -> {return this.readyToFire;});

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addDoubleProperty("Rotation Target", () -> {return this.whileMoveAngle.getDegrees();}, null);
      case LIMITED:
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
      case NONE:
        // No values!
      default:
       // break;
   }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.currentCommand = this.getCurrentCommandName();
    this.shooterTopLeftMotorDisconnected.set(!this.shooterTopLeftMotor.isConnected());
    this.shooterBottomLeftMotorDisconnected.set(!this.shooterBottomLeftMotor.isConnected());
    this.shooterTopRightMotorDisconnected.set(!this.shooterTopRightMotor.isConnected());
    this.shooterBottomRightMotorDisconnected.set(!this.shooterBottomRightMotor.isConnected());

    this.swerveState = this.swerveStateSupplier.get();

    this.currentShooterVelocity = this.getShooterVelocity();

    // Wondering if caching this will reduce the CPU usage of this call.
    this.swerveState = this.swerveStateSupplier.get();
    
    allianceColor = MatchInfo.getInstance().getAlliance();
    
    // Since the shooter command doesn't set it now, I should get the target location continuously in the shooter periodic() method.
    this.targetLocation = MatchInfo.getInstance().getCurrentTarget();
    this.targetOffset = MatchInfo.getInstance().getTargetOffset();

    if (DriverStation.isAutonomousEnabled()) {
      if (allianceColor == "Red") {
        targetLocation = ShooterConstants.kRedHubCenter;
      } else if (allianceColor == "Blue") {
        targetLocation = ShooterConstants.kBlueHubCenter;
      } else {
        // Code to handle the case where the alliance color is not yet available
      }
    }

    // Signal that we are ready to fire if the hood is at position and the shooter is at velocity.
    this.readyToFire = this.shooterAtVelocity;

    //First attempt of the shoot while moving calculation.
    this.distanceToTarget = ShotCalculation.getInstance().calculateTargetDistance(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToShooter), targetLocation);
    
    this.distanceToVirtualTarget = ShotCalculation.getInstance().calculateDistanceToVirtualTarget(this.swerveState.Speeds, this.swerveState.Pose, targetLocation);

    this.virtualFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToTarget);

    this.whileMoveFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToVirtualTarget);
    //this.whileMoveAngle = ShotCalculation.getInstance().getVirtualTarget().getTranslation().minus(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToShooter).getTranslation()).getAngle();
                        //ShotCalculation.getInstance().getVirtualTarget().minus(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToTurret)).getTranslation().getAngle().getDegrees());
    this.whileMoveAngle = ShotCalculation.getInstance().getDriveAngle(this.swerveState.Pose, ShotCalculation.getInstance().getVirtualTarget().getTranslation()).rotateBy(MatchInfo.getInstance().getAlliance() == "Blue" ? Rotation2d.kZero : Rotation2d.k180deg);
    
    // Every loop, update the odometry with the pose of the virtual target.
    switch (this.telemetryLevel) {
      case FULL:
        this.targetingField.getObject("Hub Target").setPose(ShotCalculation.getInstance().getVirtualTarget());
        this.targetingField.getObject("Angler").setPose(0.0,0.0,Rotation2d.kZero);
      case LIMITED:
      case NONE:
      default:
        break;
    }

    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //this.sim.simulationPeriodic();
    //this.sim.updateShooterHoodVis(this.currentShooterVelocity, this.currentHoodAngle, this.hoodAtPosition);
  }
}
