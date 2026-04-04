// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private TalonFX shooterLeftMotor;   // Motor type definition
  private TalonFX shooterRightMotor;   // Motor type definition
  private TalonFX hoodMotor;       // Motor type definition

  private VelocityTorqueCurrentFOC shooterMotorMode;   // Motor control type definition
  private PositionTorqueCurrentFOC hoodMotorMode;   // Motor control type definition

  private ShooterConfig shooterConfig;  // Create an object of type shooter subsystem config used to configure motors

  public Pose2d currentTarget;
  public Pose2d hubTarget;

  private double desiredHoodAngle;
  private double desiredHoodMotorRotations;
  private double desiredShooterVelocity;

  private boolean hoodAtPosition;
  private boolean shooterAtVelocity;
  private boolean readyToFire;

  private Supplier<SwerveDriveState> swerveStateSupplier;
  private SwerveDriveState swerveState;
  private double distanceToTarget;
  private double distanceToVirtualTarget;

  private double virtualHoodAngle;
  private double virtualFlywheelVelocity;

  private double whileMoveHoodAngle;
  private double whileMoveFlywheelVelocity;
  private double whileMoveAngle;

  private double currentHoodAngle;
  private double currentHoodRotations;
  private double currentShooterVelocity;

  private ShooterSim sim;
  private TalonFXSimState shooterLeftMotorSim;
  private TalonFXSimState shooterRightMotorSim;
  private TalonFXSimState hoodMotorSim;
  
  public DutyCycleOut shooterDutyCycle;

  private double simTime;

  private LinearFilter velocityFilter = LinearFilter.movingAverage(3);

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  private String allianceColor = "Blue";
  public Pose2d targetLocation = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0)); //Default it to blue

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

    this.shooterLeftMotor = new TalonFX(ShooterConfig.kShooterLeftMotorCANID, MotorEnableConstants.canivore);    // Create the first shooter motor.
    this.configureMechanism(this.shooterLeftMotor, this.shooterConfig.shooterLeftMotorConfig);

    this.shooterRightMotor = new TalonFX(ShooterConfig.kShooterRightMotorCANID, MotorEnableConstants.canivore);    // Create the second shooter motor.
    this.shooterMotorMode = new VelocityTorqueCurrentFOC(0);                                        // Set the control mode for both shooter motors.
    this.configureMechanism(this.shooterRightMotor, this.shooterConfig.shooterRightMotorConfig);
    
    this.shooterLeftMotor.setControl(new Follower(shooterRightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
      
    this.hoodMotor = new TalonFX(ShooterConfig.kHoodMotorCANID, MotorEnableConstants.canivore);            // Create hood adjustment motor.
    this.hoodMotorMode = new PositionTorqueCurrentFOC(0);                                           // Set the contorl mode for the adjustment motor.
    this.configureMechanism(this.hoodMotor, this.shooterConfig.hoodMotorConfig);

    this.shooterLeftMotorSim = this.shooterLeftMotor.getSimState();
    this.shooterRightMotorSim = this.shooterRightMotor.getSimState();
    this.hoodMotorSim = this.hoodMotor.getSimState();

    this.sim = new ShooterSim(
      this.shooterConfig,
      this.hoodMotorSim,
      this.shooterLeftMotorSim,
      this.shooterRightMotorSim
    );

    // Publish subsystem data to SmartDashboard.
    SmartDashboard.putData("Shooter", this);
    //SmartDashboard.putData("Shooter/Pose", this.targetingField);
    //SmartDashboard.putData("Shooter/Sim", this.sim.getVis());

    this.shooterDutyCycle = new DutyCycleOut(0.0);

    this.requestShoot = false;

    this.hoodMotor.setPosition(0);

    this.setHoodAngle(0);
    this.desiredShooterVelocity = -10;

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
  /* Hood Private Methods */

  /**
   * Set the position of hood.
   * @param position - The desired hood position, in angle of hood
   */
  private void setHoodAngle(double position) {
    // Always store the setpoint, to track the desired position.
    this.desiredHoodAngle = position;

    //Convert to Rotations
    this.desiredHoodMotorRotations = this.desiredHoodAngle / 360 * ShooterConstants.kHoodGearRatio;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kHoodMotorEnabled) {
      // Check if the desired hood position is within the allowable safety range.
      // Do not update the hood position if it is out of range.
      if (this.isSetpointWithinSafetyRange(this.desiredHoodAngle, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend)) {
        this.hoodMotor.setControl(this.hoodMotorMode.withPosition(this.desiredHoodMotorRotations));
      } else {
        // Log a fault with DogLog if the desired hood position was out of range.
        // DogLog.logFault(ShooterFault.HOOD_SETPOINT_OUT_OF_RANGE);
      }
    }
  }

  /**
   * Return the current position of the hood.
   * @return - The current position of the hood, in rotations.
   */
  private double getHoodAngle() {
    // Convert motor rotations and return Angle
    return (this.hoodMotor.getPosition().getValueAsDouble() * 360 / ShooterConstants.kHoodGearRatio);
  }

  /**
   * Put both shooter motors in an 'idle' state.
   * Set the control mode to DutyCycleOut and set the output to zero.
   */
  private void stopShooting() {
    this.shooterLeftMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.shooterRightMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.desiredShooterVelocity = 0;
  }

  /**
   * Return the current position of the hood.
   * @return The current position of the hood, in motor rotations.
   */
  private double getHoodRotations() {
    return (this.hoodMotor.getPosition().getValueAsDouble());
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
    if (MotorEnableConstants.kTopShooterLeftMotorEnabled && MotorEnableConstants.kTopShooterRightMotorEnabled) {
      this.shooterRightMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
      this.shooterLeftMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    }
  }

  /**
   * Return the current velocity of the first shooter motor.
   * @return - The current velocity of the first shooter motor, in rotations per second.
   */
  private double getShooterVelocity() {
    double velocity = velocityFilter.calculate(this.shooterRightMotor.getVelocity().getValueAsDouble());
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

  private boolean isHoodAtPosition() {
    return ((this.getHoodAngle() < (this.desiredHoodAngle + ShooterConstants.kHoodPositionDeadband)) 
    && (this.getHoodAngle() > (this.desiredHoodAngle - ShooterConstants.kHoodPositionDeadband)));
  }

  /* Public Shoot Commands */
  public Command requestStopShoot() {return runOnce(() -> {this.requestStopShooting();});}
  public Command requestStartShoot() {return runOnce(() -> {this.requestStartShooting();});}

  public Command stopShoot() {return runOnce(() -> {this.stopShooting();});}

  private Command setShooter(double velocity) {return run(() -> {this.setShooterVelocity(velocity);});}

  public Command newStartShootStatic() {return this.setShooter(50.0).until(this.isShooterAtVelocity).withName("startShotStatic");}
  public Command newStartShootFast() {return this.setShooter(80.0).until(this.isShooterAtVelocity).withName("startShootFast");}

  public Command startShootStatic(){
    return run(() -> {this.setShooterVelocity(50);}).until(isShooterAtVelocity); //Was 70
  }

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

  /* Public Hood Commands */

  private Command setHood(DoubleSupplier position) {return runOnce(() -> {this.setHoodAngle(position.getAsDouble());}).withName("setHoodAngle");}

  public Command newHood0() {return this.setHood(() -> {return 0.0;}).withName("hood0");}
  public Command newHood35() {return this.setHood(() -> {return 35.0;}).withName("hood35");}
  public Command newAutoHood() {return this.setHood(() -> {return this.virtualHoodAngle;}).withName("autoHood");}
  public Command newWhileMoveHood() {return this.setHood(() -> {return this.whileMoveHoodAngle;}).withName("whileMoveHood");}

  public Command hood0() {
    return runOnce(() -> {this.setHoodAngle(0);});
  }

  public Command hood30() {
    return runOnce(() -> {this.setHoodAngle(35);});
  }

  /**
   * Set the hood based on the distance to the hub.
   * @return
   */
  public Command autoHood() {
    return runOnce(() -> {this.setHoodAngle(this.virtualHoodAngle);});
  }

  /**
   * Set the hood based on the estimated distance to the hub for the shoot while move.
   * @return
   */
  public Command whileMoveHood() {
    return runOnce(() -> {this.setHoodAngle(this.whileMoveHoodAngle);});
  }

  public Supplier<Rotation2d> robotTarget() {
    return () -> {return Rotation2d.fromDegrees(this.whileMoveAngle);};
  }

  //======================Triggers=========================
  public Trigger isHoodAtPosition = new Trigger(() -> {return isHoodAtPosition();});
  public Trigger isShooterAtVelocity = new Trigger(() -> {return isShooterAtSpeed();});
  public Trigger isReadyToFire = new Trigger(() -> {return this.readyToFire;});

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addDoubleProperty("Virtual Hood Angle", () -> {return this.virtualHoodAngle;}, null);
        builder.addDoubleProperty("Virtual Flywheel Velocity", () -> {return this.virtualFlywheelVelocity;}, null);
        builder.addDoubleProperty("Desired Hood Angle", () -> {return this.desiredHoodAngle;}, null);
        builder.addDoubleProperty("Desired Hood Motor Rotations", () -> {return this.desiredHoodMotorRotations;}, null);
        builder.addDoubleProperty("Desired Shooter Velocity", () -> {return this.desiredShooterVelocity;}, null);
        builder.addDoubleProperty("Current Hood Rotations", () -> {return this.currentHoodRotations;},null);
        builder.addDoubleProperty("Current Hood Angle", () -> {return this.currentHoodAngle;}, null);
        builder.addDoubleProperty("Current Shooter Velocity", () -> {return this.currentShooterVelocity;}, null);
        builder.addBooleanProperty("Shooter At Velocity", () -> {return this.shooterAtVelocity;}, null);
        builder.addDoubleProperty("Distance to Target", () -> {return this.distanceToTarget;}, null);
        builder.addDoubleProperty("Distance to Virtual Target", () -> {return this.distanceToVirtualTarget;}, null);
        builder.addStringProperty("Target", () -> {return targetLocation.toString();}, null);
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

    this.swerveState = this.swerveStateSupplier.get();

    this.currentHoodAngle = this.getHoodAngle();
    this.currentHoodRotations = this.getHoodRotations();
    this.currentShooterVelocity = this.getShooterVelocity();

    // Wondering if caching this will reduce the CPU usage of this call.
    this.swerveState = this.swerveStateSupplier.get();
    
    allianceColor = DriverStation.getAlliance().get().toString();
    
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
    this.readyToFire = this.hoodAtPosition && this.shooterAtVelocity;

    //First attempt of the shoot while moving calculation.
    this.distanceToTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToShooter), targetLocation);
    
    this.distanceToVirtualTarget = ShotCalculation.getInstance().getDistanceToVirtualTarget(this.swerveState.Speeds, this.swerveState.Pose, targetLocation);

    this.virtualHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToTarget);
    this.virtualFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToTarget);

    this.whileMoveHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToVirtualTarget);
    this.whileMoveFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToVirtualTarget);
    this.whileMoveAngle = ShotCalculation.getInstance().getVirtualTarget().getTranslation().minus(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToShooter).getTranslation()).getAngle().getDegrees();
    
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
