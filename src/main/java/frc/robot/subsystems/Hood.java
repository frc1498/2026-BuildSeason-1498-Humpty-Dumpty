// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the hood motor 

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.MatchInfo;
import frc.robot.ShotCalculation;
import frc.robot.config.ShooterConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The hood subsystem.  Contains the hood adjustment.
 */
public class Hood extends SubsystemBase {

  /* Variables */

  private TalonFX hoodMotor;       // Motor type definition

  private PositionVoltage hoodMotorMode;   // Motor control type definition

  private ShooterConfig hoodConfig;  // Create an object of type shooter subsystem config used to configure motors

  private double desiredHoodAngle;
  private double desiredHoodMotorRotations;

  private boolean hoodAtPosition;
  private boolean readyToFire;

  private Supplier<SwerveDriveState> swerveStateSupplier;
  private SwerveDriveState swerveState;
  private double distanceToTarget;
  private double distanceToVirtualTarget;

  private double virtualHoodAngle;

  private double whileMoveHoodAngle;

  private double currentHoodAngle;
  private double currentHoodRotations;
  
  public DutyCycleOut shooterDutyCycle;

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  public Pose2d targetLocation = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0)); //Default it to blue

  private boolean requestShoot;

  public Field2d targetingField = new Field2d();

  /**
   * Creates a new instance of the shooter subsystem.
   * @param config - The motor configurations for all motors in the subsystem.
   * @param swerveDriveState - A supplier of the current swerve drive state from the drivetrain subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Hood(ShooterConfig config, Supplier<SwerveDriveState> swerveDriveState, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;
    this.swerveStateSupplier = swerveDriveState;
    this.hoodConfig = config;
      
    this.hoodMotor = new TalonFX(HoodConfig.kHoodMotorCANID, MotorEnableConstants.canivore);            // Create hood adjustment motor.
    this.hoodMotorMode = new PositionVoltage(0);                                           // Set the contorl mode for the adjustment motor.
    this.configureMechanism(this.hoodMotor, this.hoodConfig.hoodMotorConfig);

    // Publish subsystem data to SmartDashboard.
    SmartDashboard.putData("Hood", this);

    this.requestShoot = false;

    this.hoodMotor.setPosition(0);

    this.setHoodAngle(0);
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

  private boolean isHoodAtPosition() {
    return ((this.getHoodAngle() < (this.desiredHoodAngle + ShooterConstants.kHoodPositionDeadband)) 
    && (this.getHoodAngle() > (this.desiredHoodAngle - ShooterConstants.kHoodPositionDeadband)));
  }

  /**
   * Set the neutral mode of the hood motor to coast.
   * @return A command that sets the neutral mode of the hood motor to coast.
   */
  public Command setHoodCoast() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.hoodMotor, NeutralModeValue.Coast);
    });
  }

  /**
   * Reset the neutral mode of the hood motor to the initial code configuration.
   * @return A command that resets the neutral mode of the hood motor.
   */
  public Command resetHoodMotorsNeutral() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.hoodMotor, this.hoodConfig.hoodMotorConfig.MotorOutput.NeutralMode);
    });
  }

  /* Public Shoot Commands */
  public Command requestStopShoot() {return runOnce(() -> {this.requestStopShooting();});}
  public Command requestStartShoot() {return runOnce(() -> {this.requestStartShooting();});}

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
    return runOnce(() -> {this.setHoodAngle(30);});
  }

  public Command hood20() {
    return runOnce(() -> {this.setHoodAngle(20);});
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

  //======================Triggers=========================
  public Trigger isHoodAtPosition = new Trigger(() -> {return isHoodAtPosition();});
  public Trigger isHoodUp = new Trigger(() -> {return this.getHoodAngle() >= 4.5;});
  public Trigger isReadyToFire = new Trigger(() -> {return this.readyToFire;});

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
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

    // Wondering if caching this will reduce the CPU usage of this call.
    this.swerveState = this.swerveStateSupplier.get();
    
    // Since the hood subsystem doesn't set it now, I should get the target location continuously in the hood periodic() method.
    this.targetLocation = MatchInfo.getInstance().getCurrentTarget();

    // Signal that we are ready to fire if the hood is at position.
    this.readyToFire = this.hoodAtPosition;

    //First attempt of the shoot while moving calculation.
    this.distanceToTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveState.Pose.transformBy(ShooterConstants.kRobotToShooter), targetLocation);
    
    this.distanceToVirtualTarget = ShotCalculation.getInstance().getDistanceToVirtualTarget(this.swerveState.Speeds, this.swerveState.Pose, targetLocation);

    this.virtualHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToTarget);

    this.whileMoveHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToVirtualTarget);

    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
