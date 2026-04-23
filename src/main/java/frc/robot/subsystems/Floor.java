// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the floor

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.config.FloorConfig;
import frc.robot.constants.FloorConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The floor subsystem.  Contains the flywheel, turret, hood adjustment, floor, and ball kickup.
 */
public class Floor extends SubsystemBase {

/*==================Variables=======================*/

  private TalonFX floorMotor;  // Motor type definition
  private VelocityVoltage floorMotorMode; // Motor control type definition
  private double desiredFloorVelocity;
  private double currentFloorVelocity;
  public DutyCycleOut floorDutyCycle;
  private FloorConfig floorConfig;

  /* Logging Variables */
  @Logged(importance = Importance.CRITICAL)
  private String currentCommand = "";

  /* Subsystem Alerts */
  Alert floorMotorDisconnected = new Alert("Floor Motor Disconnected", AlertType.kError);

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  /**
   * The constructor for the floor subsystem.
   * @param config - The motor configuration for the motors in the floor subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Floor(FloorConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

  this.telemetryLevel = telemetryLevel;
  this.floorConfig = config;

  this.floorMotor = new TalonFX(FloorConfig.kFloorMotorCANID, MotorEnableConstants.canivore);  // Create the floor motor.
  this.floorMotorMode = new VelocityVoltage(0);                                      // Set the control mode for the floor motor.
  this.floorDutyCycle = new DutyCycleOut(0.0);
  this.configureMechanism(this.floorMotor, this.floorConfig.floorMotorConfig);


  // Publish subsystem data to SmartDashboard.
  SmartDashboard.putData("Floor", this);
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

  /**
   * Checks if the current setpoint is within the range of minimum and maximum parameters.
   * @param currentSetpoint - The setpoint to check.
   * @param minimum - The lower bound of the allowable range.
   * @param maximum - The upper bound of the allowable range.
   * @return True if the current setpoint is within the range of the minimum and maximum.
   */
  private boolean isSetpointWithinSafetyRange(double currentSetpoint, double minimum, double maximum) {
    return ((currentSetpoint >= minimum) && (currentSetpoint <= maximum));
  }

  /* Floor Private Methods */

  /**
   * Set the velocity of the floor motor.
   * @param velocity - The desired velocity of the floor motor, in rotations per second.
   */
  private void setFloorVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredFloorVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kFloorMotorEnabled) {
      this.floorMotor.setControl(this.floorMotorMode.withVelocity(this.desiredFloorVelocity));
    }
  }

  /**
   * Check if the floor motor is at or near the desired velocity.
   * @return True if the current floor motor velocity is at the desired floor motor velocity, plus and minus a deadband.
   */
  private boolean isFloorAtSpeed () {
      return ((floorMotor.getVelocity().getValueAsDouble() < (this.desiredFloorVelocity + FloorConstants.kFloorVelocityDeadband)) 
    && (floorMotor.getVelocity().getValueAsDouble() > (this.desiredFloorVelocity - FloorConstants.kFloorVelocityDeadband)));
  }

  /**
   * Stops the floor motor.
   * Sets the desired velocity to 0, and sets the control mode of the motor to dutyCycleOut to allow the motor to 'idle'.
   */
  private void stopFloorMotor(){
    this.desiredFloorVelocity = 0.0;
    floorMotor.setControl(floorDutyCycle.withOutput(FloorConstants.kFloorZeroDutyCycle));
  }

  /**
   * Return the current velocity of the floor motor.
   * @return - The current velocity of the floor motor, in rotations per second.
   */
  private double getFloorVelocity() {
    this.currentFloorVelocity = floorMotor.getVelocity().getValueAsDouble();
    return this.currentFloorVelocity;
  }

  /* Misc. Private Methods */

  /**
   * Checks if a current variable is within a deadband to the setpoint.
   * @param setpoint - The setpoint the current variable should be at.
   * @param current - The current variable to check (process variable).
   * @param deadBand - The allowable deadband (+ and -) from the setpoint.
   * @return
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

  /* Public Methods */

  /* Public Floor Commands */
  public Command stopFloor() {
    return runOnce(() -> {this.stopFloorMotor();}).withName("stopFloor");
  }

  /**
   * A factory command that sets the velocity of the floor motor.
   * @param velocity - The velocity setpoint of the floor motor, in rotations per second.
   * @return A command that runs the {@code setFloorVelocity} method.
   */
  private Command setFloor(double velocity) {
    return runOnce(() -> {this.setFloorVelocity(velocity);}).withName("setFloorVelocity");
  }

  public Command newReverseFloor() {return this.setFloor(FloorConstants.kFloorOuttake).withName("reverseFloor");};
  public Command newForwardFloor() {return this.setFloor(FloorConstants.kFloorIntake).withName("forwardFloor");};

  public Command reverseFloor() {
    return runOnce(() -> {this.setFloorVelocity(FloorConstants.kFloorOuttake);});
  }

  public Command forwardFloor() {
    return runOnce(() -> {this.setFloorVelocity(FloorConstants.kFloorIntake);});
  }

  /**
   * Set the neutral mode of the floor motor to coast.
   * @return A command that sets the neutral mode of the floor motor to coast.
   */
  public Command setFloorCoast() {
    return runOnce(() -> {this.setMotorNeutralMode(this.floorMotor, NeutralModeValue.Coast);});
  }

  /**
   * Reset the neutral mode of the floor motor to the initial code configuration.
   * @return A command that resets the neutral mode of the floor motor.
   */
  public Command resetFloorMotorNeutral() {
    return runOnce(() -> {this.setMotorNeutralMode(this.floorMotor, this.floorConfig.floorMotorConfig.MotorOutput.NeutralMode);});
  }

  public Trigger isFloorAtVelocity = new Trigger(() -> {return isFloorAtSpeed();});
  //public Trigger isFloorStopped = new Trigger(() -> {return this.isFloorStopped();});

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addBooleanProperty("Is Floor at Velocity", this.isFloorAtVelocity, null);
      case LIMITED:
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
      case NONE:
        // No values!
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    this.currentCommand = this.getCurrentCommandName();
    this.floorMotorDisconnected.set(this.floorMotor.isConnected());
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
