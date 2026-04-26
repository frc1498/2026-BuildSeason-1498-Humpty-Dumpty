// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import frc.robot.config.FrontKickupConfig;
import frc.robot.constants.FrontKickupConstants;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The front kickup subsystem.  Contains the front kickup motor.
 */
public class FrontKickup extends SubsystemBase {

  /* Variables */

  private TalonFX frontKickupMotor;     // Motor type definition
  private VelocityTorqueCurrentFOC frontKickupMotorMode;    // Motor control type definition
  private double desiredFrontKickupVelocity;
  private double currentFrontKickupVelocity;
  public DutyCycleOut frontKickupDutyCycle;
  private FrontKickupConfig frontKickupConfig;
  
  /* Logging Variables */
  @Logged(importance = Importance.CRITICAL)
  private String currentCommand = "";

  /* Subsystem Alerts */
  Alert frontKickupMotorDisconnected = new Alert("Front Kickup Motor Disconnected", AlertType.kError);

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  /**
   * Creates a new instance of the Kickup subsystem.
   * @param config - The motor configurations for all motors in the subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public FrontKickup(FrontKickupConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;
    this.frontKickupConfig = config;
    this.frontKickupMotor = new TalonFX(FrontKickupConfig.kFrontKickupMotorCANID, MotorEnableConstants.canivore);        // Create the kickup motor.
    this.frontKickupMotorMode = new VelocityTorqueCurrentFOC(0);                                         // Set the control mode for the kickup motor.
    this.frontKickupDutyCycle = new DutyCycleOut(0.0);
    this.configureMechanism(this.frontKickupMotor, this.frontKickupConfig.frontKickupMotorConfig);

    // Publish subsystem data to SmartDashboard.
    SmartDashboard.putData("Front Kickup", this);
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

  /* Front Kickup Private Methods */

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
   * Set the velocity of the front kickup motor.
   * @param velocity - The desired velocity of the front kickup motor, in rotations per second.
   */
  private void setFrontKickupVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredFrontKickupVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kFrontKickupMotorEnabled) {
      //this.frontKickupMotor.setControl(this.frontKickupMotorMode.withVelocity(this.desiredFrontKickupVelocity));
        this.frontKickupMotor.setControl(this.frontKickupMotorMode.withVelocity(this.desiredFrontKickupVelocity));
      //this.hopperMotor.setControl(this.dutyCycleOut.withOutput(0.25));
    }
  }

  /**
   * Return the current velocity of the kickup motor.
   * @return - The current velocity of the kickup motor, in rotations per second.
   */
  private double getFrontKickupVelocity() {
    this.currentFrontKickupVelocity = this.frontKickupMotor.getVelocity().getValueAsDouble();
    return this.currentFrontKickupVelocity;
  }

  /**
   * Stops the front kickup motor.
   * Sets the desired velocity to 0, and sets the control mode of the motor to dutyCycleOut to allow the motor to 'idle'.
   */
  private void stopFrontKick(){
    frontKickupMotor.setControl(frontKickupDutyCycle.withOutput(FrontKickupConstants.kFrontKickupZeroDutyCycle));
  }

  /* Misc. Private Methods */

  /**
   * Checks if a current variable is within a deadband to the setpoint.
   * @param setpoint - The setpoint the current variable should be at.
   * @param current - The current variable to check (process variable).
   * @param deadBand - The allowable deadband (+ and -) from the setpoint.
   * @return True if the current value is at the setpoint and within the deadband.
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
  /* Public Front Kickup Commands */

  /**
   * A command that stops the front kickup motor.
   * The control mode of the motor is set to DutyCycleOut and the output is set to 0 to 'idle' the motor.
   * @return A command that runs the {@code stopFrontKick} method.
   */
  public Command stopFrontKickup() {
    return runOnce(() -> {this.stopFrontKick();});
  }

  /**
   * Set the neutral mode of the front kickup motor to coast.
   * @return A command that sets the neutral mode of the front kickup motor to coast.
   */
  public Command setFrontKickupCoast() {
    return runOnce(() -> {this.setMotorNeutralMode(this.frontKickupMotor, NeutralModeValue.Coast);});
  }

  /**
   * Reset the neutral mode of the kickup motor to the initial code configuration.
   * @return A command that resets the neutral mode of the front kickup motor.
   */
  public Command resetFrontKickupMotorNeutral() {
    return runOnce(() -> {this.setMotorNeutralMode(this.frontKickupMotor, this.frontKickupConfig.frontKickupMotorConfig.MotorOutput.NeutralMode);});
  }

  /**
   * A factory command that sets the velocity of the front kickup motor.
   * @param velocity - The velocity setpoint of the kickup motor, in rotations per second.
   * @return A command that runs the {@code setFrontKickupVelocity} method.
   */
  private Command setFrontKickup(double velocity) {return runOnce(() -> {this.setFrontKickupVelocity(velocity);}).withName("setFrontKickupVelocity");}

  public Command newReverseFrontKickup() {return this.setFrontKickup(FrontKickupConstants.kFrontKickupOuttake).withName("reverseFrontKickup");}
  public Command newForwardFrontKickup() {return this.setFrontKickup(FrontKickupConstants.kFrontKickupIntake).withName("forwardFrontKickup");}

  public Command reverseFrontKickup() {
    return runOnce(() -> {this.setFrontKickupVelocity(FrontKickupConstants.kFrontKickupOuttake);});
  }

  public Command forwardFrontKickup() {
    return runOnce(() -> {this.setFrontKickupVelocity(FrontKickupConstants.kFrontKickupIntake);});
  }

  /* Triggers */

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
        break;
    }
  }

  @Override
  public void periodic() {
    this.currentCommand = this.getCurrentCommandName();
    this.frontKickupMotorDisconnected.set(!this.frontKickupMotor.isConnected());
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
