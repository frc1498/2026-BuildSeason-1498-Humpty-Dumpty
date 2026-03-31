// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The shooter subsystem.  Contains the flywheel, turret, hood adjustment, spindexer, and ball kickup.
 */
public class FrontKickup extends SubsystemBase {

/*==================Variables=======================*/

  private TalonFX frontKickupMotor;     // Motor type definition
  private VelocityTorqueCurrentFOC frontKickupMotorMode;    // Motor control type definition
  private double desiredFrontKickupVoltage;
  private double currentFrontKickupVoltage;
  public DutyCycleOut frontKickupDutyCycle;
  
  // Fall back to a default of no telemetry.
  MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

    /**
     * Returns the kickup velocity for the state.
     * @return - Desired velocity for the kickup motor, in rotations per second.
     */
    public double frontKickup() {
      return this.KickupVoltage;
    }

    /**
     * Returns the motor direction for the state.
     * @return - Motor direction.  True is forward (intake), false is reverse (outtake).
     */
    public boolean direction() {
      return this.direction;
    }
  }

/**
 * Creates a new instance of the Kickup subsystem.
 * @param config - The motor configurations for all motors in the subsystem.
 */
public FrontKickup(ShooterConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

  this.telemetryLevel = telemetryLevel;
  this.frontKickupMotor = new TalonFX(FrontKickupConfig.kFrontKickupMotorCANID, "canivore");        // Create the kickup motor.
  this.frontKickupMotorMode = new VelocityTorqueCurrentFOC(0);                                         // Set the control mode for the kickup motor.
  this.configureMechanism(this.frontKickupMotor, this.FrontKickupConfig.frontKickupMotorConfig);

  // Publish subsystem data to SmartDashboard.
  //SmartDashboard.putData("Kickup", this);

  //turretZeroed = true;
  frontKickupDutyCycle = new DutyCycleOut(0.0);
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

  //=========================================================
  /*===================Private Methods=====================*/
  //===========================================================

  //===================================Kickup Private Methods======================================

  /**
   * Set the velocity of the kickup motor.
   * @param velocity - The desired velocity of the kickup motor, in rotations per second.
   */
  private void setFrontKickupVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredFrontKickupVoltage = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kFrontKickupMotorEnabled) {
      this.frontKickupMotor.setControl(this.frontKickupMotorMode.withVelocity(this.desiredFrontKickupVelocity));
    }
  }

  /**
   * Return the current velocity of the kickup motor.
   * @return - The current velocity of the kickup motor, in rotations per second.
   */
  private double getFrontKickupVelocity() {
    this.currentFrontKickupVoltage = this.frontKickupMotor.getVelocity().getValueAsDouble();
    return this.frontKickupMotor.getVelocity().getValueAsDouble();
  }

  private void stopFrontKick(){
    frontKickupMotor.setControl(frontKickupDutyCycle.withOutput(FrontKickupConstants.kFrontKickupZeroDutyCycle));
  }

  //===============================Misc. Private Methods===================

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
   * @return
   */
  private String getCurrentCommandName() {
      if (this.getCurrentCommand() == null) {
          return "No Command";
      }
      else {
          return this.getCurrentCommand().getName();
      }
      // Refactoring this method with a ternary operator.
      // return (this.getCurrentCommand == null) ? "No Command" : this.getCurrentCommand().getName();
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

  //====================Public Methods=====================
  //======================Public Kickup Commands=====================
  public Command stopFrontKickup() {
    return runOnce(() -> {this.stopFrontKick();});
  }

  public Command reverseFrontKickup() {
    return runOnce(() -> {this.setFrontKickupVelocity(FrontKickupConstants.kFrontKickupOuttake);});
  }

  public Command forwardFrontKickup() {
    return runOnce(() -> {this.setFrontKickupVoltage(FrontKickupConstants.kFrontKickupIntake);});
  }

  //======================Triggers=========================

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
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
