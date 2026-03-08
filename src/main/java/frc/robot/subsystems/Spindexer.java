// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.config.ShooterConfig;


import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;


/**
 * The shooter subsystem.  Contains the flywheel, turret, hood adjustment, spindexer, and ball kickup.
 */
public class Spindexer extends SubsystemBase {

/*==================Variables=======================*/

  private TalonFX spindexerMotor;  // Motor type definition

  private VelocityVoltage spindexerMotorMode; // Motor control type definition

  private ShooterConfig shooterConfig;  // Create an object of type shooter subsystem config used to configure motors

  private double desiredSpindexerVelocity;

  private boolean spindexerAtVelocity;

  private double currentSpindexerVelocity;


  public DutyCycleOut spindexerDutyCycle;

  private double simTime;

  private LinearFilter velocityFilter = LinearFilter.movingAverage(3);

  //boolean turretZeroed;
  boolean requestShoot;

  public Field2d targetingField = new Field2d();

  private enum ShooterState {
    IDLE(0.0, 0.0, true),
    FORWARD(ShooterConstants.kSpindexerIntake, ShooterConstants.kKickupIntake, true),
    REVERSE(ShooterConstants.kSpindexerOuttake, ShooterConstants.kKickupOuttake, false);

    private double spindexerVelocity;
    private double KickupVoltage;
    private boolean direction;

    /**
     * Constructor for the ShooterState enumeration.
     * @param spindexerVelocity - The velocity of the spindexer.
     * @param KickupVoltage - The direction of the kickup motor.
     * @param direction - Represents direction of the motors.  True is forward (intake), false is reverse (outtake).
     */
    ShooterState(double spindexerVelocity, double KickupVoltage, boolean direction) {
      this.spindexerVelocity = spindexerVelocity;
      this.KickupVoltage = KickupVoltage;
      this.direction = direction;
    }

    /**
     * Returns the spindexer velocity for the state.
     * @return - Desired velocity for the spindexer, in rotations per second.
     */
    public double spindexer() {
      return this.spindexerVelocity;
    }

    /**
     * Returns the kickup velocity for the state.
     * @return - Desired velocity for the kickup motor, in rotations per second.
     */
    public double kickup() {
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
 * Creates a new instance of the spindexer subsystem.
 * @param config - The motor configurations for all motors in the subsystem.
 */
public Spindexer(ShooterConfig config) {

  this.shooterConfig = config;

  this.spindexerMotor = new TalonFX(ShooterConfig.kSpindexerMotorCANID, "canivore");  // Create the spindexer motor.
  this.spindexerMotorMode = new VelocityVoltage(0);                                      // Set the control mode for the spindexer motor.
  this.configureMechanism(this.spindexerMotor, this.shooterConfig.spindexerMotorConfig);


  // Publish subsystem data to SmartDashboard.
  SmartDashboard.putData("Spindexer", this);

  spindexerDutyCycle = new DutyCycleOut(0.0);
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

  /**
   * Returns true if the current setpoint is within the range of minimum and maximum parameters.
   */
  private boolean isSetpointWithinSafetyRange(double currentSetpoint, double minimum, double maximum) {
    return ((currentSetpoint >= minimum) && (currentSetpoint <= maximum));
  }

  //===========================Spindexer Private Methods=====================================

  /**
   * Set the velocity of the spindexer motor.
   * @param velocity - The desired velocity of the spindexer motor, in rotations per second.
   */
  private void setSpindexerVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredSpindexerVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kSpindexerMotorEnabled) {
      this.spindexerMotor.setControl(this.spindexerMotorMode.withVelocity(this.desiredSpindexerVelocity));
    }
  }

  private boolean isSpindexerAtSpeed () {
      return ((spindexerMotor.getVelocity().getValueAsDouble() < (this.desiredSpindexerVelocity + ShooterConstants.kSpindexerVelocityDeadband)) 
    && (spindexerMotor.getVelocity().getValueAsDouble() > (this.desiredSpindexerVelocity - ShooterConstants.kSpindexerVelocityDeadband)));
  }

  private void stopSpindex(){
    this.desiredSpindexerVelocity=0;
    spindexerMotor.setControl(spindexerDutyCycle.withOutput(ShooterConstants.kSpindexerZeroDutyCycle));
  }

  /**
   * Return the current velocity of the spindexer motor.
   * @return - The current velocity of the spindexer motor, in rotations per second.
   */
  private double getSpindexerVelocity() {
    this.currentSpindexerVelocity = spindexerMotor.getVelocity().getValueAsDouble();
    return spindexerMotor.getVelocity().getValueAsDouble();
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

  //=========================================================
  /*====================Public Methods=====================*/
  //=========================================================

  //====================Public Methods=====================
  //=======================Public Spindexer Commands==================
  public Command stopSpindexer() {
    return run(() -> {this.stopSpindex();}).until(isSpindexerAtVelocity);
  }

  public Command reverseSpindexer() {
    return runOnce(() -> {this.setSpindexerVelocity(ShooterConstants.kSpindexerOuttake);});
  }

  public Command forwardSpindexer() {
    return runOnce(() -> {this.setSpindexerVelocity(ShooterConstants.kSpindexerIntake);});
  }

  public Trigger isSpindexerAtVelocity = new Trigger(() -> {return isSpindexerAtSpeed();});
  //public Trigger isSpindexerStopped = new Trigger(() -> {return this.isSpindexerStopped();});

  @Override
  public void initSendable(SendableBuilder builder) {
    //builder.addStringProperty("Command", this::getCurrentCommandName, null);
    //builder.addDoubleProperty("Distance to Target", () -> {return this.distanceToTarget;}, null);
    //builder.addDoubleProperty("Distance to Virtual Target", () -> {return this.distanceToVirtualTarget;}, null);
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
