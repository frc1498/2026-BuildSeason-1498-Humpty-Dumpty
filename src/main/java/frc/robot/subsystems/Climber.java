// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the climber

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ClimberConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;
import frc.robot.constants.ClimberConstants;

//For zeroing - 5 A Supply Current, 50 A Stator Current, 3 V Output

public class Climber extends SubsystemBase {
/* Variables */
  public TalonFX climbMotor;  //Motor type definition

  public PositionVoltage climbMotorMode; //Motor control type definition

  public DutyCycleOut dutyCycleOut; //Motor Control type definition

  private double desiredClimbMotorPosition;

  public boolean hasDSAttachLatched = false;

  ClimberConfig climberConfig; //Create an object of type climber config to use to configure motors

  // Fall back to a default of no telemetry.
  MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  /**
   * The constructor for the climber subsystem.
   * @param config - The motor configuration for the motors in the climber subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Climber(ClimberConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;
    this.climberConfig = config;

    this.climbMotor = new TalonFX(ClimberConfig.kClimbMotorCANID, MotorEnableConstants.canivore);  //Create a motor for this subsystem
    this.climbMotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(climbMotor, config.climbMotorConfig);

    this.dutyCycleOut = new DutyCycleOut(0.0);
    this.climbMotor.setPosition(0);

    SmartDashboard.putData("Climber", this);
  }

  /**
   * Applies a TalonFX configuration to a TalonFX motor.
   * @param mechanism - The motor to apply the configuration to.
   * @param config - The TalonFX configuration to apply to the motor.
   */
  public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config){     
    //Start Configuring Hopper Motor
    StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
       mechanismStatus = mechanism.getConfigurator().apply(config);
      if (mechanismStatus.isOK()) break;
    }
    if (!mechanismStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
    }
  }

  /* Private Methods */

  /* Private Set/Goto Methods */

  /**
   * Commands the climber motor to a position.
   * The method checks if the motor is enabled, and that the setpoint is within the safety limits for the climber before sending the command to the motor.
   * This method also caches the parameter as {@code desiredClimbMotorPosition} for use in the subsystem.
   * @param position - The position setpoint for the climber motor, in rotations.
   */
  private void goToPositionClimb(double position) {
    desiredClimbMotorPosition = position;
    if (MotorEnableConstants.kClimbMotorEnabled) {
      if (position <= ClimberConstants.kClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kClimbSafeRetract) { //Check that Value is above retracted distance
        climbMotor.setControl(climbMotorMode.withPosition(position));
      }
    }
  }

  /**
   * Stops the climber motor.
   * This method sets the control mode to dutyCycleOut with an output of 0.
   * This allows the mechanism to 'idle' without attempting to keep it's previously commanded position.
   */
  private void climberStop(){
    climbMotor.setControl(dutyCycleOut.withOutput(0.0));
  }

  /**
   * Returns the name of the current command running on the subsystem.
   * @return The name of the currently running command.  "No Command" if no command is scheduled.
   */
  private String getCurrentCommandName() {
      if (this.getCurrentCommand() == null) {
          return "No Command";
      }
      else {
          return this.getCurrentCommand().getName();
      }
  }

  /* Private Get Methods */

  /**
   * Get the current position of the climber.  This is the motor position, technically different from the mechanism position.
   * But not in a significant way.
   * @return The current position of the climber motor, in rotations.
   */
  private double getClimbPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Check if the climber is at or near the position.
   * @param position - The position to check if the climber is at or near.
   * @return True if the current climber position is at the position parameter, plus or minus a deadband.
   */
  private boolean isClimbAtPosition(double position) {
    return ((position - ClimberConstants.kClimbDeadband) <= this.getClimbPosition()) && ((position + ClimberConstants.kClimbDeadband) >= this.getClimbPosition());
  }

  /**
   * Should return true if the supply limit has been exceeded.
   * @return True if the climb motor stator current is higher than 20 A.
   */
  private boolean climberCurrentLimitTripped() {
    return (this.climbMotor.getStatorCurrent().getValueAsDouble() > 20);
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
        DogLog.log("Current Climber Command", this.getCurrentCommandName());
        DogLog.log("Desired Climb Motor Position", this.desiredClimbMotorPosition);
        DogLog.log("Actual Climb Motor Position", this.getClimbPosition());
        DogLog.log("Actual Climb Motor Current", this.climbMotor.getSupplyCurrent().getValueAsDouble());
        break;
      default:
        break;
    }
  }

  /* Commands */

  /**
   * A zeroing routine for the climber.  This should drive the motor down until the supply current limit is tripped (or stalled).
   * @return
   */
  public Command zeroRoutine() {
    return run(
      () -> {
        this.configureMechanism(this.climbMotor, this.climberConfig.climbMotorZeroConfig);
        this.climbMotor.setControl(this.dutyCycleOut.withOutput(-0.25));
      }
    ).until(this.isClimberCurrentLimitTripped)
    .andThen(
      runOnce(
        () -> {
          this.climbMotor.setControl(this.dutyCycleOut.withOutput(0));
          this.climbMotor.setPosition(0);
          if (this.climbMotor.getStatorCurrent().getValueAsDouble() > 20) {
            this.configureMechanism(this.climbMotor, this.climberConfig.climbMotorConfig);
          }
        }
      )
    ).withName("zeroRoutine");
  }

  /**
   * A command that drives the climber to a constant position.  Used as a factory for more specific climber position commands.
   * @param climberPosition - The climber position to move to.
   * @return - A factory command that moves the climb motor to the position defined by the constant.
   */
  private Command climbMove(double climberPosition) {
    return run (() -> {this.goToPositionClimb(climberPosition);}).withName("climbMove");
  }

  public Command newClimbExtend() {return this.climbMove(ClimberConstants.kClimbExtend).until(isClimbExtended).withName("climbExtend");}
  public Command newClimbRetract() {return this.climbMove(ClimberConstants.kClimbRetract).until(isClimbRetracted).withName("climbRetract");}
  public Command newClimbHome() {return this.climbMove(ClimberConstants.kClimbHome).until(isClimbHome).withName("climbHome");}

  public Command climbExtend() {
    return run(
      () -> {this.goToPositionClimb(ClimberConstants.kClimbExtend);}
    ).until(isClimbExtended).withName("climbExtend");
  }

  public Command climbRetract() {
    return run(
      () -> {this.goToPositionClimb(ClimberConstants.kClimbRetract);}
    ).until(isClimbRetracted).withName("climbRetract");
  }

  public Command climbHome() {
    return run(
      () -> {this.goToPositionClimb(ClimberConstants.kClimbHome);}
    ).until(isClimbHome).withName("climbHome");
  }

  /**
   * Stops the climber motor, placing it in an idle state.
   * @return A command that runs the {@code climberStop} method.
   */
  public Command climbStop() {
    return run(
      () -> {this.climberStop();}
    ).withName("climbStop");
  }

  /* Triggers */
  public Trigger isClimbExtended = new Trigger(() -> {return this.isClimbAtPosition(ClimberConstants.kClimbExtend);});
  public Trigger isClimbRetracted = new Trigger(() -> {return this.isClimbAtPosition(ClimberConstants.kClimbRetract);});
  public Trigger isClimbHome = new Trigger(() -> {return this.isClimbAtPosition(ClimberConstants.kClimbHome);});
  public Trigger isClimberCurrentLimitTripped = new Trigger(this::climberCurrentLimitTripped);

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addDoubleProperty("Desired Climb Position", () -> {return this.desiredClimbMotorPosition;}, null);
        builder.addDoubleProperty("Current Climb Position", this::getClimbPosition, null);
      case LIMITED:
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
      case NONE:
        // No values!
      default:
        //break;
    } 
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
