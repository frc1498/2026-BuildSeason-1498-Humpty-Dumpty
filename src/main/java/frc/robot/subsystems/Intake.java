// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the run/stop of the intake rollers

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.config.IntakeConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import dev.doglog.DogLog;

public class Intake extends SubsystemBase {
  /* Variables */
  public TalonFX intakeRightMotor;  //Motor type definition
  public TalonFX intakeLeftMotor;  //Motor type definition

  public VelocityVoltage intakeVelocityVoltage; //Motor control type definition

  private IntakeConfig intakeConfig; //Create an object of type IntakeConfig

  public String intakeState = "stopped";

  public DutyCycleOut intakeDutyCycle;

  /* Logging Variables */
  @Logged(importance = Importance.CRITICAL)
  private String currentCommand = "";

  /* Subsystem Alerts */
  Alert intakeLeftMotorDisconnected = new Alert("Intake Left Motor Disconnected", AlertType.kError);
  Alert intakeRightMotorDisconnected = new Alert("Intake Right Motor Disconnected", AlertType.kError);  

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  /**
   * The constructor for the intake subsystem.  The two motors that run the intake bar.
   * @param config - The configuration for the motors in the intake subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Intake(IntakeConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {
    this.telemetryLevel = telemetryLevel;
    this.intakeConfig = config;

    this.intakeRightMotor = new TalonFX(IntakeConfig.kIntakeRightCANID, "canivore");  //Create the intake motor for this subsystem
    this.configureMechanism(this.intakeRightMotor, this.intakeConfig.intakeRightMotorConfig);

    intakeLeftMotor = new TalonFX(IntakeConfig.kIntakeLeftCANID, "canivore");  //Create the intake motor for this subsystem
    this.configureMechanism(this.intakeLeftMotor, this.intakeConfig.intakeLeftMotorConfig);
  
    this.intakeDutyCycle = new DutyCycleOut(0);
    this.intakeVelocityVoltage = new VelocityVoltage(0);

    SmartDashboard.putData("Intake", this);
  }

  /**
   * Apply the configuration to the motor.  This will attempt to re-apply the configuration if unsuccessful, up to 5 times.
   * @param mechanism - The TalonFX object (motor) to apply the configuration to.
   * @param config - The set of configurations to apply.
   */
  public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config){     
    //Start Configuring Climber Motor
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
   * Sets the left and right intake motors to the intake velocity, in rotations per second.
   * The control will only be set if the motor is enabled.
   * Each motor can be enabled or disabled separately.
   */
  private void intake(){
    if (MotorEnableConstants.kIntakeLeftMotorEnabled) {
        intakeLeftMotor.setControl(intakeVelocityVoltage.withVelocity(IntakeConstants.kIntakeSpeed));
    }
    if (MotorEnableConstants.kIntakeRightMotorEnabled) {
        intakeRightMotor.setControl(intakeVelocityVoltage.withVelocity(IntakeConstants.kIntakeSpeed));
    }
  }
  
  /**
   * Sets the left and right intake motors to the outtake velocity, in rotations per second.
   * The control will only be set if the motor is enabled.
   * Each motor can be enabled or disabled separately.
   */
  private void outtake(){
    if (MotorEnableConstants.kIntakeLeftMotorEnabled) {
        intakeLeftMotor.setControl(intakeDutyCycle.withOutput(-0.6));
    }
    if (MotorEnableConstants.kIntakeRightMotorEnabled) {
        intakeLeftMotor.setControl(intakeDutyCycle.withOutput(-0.6));
    }
    
  }

  /**
   * Stops both intake motors.
   * Both motors are changed over to the duty cycle control mode with the output set to 0.
   * This puts the motor in an 'idle' state.
   */
  private void stop(){
    if (MotorEnableConstants.kIntakeLeftMotorEnabled) {
      intakeLeftMotor.setControl(intakeDutyCycle.withOutput(0));
    }
    if (MotorEnableConstants.kIntakeRightMotorEnabled) {
      intakeRightMotor.setControl(intakeDutyCycle.withOutput(0));
    }
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
        DogLog.log("Current Intake Command", getCurrentCommandName());
        DogLog.log("Intake Current", intakeLeftMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Actual Intake State", intakeState);
        DogLog.log("Actual Intake Velocity", intakeLeftMotor.getVelocity().getValueAsDouble());
        break;
      default:
        break;
    }
  }

  /* Public Methods */
  
  /**
   * Set the neutral mode of the intake motors to coast.
   * @return A command that sets the neutral mode of the intake motors to coast.
   */
  public Command setIntakeCoast() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.intakeLeftMotor, NeutralModeValue.Coast);
      this.setMotorNeutralMode(this.intakeRightMotor, NeutralModeValue.Coast);
    });
  }

  /**
   * Reset the neutral mode of the intake motors to the initial code configuration.
   * @return A command that resets the neutral mode of the intake motors.
   */
  public Command resetIntakeMotorsNeutral() {
    return runOnce(() -> {
      this.setMotorNeutralMode(this.intakeLeftMotor, this.intakeConfig.intakeLeftMotorConfig.MotorOutput.NeutralMode);
      this.setMotorNeutralMode(this.intakeRightMotor, this.intakeConfig.intakeRightMotorConfig.MotorOutput.NeutralMode);
    });
  }

  public Command intakeSuck() {return runOnce(() -> {this.intake();}).withName("intakeSuck");}
  public Command intakeSpit() {return run(() -> {this.outtake();}).withName("intakeSpit");}
  public Command intakeStop() {return runOnce(() -> {this.stop();}).withName("intakeStop");}

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addStringProperty("Intake State", () -> {return this.intakeState;}, null);
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
    // This method will be called once per scheduler run
    this.currentCommand = this.getCurrentCommandName();
    this.intakeLeftMotorDisconnected.set(!this.intakeLeftMotor.isConnected());
    this.intakeRightMotorDisconnected.set(!this.intakeRightMotor.isConnected());
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
