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
import edu.wpi.first.wpilibj.DriverStation;
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
//==================Variables=======================
  public TalonFX climbMotor;  //Motor type definition

  public PositionVoltage climbMotorMode; //Motor control type definition

  public DutyCycleOut dutyCycleOut; //Motor Control type definition

  private double desiredClimbMotorPosition;
  private boolean isClimberCurrentLimitLatched=false;

  public boolean hasDSAttachLatched = false;

  ClimberConfig climberConfig; //Create an object of type climber config to use to configure motors

  // Fall back to a default of no telemetry.
  MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

  //===============Constructor======================
  public Climber(ClimberConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;

    climbMotor = new TalonFX(ClimberConfig.kClimbMotorCANID, "canivore");  //Create a motor for this subsystem
    climbMotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(climbMotor, config.climbMotorConfig);
    this.climberConfig=config;
    this.dutyCycleOut = new DutyCycleOut(0.0);

    this.climbMotor.setPosition(0);

    SmartDashboard.putData("Climber", this);
  }

  //===================Configuration=====================
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

//=======================================================
//====================Private Methods====================
//=======================================================
//===============Private Set/Goto Methods================
  private void goToPositionClimb(double position) {
    desiredClimbMotorPosition = position;
    if (MotorEnableConstants.kClimbMotorEnabled) {
      if (position <= ClimberConstants.kClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kClimbSafeRetract) { //Check that Value is above retracted distance
        climbMotor.setControl(climbMotorMode.withPosition(position));
      }
    }
  }

  private void climberStop(){
    climbMotor.setControl(dutyCycleOut.withOutput(0.0));
  }

  private String getCurrentCommandName() {
      if (this.getCurrentCommand() == null) {
          return "No Command";
      }
      else {
          return this.getCurrentCommand().getName();
      }
  }

  //=====================Private Get Methods==================================

  private double getClimbPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  //=====================Private Trigger Methods
  private boolean isClimbAtPosition(double position) {
    return ((position - ClimberConstants.kClimbDeadband) <= this.getClimbPosition()) 
    && ((position + ClimberConstants.kClimbDeadband) >= this.getClimbPosition());
  }

  private boolean isClimberReadyToClimb() {
    return this.isClimbAtPosition(ClimberConstants.kClimbExtend); /*&&
    this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbExtend))*/
  }

  private boolean isClimberHome() {
    return this.isClimbAtPosition(ClimberConstants.kClimbHome);
  }

  /**
   * Should return true if the supply limit has been exceeded.
   * @return
   */
  private boolean climberCurrentLimitTripped() {  //Modified to look at the current itself rather than relying on the fault flag
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

  private boolean isDSAttachLatched() {
        if (DriverStation.getAlliance().isPresent()) {
          hasDSAttachLatched=true;
        } else {
          hasDSAttachLatched=false;
        }
        return hasDSAttachLatched;
  }

//=======================================================
//=====================Public Methods====================
//=======================================================
//=================Public Climb Climb Methods=============

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

  public Command climbStop() {
    return run(
      () -> {this.climberStop();}
    ).withName("climbStop");
  }

  //=======================Triggers======================
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
