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
import frc.robot.constants.MotorEnableConstants.TelemetryLevel;
import frc.robot.constants.ClimberConstants;

//For zeroing - 5 A Supply Current, 50 A Stator Current, 3 V Output

public class Climber extends SubsystemBase {
//==================Variables=======================
  public TalonFX liftClimbMotor;  //Motor type definition

  public PositionVoltage liftClimbMotorMode; //Motor control type definition

  public DutyCycleOut dutyCycleOut; //Motor Control type definition

  private double desiredLiftMotorPosition;
  private boolean isLiftClimberCurrentLimitLatched=false;

  ClimberConfig climberConfig; //Create an object of type climber config to use to configure motors

  //===============Constructor======================
  public Climber(ClimberConfig config) {
    liftClimbMotor = new TalonFX(ClimberConfig.kLiftClimbMotorCANID, "canivore");  //Create a motor for this subsystem
    liftClimbMotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(liftClimbMotor, config.liftClimbMotorConfig);

    this.dutyCycleOut = new DutyCycleOut(0.0);

    this.liftClimbMotor.setPosition(0);

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
  private void goToPositionLiftClimb(double position) {
    desiredLiftMotorPosition = position;
    if (MotorEnableConstants.kLiftClimbMotorEnabled) {
      if (position <= ClimberConstants.kLiftClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kLiftClimbSafeRetract) { //Check that Value is above retracted distance
        liftClimbMotor.setControl(liftClimbMotorMode.withPosition(position));
      }
    }
  }

  private void liftClimberStop(){
    liftClimbMotor.setControl(dutyCycleOut.withOutput(0.0));
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

  private double getLiftClimbPosition() {
    return liftClimbMotor.getPosition().getValueAsDouble();
  }

  //=====================Private Trigger Methods
  private boolean isLiftClimbAtPosition(double position) {
    return ((position - ClimberConstants.kLiftClimbDeadband) <= this.getLiftClimbPosition()) 
    && ((position + ClimberConstants.kLiftClimbDeadband) >= this.getLiftClimbPosition());
  }

  private boolean isClimberReadyToClimb() {
    return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbExtend); /*&&
    this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbExtend))*/
  }

  private boolean isClimberHome() {
    return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHome);
  }

  /**
   * Should return true if the supply limit has been exceeded.
   * @return
   */
  private boolean liftClimberCurrentLimitTripped() {  //Modified to look at the current itself rather than relying on the fault flag
    return (this.liftClimbMotor.getStatorCurrent().getValueAsDouble() > 4.5);
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
        DogLog.log("Desired Lift Motor Position", this.desiredLiftMotorPosition);
        DogLog.log("Actual Lift Motor Position", this.getLiftClimbPosition());
        DogLog.log("Actual Lift Motor Current", this.liftClimbMotor.getSupplyCurrent().getValueAsDouble());
        break;
      default:
        break;
    }


    /* These motors don't exist at the moment, but I want to keep the calls commented out until a decision is made on the level 3 climb
    DogLog.log("Desired Rotate1 Motor Position", desiredRotate1MotorPosition);
    DogLog.log("Actual Rotate1 Motor Position", getRotateClimb1Position());
    DogLog.log("Actual Rotate1 Motor Current", rotateClimb1Motor.getSupplyCurrent().getValueAsDouble());

    DogLog.log("Desired Rotate2 Motor Position", desiredRotate2MotorPosition);
    DogLog.log("Actual Rotate2 Motor Position", getRotateClimb2Position());
    DogLog.log("Actual Rotate2 Motor Current", rotateClimb2Motor.getSupplyCurrent().getValueAsDouble());
    */
  }

//=======================================================
//=====================Public Methods====================
//=======================================================
//=================Public Lift Climb Methods=============

  /**
   * A zeroing routine for the climber.  This should drive the motor down until the supply current limit is tripped (or stalled).
   * @return
   */
  public Command zeroRoutine() {
    return run(
      () -> {
        isLiftClimberCurrentLimitLatched=false;
        this.configureMechanism(this.liftClimbMotor, this.climberConfig.liftClimbMotorZeroConfig);
        this.liftClimbMotor.setControl(this.dutyCycleOut.withOutput(0.25));
      }
    ).until(this.isLiftClimberCurrentLimitTripped)
    .andThen(
      runOnce(
        () -> {
          this.configureMechanism(this.liftClimbMotor, this.climberConfig.liftClimbMotorConfig);
          this.liftClimbMotor.setPosition(0);
          if (this.liftClimbMotor.getStatorCurrent().getValueAsDouble() > 4.5) {isLiftClimberCurrentLimitLatched=true;}
        }
      )
    ).withName("zeroRoutine");
  }

  public Command liftClimbExtend() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbExtend);}
    ).until(isLiftClimbExtended).withName("liftClimbExtend");
  }

  public Command liftClimbHandoff() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbHandOff);}
    ).until(isLiftClimbHandedOff).withName("liftClimbHandoff");
  }

  public Command liftClimbRetract() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbRetract);}
    ).until(isLiftClimbRetracted).withName("liftClimbRetract");
  }

  public Command liftClimbHome() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbHome);}
    ).until(isLiftClimbHome).withName("liftClimbHome");
  }

  public Command liftClimbStop() {
    return run(
      () -> {this.liftClimberStop();}
    ).withName("liftClimbStop");
  }

  //=======================Triggers======================
  public Trigger isLiftClimbExtended = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbExtend);});
  public Trigger isLiftClimbHandedOff = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHandOff);});
  public Trigger isLiftClimbRetracted = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbRetract);});
  public Trigger isLiftClimbHome = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHome);});
  public Trigger isLiftClimberCurrentLimitTripped = new Trigger(this::liftClimberCurrentLimitTripped);
  public Trigger isClimberReadyToClimb = new Trigger(() -> {return this.isClimberReadyToClimb();});
  public Trigger isClimberHome = new Trigger (() -> {return this.isClimberHome();});

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Command", this::getCurrentCommandName, null);
    builder.addDoubleProperty("Desired Climb Position", () -> {return this.desiredLiftMotorPosition;}, null);
    builder.addDoubleProperty("Current Climb Position", this::getLiftClimbPosition, null);
    builder.addBooleanProperty("Climber Latched", () -> {return isLiftClimberCurrentLimitLatched;}, null);
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
