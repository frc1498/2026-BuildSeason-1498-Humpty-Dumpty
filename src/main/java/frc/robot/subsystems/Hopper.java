// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the in and out motion of the hopper

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.HopperConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;
import frc.robot.constants.HopperConstants;
//import dev.doglog.DogLog;

public class Hopper extends SubsystemBase {

//======================Variables==========================
  public TalonFX hopperMotor;  //Motor type definition
  
  public PositionTorqueCurrentFOC hopperMotorMode; //Motor control type definition
  public DutyCycleOut dutyCycleOut;
  private double desiredPosition;

  HopperConfig hopperConfig; //Create an object of type HopperConfig
  public boolean isHopperVelocityLimitLatched = false;

  public Hopper(HopperConfig config) {
    hopperMotor = new TalonFX(HopperConfig.kHopperExtendCANID, "canivore");  //Create a motor for this subsystem
    hopperMotorMode = new PositionTorqueCurrentFOC(0);  //Set the motor's control mode

    this.configureMechanism(hopperMotor, config.hopperConfig);

    this.hopperMotor.setPosition(0);

    //SmartDashboard.putData("Hopper", this);
  }

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

//================Private Methods=========================
  private void goToPosition(double position) {
    desiredPosition = position;
    if (MotorEnableConstants.kHopperMotorEnabled) {
      if (position <= HopperConstants.kHopperSafeExtend //Check that Value is below extended distance 
      && position >= HopperConstants.kHopperSafeRetract) { //Check that Value is above retracted distance
        hopperMotor.setControl(hopperMotorMode.withPosition(position));
      }
    }
  }

  private double getHopperPosition() {
    return hopperMotor.getPosition().getValueAsDouble();
  }

  private boolean isHopperAtPosition(double position) {
    return ((position - HopperConstants.kDeadband) <= this.getHopperPosition()) 
    && ((position + HopperConstants.kDeadband) >= this.getHopperPosition());
  }

  private String getCurrentCommandName() {
      if (this.getCurrentCommand() == null) {
          return "No Command";
      }
      else {
          return this.getCurrentCommand().getName();
      }
  }

  private void agitateHopper(){
    if (isHopperAtPosition(HopperConstants.kHopperExtend)){
      this.goToPosition(HopperConstants.kHopperMidPosition);
    } else if (isHopperAtPosition(HopperConstants.kHopperMidPosition)){
      this.goToPosition(HopperConstants.kHopperExtend);
    } else {
    this.goToPosition(HopperConstants.kHopperExtend);
    }
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
        DogLog.log("Desired Hopper Position", desiredPosition);
        DogLog.log("Actual Hopper Position", hopperMotor.getPosition().getValueAsDouble());
        DogLog.log("Hopper Current", hopperMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Current Hopper Command", getCurrentCommandName());
        break;
      default:
        break;
    }
  }

  private boolean hopperCurrentLimitTripped() {  //Modified to look at the current itself rather than relying on the fault flag
    return (this.hopperMotor.getStatorCurrent().getValueAsDouble() > 20);
  }

//=================Public Methods=========================
  public Command hopperExtend() {
    return run(
      () -> {this.goToPosition(HopperConstants.kHopperExtend);}
    ).until(isHopperExtended).withName("hopperExtend");
  }

  public Command hopperRetract() {
    return run(
      () -> {this.goToPosition(HopperConstants.kHopperRetract);}
    ).until(isHopperRetracted).withName("hopperRetract");
  }

  public Command hopperMidPosition() {
    return run(
      () -> {this.goToPosition(HopperConstants.kHopperMidPosition);}
    ).until(isHopperMidpoint).withName("hopperMidpoint");
  }

   public Command zeroRoutine() {  //This routine goes maximum out and sets that position
    return run(
      () -> {
        this.configureMechanism(this.hopperMotor, this.hopperConfig.hopperZeroConfig);
        this.hopperMotor.setControl(this.dutyCycleOut.withOutput(0.25));
      }
    ).until(this.isHopperCurrentLimitTripped)
    .andThen(
      runOnce(
        () -> {
          this.hopperMotor.setControl(this.dutyCycleOut.withOutput(0));
          this.hopperMotor.setPosition(0);
          if (this.hopperMotor.getStatorCurrent().getValueAsDouble() > 20) {
            this.configureMechanism(this.hopperMotor, this.hopperConfig.hopperConfig);
          }
        }
      )
    ).withName("zeroRoutine");
  }

  public Command agitate() {
    return (this.hopperExtend()
      .andThen(Commands.waitSeconds(0.5))
      .andThen(this.hopperMidPosition())
      .andThen(Commands.waitSeconds(0.5))
      ).repeatedly().withName("agitate");
    //return runOnce(() -> {this.agitateHopper();}).andThen(
    //  Commands.waitSeconds(0.5));
  }

  public Command setHopperZero() {
    return run(() -> {this.hopperMotor.setPosition(0.0);});
  }

//================================Triggers================================  
  public Trigger isHopperExtended= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperExtend);});
  public Trigger isHopperRetracted= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperRetract);});
  public Trigger isHopperMidpoint = new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperMidPosition);});
  public Trigger isHopperCurrentLimitTripped = new Trigger(this::hopperCurrentLimitTripped);

  @Override
  public void initSendable(SendableBuilder builder) {
    //builder.addStringProperty("Command", this::getCurrentCommandName, null);
    //builder.addDoubleProperty("Desired Hopper Position", () -> {return this.desiredPosition;}, null);
    //builder.addDoubleProperty("Actual Hopper Position", this::getHopperPosition, null);
    //builder.addBooleanProperty("Hopper at Extend", this.isHopperExtended, null);
    //builder.addBooleanProperty("Hopper at Retract", this.isHopperRetracted, null);
    //builder.addBooleanProperty("Hoppet at Midpoint", this.isHopperMidpoint,null);
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
