// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the in and out motion of the hopper

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.HopperConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.HopperConstants;

public class Hopper extends SubsystemBase {

//======================Variables==========================
  public TalonFX hopperMotor;  //Motor type definition
  
  public PositionVoltage hopperMotorMode; //Motor control type definition

  HopperConfig hopperConfig; //Create an object of type HopperConfig

  public Hopper(HopperConfig config) {
    hopperMotor = new TalonFX(HopperConfig.kHopperExtendCANID, "canivore");  //Create a motor for this subsystem
    hopperMotorMode = new PositionVoltage(0);  //Set the motor's control mode

    this.configureMechanism(hopperMotor, config.hopperConfig);

    this.hopperMotor.setPosition(0);
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

//=================Public Methods=========================
  public Command hopperExtend() {
    return run(
      () -> {this.goToPosition(HopperConstants.kHopperExtend);}
    ).until(isHopperExtended);
  }

  public Command hopperRetract() {
    return run(
      () -> {this.goToPosition(HopperConstants.kHopperRetract);}
    ).until(isHopperRetracted);
  }

//================================Triggers================================  
  public Trigger isHopperExtended= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperExtend);});
  public Trigger isHopperRetracted= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperRetract);});

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
