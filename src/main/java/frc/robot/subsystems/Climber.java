// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the climber

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ClimberConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
//==================Variables=======================
  public TalonFX liftClimbMotor;  //Motor type definition
  public TalonFX rotateClimb1Motor;  //Motor type definition
  public TalonFX rotateClimb2Motor;  //Motor type definition
  public PWMVictorSPX liftHookReleasePin; //Motor type definition
  public PWMVictorSPX rollerHookReleasePin; //Motor type definition

  public PositionVoltage liftClimbMotorMode; //Motor control type definition
  public PositionVoltage rotateClimb1MotorMode; //Motor control type definition
  public PositionVoltage rotateClimb2MotorMode; //Motor control type definition

  public DutyCycleOut dutyCycleOut; //Motor Control type definition

  CANcoder hookRotateEncoder;
  CANcoder liftEncoder;

  ClimberConfig climberConfig; //Create an object of type climber config to use to configure motors

  //===============Constructor======================
  public Climber(ClimberConfig config) {
    rotateClimb1Motor = new TalonFX(ClimberConfig.kRotateClimb1MotorCANID, "canivore");  //Create a motor for this subsystem
    rotateClimb1MotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(rotateClimb1Motor, config.rotateClimb1MotorConfig);

    rotateClimb2Motor = new TalonFX(ClimberConfig.kRotateClimb2MotorCANID, "canivore");  //Create a motor for this subsystem
    rotateClimb2MotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(rotateClimb2Motor, config.rotateClimb2MotorConfig);

    liftClimbMotor = new TalonFX(ClimberConfig.kLiftClimbMotorCANID, "canivore");  //Create a motor for this subsystem
    liftClimbMotorMode = new PositionVoltage(0);  //Set the motor's control mode
    this.configureMechanism(liftClimbMotor, config.liftClimbMotorConfig);

    //hookRotateEncoder = new CANcoder(ClimberConfig.kHookRotateEncoderCANID,"canivore");
    //this.configureCANcoder(hookRotateEncoder,config.hookRotateCANcoderConfig);

    //liftEncoder = new CANcoder(ClimberConfig.kLiftEncoderCANID,"canivore");
    //this.configureCANcoder(liftEncoder,config.liftCANcoderConfig);

    liftHookReleasePin = new PWMVictorSPX(0);
    rollerHookReleasePin = new PWMVictorSPX(1);

    dutyCycleOut = new DutyCycleOut(0.0);

    this.liftClimbMotor.setPosition(0);

  }

  //===================Configuration=====================
  public void configureCANcoder(CANcoder cancoder, CANcoderConfiguration config){       

        //Start Configuring Climber Motor
        StatusCode cancoderRotateStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            cancoderRotateStatus = cancoder.getConfigurator().apply(config);
            if (cancoderRotateStatus.isOK()) break;
        }

        if (!cancoderRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + cancoderRotateStatus.toString());
        }
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

//=======================================================
//====================Private Methods====================
//=======================================================
//===============Private Set/Goto Methods================
  private void goToPositionLiftClimb(double position) {
    if (MotorEnableConstants.kLiftClimbMotorEnabled) {
      if (position <= ClimberConstants.kLiftClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kLiftClimbSafeRetract) { //Check that Value is above retracted distance
        rotateClimb1Motor.setControl(liftClimbMotorMode.withPosition(position));
      }
    }
  }

  private void goToPositionRotateClimb(double position) {
    if (MotorEnableConstants.kRotateClimb1MotorEnabled) {
      if (position <= ClimberConstants.kRotateClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kRotateClimbSafeRetract) { //Check that Value is above retracted distance
        rotateClimb1Motor.setControl(rotateClimb1MotorMode.withPosition(position));
      }
    }
        if (MotorEnableConstants.kRotateClimb2MotorEnabled) {
      if (position <= ClimberConstants.kRotateClimbSafeExtend //Check that Value is below extended distance 
      && position >= ClimberConstants.kRotateClimbSafeRetract) { //Check that Value is above retracted distance
        rotateClimb2Motor.setControl(rotateClimb2MotorMode.withPosition(position));
      }
    }
  }

  private void retractClimberPins() {
    liftHookReleasePin.set(-1);
    rollerHookReleasePin.set(-1);
  }

  private void extendClimberPins() {
    liftHookReleasePin.set(1);
    rollerHookReleasePin.set(1);
  }

  private void rotateClimberStop(){
    rotateClimb1Motor.setControl(dutyCycleOut.withOutput(0.0));
    rotateClimb2Motor.setControl(dutyCycleOut.withOutput(0.0));
  }

  private void liftClimberStop(){
    liftClimbMotor.setControl(dutyCycleOut.withOutput(0.0));
  }

  //=====================Private Get Methods==================================
  private double getRotateClimb1Position() {
    return rotateClimb1Motor.getPosition().getValueAsDouble();
  }

  private double getRotateClimb2Position() {
    return rotateClimb2Motor.getPosition().getValueAsDouble();
  }

  private double getLiftClimbPosition() {
    return liftClimbMotor.getPosition().getValueAsDouble();
  }

  //=====================Private Trigger Methods
  private boolean isLiftClimbAtPosition(double position) {
    return ((position - ClimberConstants.kLiftClimbDeadband) <= this.getLiftClimbPosition()) 
    && ((position + ClimberConstants.kLiftClimbDeadband) >= this.getLiftClimbPosition());
  }

  private boolean isRotateClimbAtPosition(double position) {
    return ((position - ClimberConstants.kRotateClimbDeadband) <= this.getRotateClimb1Position()) 
    && ((position + ClimberConstants.kRotateClimbDeadband) >= this.getRotateClimb1Position()) 
    && ((position - ClimberConstants.kRotateClimbDeadband) <= this.getRotateClimb2Position()) 
    && ((position + ClimberConstants.kRotateClimbDeadband) >= this.getRotateClimb2Position());
  }

  private boolean isClimberReadyToClimb() {
    return (this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbExtend) &&
    this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbExtend));
  }

  private boolean isClimberHome() {
    return (this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHome) &&
    this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbHome));
  }
//=======================================================
//=====================Public Methods====================
//=======================================================
  //=================Public Lift Climb Methods================
  public Command liftClimbExtend() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbExtend);}
    ).until(isLiftClimbExtended);
  }

  public Command liftClimbHandoff() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbHandOff);}
    ).until(isLiftClimbHandedOff);
  }

  public Command liftClimbRetract() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbRetract);}
    ).until(isLiftClimbRetracted);
  }

  public Command liftClimbHome() {
    return run(
      () -> {this.goToPositionLiftClimb(ClimberConstants.kLiftClimbHome);}
    ).until(isLiftClimbHome);
  }

  public Command liftClimbStop() {
    return run(
      () -> {this.liftClimberStop();}
    );
  }

  //===============Rotate Climb Commands - Simplified==========================
  public Command rotateClimbHandoff() {
    return run(
      () -> {this.goToPositionRotateClimb(ClimberConstants.kRotateClimbRetract);}
    ).until(isRotateClimbHandedOff);
  }

  public Command rotateClimbHome() {
    return run(
      () -> {this.goToPositionRotateClimb(ClimberConstants.kRotateClimbHome);}
    ).until(isRotateClimbHome);
  }

  public Command rotateClimbExtend() {
    return run(
      () -> {this.goToPositionRotateClimb(ClimberConstants.kRotateClimbExtend);}
    ).until(isRotateClimbExtended);
  }
  
  public Command rotateClimbRetract() {
    return run(
      () -> {this.goToPositionRotateClimb(ClimberConstants.kRotateClimbRetract);}
    ).until(isRotateClimbRetracted);
  }

  public Command rotateClimbStop() {
    return run(
      () -> {this.rotateClimberStop();}
    );
  }

  //===================Pin Commands========================
  public Command retractPins() {
    return run(
      () -> {this.retractClimberPins();});
  }

    public Command extendPins() {
    return run(
      () -> {this.extendClimberPins();});
  }

  //=======================Triggers======================
  public Trigger isLiftClimbExtended = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbExtend);});
  public Trigger isLiftClimbHandedOff = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHandOff);});
  public Trigger isLiftClimbRetracted = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbRetract);});
  public Trigger isLiftClimbHome = new Trigger(() -> {return this.isLiftClimbAtPosition(ClimberConstants.kLiftClimbHome);});

  public Trigger isRotateClimbExtended = new Trigger(() -> {return this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbExtend);});
  public Trigger isRotateClimbHandedOff = new Trigger(() -> {return this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbHandOff);});
  public Trigger isRotateClimbRetracted = new Trigger(() -> {return this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbRetract);});
  public Trigger isRotateClimbHome = new Trigger(() -> {return this.isRotateClimbAtPosition(ClimberConstants.kRotateClimbHome);});

  public Trigger isClimberReadyToClimb = new Trigger(() -> {return this.isClimberReadyToClimb();});
  public Trigger isClimberHome = new Trigger (() -> {return this.isClimberHome();});

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
