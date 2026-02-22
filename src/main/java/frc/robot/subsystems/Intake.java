// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the run/stop of the intake rollers

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.IntakeConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public class Intake extends SubsystemBase {
  //Variables
  public TalonFX intakeMotor;  //Motor type definition
  
  public VelocityVoltage intakeMotorMode; //Motor control type definition

  IntakeConfig intakeConfig; //Create an object of type IntakeConfig

  public String intakeState = "stopped";

  public DutyCycleOut intakeDutyCycle;

  //Constructor
  public Intake(IntakeConfig config) {

    intakeMotor = new TalonFX(IntakeConfig.kIntakeCANID, "canivore");  //Create the intake motor for this subsystem
    intakeMotorMode = new VelocityVoltage(0);  //Set the motor's control mode

    this.configureMechanism(intakeMotor, config.intakeConfig);

    intakeDutyCycle = new DutyCycleOut(0);

    SmartDashboard.putData("Intake", this);

  }

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

  //====================================================
  //=============Private Methods========================
  //====================================================

  private void intake(){
    if (MotorEnableConstants.kIntakeMotorEnabled) {
      if (intakeState == "outtaking") {
        intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kStopSpeed));
        intakeState = "stopped";
      } else if (intakeState == "stopped") {
        intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kIntakeSpeed));
        intakeState = "intaking";
      }
    }
  }
  
  private void outtake(){
    if (MotorEnableConstants.kIntakeMotorEnabled) {
      if (intakeState == "intaking") {
        intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kStopSpeed));
        intakeState = "stopped";
      } else if (intakeState == "stopped") {
        intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kOuttakeSpeed));
        intakeState = "outtaking";
      }
    }
  }

  private void stop(){
    if (MotorEnableConstants.kIntakeMotorEnabled) {
      intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kStopSpeed));
    }
  }

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

  //=====================================================
  //=============Public Methods==========================
  //=====================================================
  
  public Command intakeSuck() {
    return run(
      () -> {this.intake();}
    ).withName("intakeSuck");
  }

  public Command intakeSpit() {
    return run(
      () -> {this.outtake();}
    ).withName("intakeSpit");
  }

    public Command intakeStop() {
    return run(
      () -> {this.stop();}
    ).withName("intakeStop");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Command", this::getCurrentCommandName, null);
    builder.addStringProperty("Intake State", () -> {return this.intakeState;}, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Current Intake Command", getCurrentCommandName());
    DogLog.log("Intake Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Actual Intake State", intakeState);
    DogLog.log("Actual Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
