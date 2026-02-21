// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the run/stop of the intake rollers

/* TO DO: Doglog
 * Desired Intake Velocity
 * Actual Intake Velocity
 * Actual Intake Current
 * Desired Intake State
 * Actual Intake State
 * Current Subsystem Command
 */

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.IntakeConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Intake extends SubsystemBase {
  //Variables
  public TalonFX intakeMotor;  //Motor type definition
  
  public VelocityVoltage intakeMotorMode; //Motor control type definition

  IntakeConfig intakeConfig; //Create an object of type IntakeConfig

  public String intakeState="stopped";

  public DutyCycleOut intakeDutyCycle;

  //Constructor
  public Intake(IntakeConfig config) {

    intakeMotor = new TalonFX(IntakeConfig.kIntakeCANID, "canivore");  //Create the intake motor for this subsystem
    intakeMotorMode = new VelocityVoltage(0);  //Set the motor's control mode

    this.configureMechanism(intakeMotor, config.intakeConfig);

    intakeDutyCycle = new DutyCycleOut(0);

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
      if (intakeState=="outtaking") {
        intakeMotor.setControl(intakeDutyCycle.withOutput(IntakeConstants.kIntakeDutyCycleStop));
        intakeState="stopped";
      } else if (intakeState == "stopped") {
        intakeMotor.setControl(intakeMotorMode.withVelocity(IntakeConstants.kIntakeSpeed));
        intakeState="intaking";
      }
    }
  }
  
  private void outtake(){
    if (MotorEnableConstants.kIntakeMotorEnabled) {
      if (intakeState == "intaking") {
        intakeMotor.setControl(intakeMotorMode.withVelocity(IntakeConstants.kStopSpeed));
        intakeState = "stopped";
      } else if (intakeState=="stopped") {
        intakeMotor.setControl(intakeMotorMode.withVelocity(IntakeConstants.kOuttakeSpeed));
        intakeState="outtaking";
      }
    }
  }

  private void stop(){
    if (MotorEnableConstants.kIntakeMotorEnabled) {
      intakeMotor.setControl(intakeMotorMode.withVelocity(IntakeConstants.kStopSpeed));
    }
  }
  //=====================================================
  //=============Public Methods==========================
  //=====================================================
  
  public Command intakeSuck() {
    return run(
      () -> {this.intake();}
    );
  }

  public Command intakeSpit() {
    return run(
      () -> {this.outtake();}
    );
  }

    public Command intakeStop() {
    return run(
      () -> {this.stop();}
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
