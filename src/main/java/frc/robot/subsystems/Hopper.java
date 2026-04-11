// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the in and out motion of the hopper

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.HopperConfig;
import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;
import frc.robot.constants.HopperConstants;
//import dev.doglog.DogLog;

public class Hopper extends SubsystemBase {

  /* Variables */
  public TalonFX hopperMotor;  //Motor type definition
  
  public PositionVoltage hopperMotorMode; //Motor control type definition
  public DutyCycleOut dutyCycleOut;
  private double desiredPosition;

  private HopperConfig hopperConfig; //Create an object of type HopperConfig
  public boolean isHopperVelocityLimitLatched = false;

  // Fall back to a default of no telemetry.
  private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;
  
  /**
   * The constructor for the hopper subsystem.
   * @param config - The configuration for the motors in the hopper subsystem.
   * @param telemetryLevel - The level of telemetry to enable for the subsystem.  Currently FULL, LIMITED, or NONE.
   */
  public Hopper(HopperConfig config, MotorEnableConstants.TelemetryLevel telemetryLevel) {

    this.telemetryLevel = telemetryLevel;
    this.hopperConfig = config;

    this.hopperMotor = new TalonFX(HopperConfig.kHopperExtendCANID, "canivore");  //Create a motor for this subsystem
    this.hopperMotorMode = new PositionVoltage(0);  //Set the motor's control mode

    this.configureMechanism(this.hopperMotor, this.hopperConfig.hopperConfig);

    this.hopperMotor.setPosition(0);

    SmartDashboard.putData("Hopper", this);
  }

  /**
   * Apply the configuration to the motor.  This will attempt to re-apply the configuration if unsuccessful, up to 5 times.
   * @param mechanism - The TalonFX object (motor) to apply the configuration to.
   * @param config - The set of configurations to apply.
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

  /**
   * Set the motor to the desired position.
   * The setpoint will be ignored if the motor is disabled, or the setpoint is outside of the safety limits.
   * @param position - The desired position of the motor, in rotations.
   */
  private void goToPosition(double position) {
    desiredPosition = position;
    if (MotorEnableConstants.kHopperMotorEnabled) {
      if (position <= HopperConstants.kHopperSafeExtend //Check that Value is below extended distance 
      && position >= HopperConstants.kHopperSafeRetract) { //Check that Value is above retracted distance
        hopperMotor.setControl(hopperMotorMode.withPosition(position));
      }
    }
  }

  /**
   * Returns the current position of the hopper motor, in rotations.
   * Please note that this is different from the position of the mechanism.
   * @return The current position of the hopper motor, in rotations.
   */
  private double getHopperPosition() {
    return hopperMotor.getPosition().getValueAsDouble();
  }

  /**
   * Check if the current hopper position is within the allowable deadband of the parameter.
   * @param position - The position to check the current hopper position against.
   * @return True if the hopper position is within the allowable deadband of the position.
   */
  private boolean isHopperAtPosition(double position) {
    return ((position - HopperConstants.kDeadband) <= this.getHopperPosition()) 
    && ((position + HopperConstants.kDeadband) >= this.getHopperPosition());
  }

  /**
   * Returns a string of the name of the currently running command.
   * If no command is running, return "No Command".
   * @return A string with the name of the currently running command.
   */
  private String getCurrentCommandName() {
      if (this.getCurrentCommand() == null) {
          return "No Command";
      }
      else {
          return this.getCurrentCommand().getName();
      }
  }

  /**
   * A routine to agitate the hopper to move balls into the kickup.
   * The hopper moves to the midpoint if it is fully extended, and fully extends if it is at the midpoint.
   */
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

  /**
   * Check the current draw of the hopper against a preset limit of 20 A.
   * Use to determine if the hopper is stalling against the hard stop for a zeroing routine.
   * @return True if the stator current of the hopper motor is greater than 20 A.
   */
  private boolean hopperCurrentLimitTripped() {  //Modified to look at the current itself rather than relying on the fault flag
    return (this.hopperMotor.getStatorCurrent().getValueAsDouble() > 20);
  }

  /* Public Methods */

  /**
   * A factory command that sets the position of the hopper motor.
   * @param position - The position setpoint of the hopper motor, in rotations.
   * @return A command that runs the {@code goToPosition} method.
   */
  private Command moveHopper(DoubleSupplier position) {return run(() -> {this.goToPosition(position.getAsDouble());}).withName("moveHopper");}

  public Command newHopperExtend() {return this.moveHopper(() -> {return HopperConstants.kHopperExtend;}).until(isHopperExtended).withName("hopperExtend");}
  public Command newHopperRetract() {return this.moveHopper(() -> {return HopperConstants.kHopperRetract;}).until(isHopperRetracted).withName("hopperRetract");}
  public Command newHopperMidpoint() {return this.moveHopper(() -> {return HopperConstants.kHopperMidPosition;}).until(isHopperMidpoint).withName("hopperMidpoint");}
  public Command hopperHold() {return this.moveHopper(() -> {return this.getHopperPosition();}).withName("hopperHold");}

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

  /**
   * A zeroing routine for the hopper.  This should drive the motor down until the supply current limit is tripped (or stalled).
   * @return A command the zeros the hopper.
   */
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

  /**
   * A command that agitates the hopper by moving between the fully extended position and midpoint position.
   * Hopefully this dislodges any stuck balls in the robot.
   * @return A command that agitates the hopper.
   */
  public Command agitate() {
    return (this.hopperMidPosition()
      .andThen(Commands.waitSeconds(.5))
      .andThen(this.hopperExtend())
      .andThen(Commands.waitSeconds(.75))
    ).repeatedly().withName("agitate");
      
    //return runOnce(() -> {this.agitateHopper();}).andThen(
    //  Commands.waitSeconds(0.5));
  }

  /**
   * Sets the current hopper position to zero.
   * @return A command that sets the hopper motor position to 0.0.
   */
  public Command setHopperZero() {
    return run(() -> {this.hopperMotor.setPosition(0.0);});
  }

  /* Triggers */
  public Trigger isHopperExtended= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperExtend);});
  public Trigger isHopperRetracted= new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperRetract);});
  public Trigger isHopperMidpoint = new Trigger(() -> {return this.isHopperAtPosition(HopperConstants.kHopperMidPosition);});
  public Trigger isHopperCurrentLimitTripped = new Trigger(this::hopperCurrentLimitTripped);

  @Override
  public void initSendable(SendableBuilder builder) {
    // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
    // That can be used to 'gate' values to log without lines of identical code.
    switch (this.telemetryLevel) {
      case FULL:
        builder.addDoubleProperty("Desired Hopper Position", () -> {return this.desiredPosition;}, null);
        builder.addDoubleProperty("Actual Hopper Position", this::getHopperPosition, null);
        builder.addBooleanProperty("Hopper at Extend", this.isHopperExtended, null);
        builder.addBooleanProperty("Hopper at Retract", this.isHopperRetracted, null);
        builder.addBooleanProperty("Hopper at Midpoint", this.isHopperMidpoint,null);
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
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
