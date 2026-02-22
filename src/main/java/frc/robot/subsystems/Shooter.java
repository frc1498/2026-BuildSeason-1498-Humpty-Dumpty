// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.ShotCalculation;
import frc.robot.config.ShooterConfig;
import frc.robot.sim.ShooterSim;

import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterFault;

/**
 * The shooter subsystem.  Contains the flywheel, turret, hood adjustment, spindexer, and ball kickup.
 */
public class Shooter extends SubsystemBase {

/*==================Variables=======================*/

  private TalonFX shooterLeftMotor;   // Motor type definition
  private TalonFX shooterRightMotor;   // Motor type definition
  private TalonFX spindexerMotor;  // Motor type definition
  private TalonFX turretMotor;     // Motor type definition
  private TalonFX kickupMotor;     // Motor type definition
  private TalonFX hoodMotor;       // Motor type definition

  private VelocityTorqueCurrentFOC shooterMotorMode;   // Motor control type definition
  private VelocityVoltage spindexerMotorMode; // Motor control type definition
  private VelocityTorqueCurrentFOC kickupMotorMode;    // Motor control type definition

  private PositionTorqueCurrentFOC turretMotorMode; // Motor control type definition
  private PositionTorqueCurrentFOC hoodMotorMode;   // Motor control type definition

  private ShooterConfig shooterConfig;  // Create an object of type shooter subsystem config used to configure motors

  public Pose2d currentTarget;
  public Pose2d hubTarget;

  private double desiredHoodPosition;
  private double desiredTurretPosition;
  private double desiredShooterVelocity;
  private double desiredKickupVelocity;
  private double desiredSpindexerVelocity;

  private boolean hoodAtPosition;
  private boolean turretAtPosition;
  private boolean shooterAtVelocity;
  private boolean kickupAtVelocity;
  private boolean spindexerAtVelocity;
  private boolean readyToFire;

  private Supplier<SwerveDriveState> swerveStateSupplier;
  private double distanceToTarget;
  private double distanceToVirtualTarget;

  private double virtualHoodAngle;
  private double virtualFlywheelVelocity;
  private double virtualTurretAngle;

  private double currentHoodAngle;
  private double currentTurretAngle;
  private double currentFlywheelVelocity;

  private double tuningHoodAngle;
  private double tuningTurretAngle;
  private double tuningFlywheelVelocity;

  private ShooterSim sim;
  private TalonFXSimState shooterLeftMotorSim;
  private TalonFXSimState shooterRightMotorSim;
  private TalonFXSimState spindexerMotorSim;
  private TalonFXSimState kickupMotorSim;
  private TalonFXSimState turretMotorSim;
  private TalonFXSimState hoodMotorSim;
  
  public DutyCycleOut turretDutyCycle;
  public DutyCycleOut shooterDutyCycle;
  public DutyCycleOut kickupDutyCycle;
  public DutyCycleOut spindexerDutyCycle;

  private double simTime;

  boolean turretZeroed;
  boolean requestShoot;

  public Field2d targetingField = new Field2d();

  private enum ShooterState {
    IDLE(0.0, 0.0, true),
    FORWARD(ShooterConstants.kSpindexerIntake, ShooterConstants.kKickupIntake, true),
    REVERSE(ShooterConstants.kSpindexerOuttake, ShooterConstants.kKickupOuttake, false);

    private double spindexerVelocity;
    private double kickupVelocity;
    private boolean direction;

    /**
     * Constructor for the ShooterState enumeration.
     * @param spindexerVelocity - The velocity of the spindexer.
     * @param kickupVelocity - The direction of the kickup motor.
     * @param direction - Represents direction of the motors.  True is forward (intake), false is reverse (outtake).
     */
    ShooterState(double spindexerVelocity, double kickupVelocity, boolean direction) {
      this.spindexerVelocity = spindexerVelocity;
      this.kickupVelocity = kickupVelocity;
      this.direction = direction;
    }

    /**
     * Returns the spindexer velocity for the state.
     * @return - Desired velocity for the spindexer, in rotations per second.
     */
    public double spindexer() {
      return this.spindexerVelocity;
    }

    /**
     * Returns the kickup velocity for the state.
     * @return - Desired velocity for the kickup motor, in rotations per second.
     */
    public double kickup() {
      return this.kickupVelocity;
    }

    /**
     * Returns the motor direction for the state.
     * @return - Motor direction.  True is forward (intake), false is reverse (outtake).
     */
    public boolean direction() {
      return this.direction;
    }
  }

/**
 * Creates a new instance of the shooter subsystem.
 * @param config - The motor configurations for all motors in the subsystem.
 * @param swerveDriveState - A supplier of the current swerve drive state from the drivetrain subsystem.
 */
public Shooter(ShooterConfig config, Supplier<SwerveDriveState> swerveDriveState) {

  this.swerveStateSupplier = swerveDriveState;
  this.shooterConfig = config;

  this.shooterLeftMotor = new TalonFX(ShooterConfig.kShooterLeftMotorCANID, "canivore");    // Create the first shooter motor.
  this.configureMechanism(this.shooterLeftMotor, this.shooterConfig.shooterLeftMotorConfig);

  this.shooterRightMotor = new TalonFX(ShooterConfig.kShooterRightMotorCANID, "canivore");    // Create the second shooter motor.
  this.shooterMotorMode = new VelocityTorqueCurrentFOC(0);                                        // Set the control mode for both shooter motors.
  this.configureMechanism(this.shooterRightMotor, this.shooterConfig.shooterRightMotorConfig);
  
  this.shooterLeftMotor.setControl(new Follower(shooterRightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

  this.spindexerMotor = new TalonFX(ShooterConfig.kSpindexerMotorCANID, "canivore");  // Create the spindexer motor.
  this.spindexerMotorMode = new VelocityVoltage(0);                                      // Set the control mode for the spindexer motor.
  this.configureMechanism(this.spindexerMotor, this.shooterConfig.spindexerMotorConfig);

  this.kickupMotor = new TalonFX(ShooterConfig.kKickupMotorCANID, "canivore");        // Create the kickup motor.
  this.kickupMotorMode = new VelocityTorqueCurrentFOC(0);                                         // Set the control mode for the kickup motor.
  this.configureMechanism(this.kickupMotor, this.shooterConfig.kickupMotorConfig);
   
  this.turretMotor = new TalonFX(ShooterConfig.kTurretMotorCANID, "canivore");        // Create the turret rotate motor.
  this.turretMotorMode = new PositionTorqueCurrentFOC(0);                                         // Set the control mode for the turret motor.
  this.configureMechanism(this.turretMotor, this.shooterConfig.turretMotorConfig);
    
  this.hoodMotor = new TalonFX(ShooterConfig.kHoodMotorCANID, "canivore");            // Create hood adjustment motor.
  this.hoodMotorMode = new PositionTorqueCurrentFOC(0);                                           // Set the contorl mode for the adjustment motor.
  this.configureMechanism(this.hoodMotor, this.shooterConfig.hoodMotorConfig);

  this.shooterLeftMotorSim = this.shooterLeftMotor.getSimState();
  this.shooterRightMotorSim = this.shooterRightMotor.getSimState();
  this.spindexerMotorSim = this.spindexerMotor.getSimState();
  this.kickupMotorSim = this.kickupMotor.getSimState();
  this.turretMotorSim = this.turretMotor.getSimState();
  this.hoodMotorSim = this.hoodMotor.getSimState();

  this.sim = new ShooterSim(
    this.shooterConfig,
    this.hoodMotorSim,
    this.turretMotorSim,
    this.shooterLeftMotorSim,
    this.shooterRightMotorSim,
    this.spindexerMotorSim,
    this.kickupMotorSim
    );

  // Publish subsystem data to SmartDashboard.
  SmartDashboard.putData("Shooter", this);
  SmartDashboard.putData("Shooter/Pose", this.targetingField);
  SmartDashboard.putData("Shooter/Sim", this.sim.getVis());

  turretZeroed = true;
  turretDutyCycle = new DutyCycleOut(0.0);
  shooterDutyCycle = new DutyCycleOut(0.0);
  spindexerDutyCycle = new DutyCycleOut(0.0);
  kickupDutyCycle = new DutyCycleOut(0.0);

  requestShoot = false;

  this.hoodMotor.setPosition(0);
  this.turretMotor.setPosition(0);

  // Initialize the tuning parameters.
  // This is not for PID tuning, but instead for interpolation tuning.
  this.tuningHoodAngle = 0.0;
  this.tuningTurretAngle = 0.0;
  this.tuningFlywheelVelocity = 0.0;

}

/**
 * Apply the configuration to the motor.  This will attempt to re-apply the configuration if unsuccessful, up to 5 times.
 * @param mechanism - The TalonFX object (motor) to apply the configuration to.
 * @param config - The set of configurations to apply.
 */
public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config) {

    // Start Configuring the motor with the supplied configuration.
    StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

    // Attempt to apply the configuration 5 times.  Immediately stop if the configuration was successful.
    for(int i = 0; i < 5; ++i) {

        mechanismStatus = mechanism.getConfigurator().apply(config);

        if (mechanismStatus.isOK()) {break;}
    }

    // If the configuration was still not successful, print an error to the console.
    if (!mechanismStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
    }
  }

  /*===================Private Methods=====================*/

  /**
   * Set the position of turret hood.
   * @param position - The desired hood position, in rotations of the motor.
   */
  private void setHoodPosition(double position) {
    // Always store the setpoint, to track the desired position.
    this.desiredHoodPosition = position;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kHoodMotorEnabled) {
      // Check if the desired hood position is within the allowable safety range.
      // Do not update the hood position if it is out of range.
      if (this.isSetpointWithinSafetyRange(this.desiredHoodPosition, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend)) {
        this.hoodMotor.setControl(this.hoodMotorMode.withPosition(this.desiredHoodPosition));
      } else {
        // Log a fault with DogLog if the desired hood position was out of range.
        DogLog.logFault(ShooterFault.HOOD_SETPOINT_OUT_OF_RANGE);
      }
    }
  }

  /**
   * Return the current position of the turret hood.
   * @return - The current position of the turret hood, in rotations.
   */
  private double getHoodPosition() {
    return this.hoodMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns true if the current setpoint is within the range of minimum and maximum parameters.
   */
  private boolean isSetpointWithinSafetyRange(double currentSetpoint, double minimum, double maximum) {
    return ((currentSetpoint >= minimum) && (currentSetpoint <= maximum));
  }

  /**
   * Updates the hood angle setpoint to use while tuning the shooter interpolation.
   * @param hoodAngle
   */
  private void setTuningHoodPosition(double hoodAngle) {
    this.tuningHoodAngle = hoodAngle;
  }

  /**
   * Set the position of the turret angle.
   * @param position - The desired position of the turret, in rotations.
   */
  private void setTurretPosition(double position) {
    // Always store the setpoint, to track the desired position.
    this.desiredTurretPosition = position;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kTurretMotorEnabled) {
      // Check if the desired turret position is within the allowable safety range.
      // Do not update the turret position if it is out of range.
      if (this.isSetpointWithinSafetyRange(this.desiredTurretPosition, ShooterConstants.kTurretSafeCounterClockwise, ShooterConstants.kTurretSafeClockwise)) {
        this.turretMotor.setControl(this.turretMotorMode.withPosition(this.desiredTurretPosition));
      } else {
        // Log a fault with DogLog if the desired turret position was out of range.
        DogLog.logFault(ShooterFault.TURRET_SETPOINT_OUT_OF_RANGE);        
      }
    }
  }

  /**
   * Return the current position of the turret angle.
   * @return - The current position of the turret, in rotations.
   */
  private double getTurretPosition() {
    return this.turretMotor.getPosition().getValueAsDouble();
  }

  /**
   * Updates the turret angle setpoint to use while tuning the shooter interpolation.
   * @param tuningTurretAngle
   */
  private void setTuningTurretPosition(double tuningTurretAngle) {
    this.tuningTurretAngle = tuningTurretAngle;
  }

  /**
   * Set the velocity of both shooter motors.
   * @param velocity - The desired velocity of the shooter motors, in rotations per second.
   */
  private void setShooterVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredShooterVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kShooterLeftMotorEnabled && MotorEnableConstants.kShooterRightMotorEnabled) {
      //this.shooterLeftMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
      this.shooterRightMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    }
  }

  /**
   * Return the current velocity of the first shooter motor.
   * @return - The current velocity of the first shooter motor, in rotations per second.
   */
  private double getShooterOneVelocity() {
    return this.shooterLeftMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Updates the flywheel velocity setpoint to use while tuning the shooter interpolation.
   * @param tuningFlywheelVelocity
   */
  private void setTuningShooterVelocity(double tuningFlywheelVelocity) {
    this.tuningFlywheelVelocity = tuningFlywheelVelocity;
  }

  /**
   * Set the velocity of the kickup motor.
   * @param velocity - The desired velocity of the kickup motor, in rotations per second.
   */
  private void setKickupVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredKickupVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kKickupMotorEnabled) {
      this.kickupMotor.setControl(this.kickupMotorMode.withVelocity(this.desiredKickupVelocity));
    }
  }

  /**
   * Return the current velocity of the kickup motor.
   * @return - The current velocity of the kickup motor, in rotations per second.
   */
  private double getKickupSpeed() {
    return this.kickupMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Set the velocity of the spindexer motor.
   * @param velocity - The desired velocity of the spindexer motor, in rotations per second.
   */
  private void setSpindexerVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredSpindexerVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kSpindexerMotorEnabled) {
      this.spindexerMotor.setControl(this.spindexerMotorMode.withVelocity(this.desiredSpindexerVelocity));
    }
  }

  /**
   * Return the current velocity of the spindexer motor.
   * @return - The current velocity of the spindexer motor, in rotations per second.
   */
  private double getSpindexerVelocity() {
    return spindexerMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Checks if a current variable is within a deadband to the setpoint.
   * @param setpoint - The setpoint the current variable should be at.
   * @param current - The current variable to check (process variable).
   * @param deadBand - The allowable deadband (+ and -) from the setpoint.
   * @return
   */
  private boolean atSetpoint(double setpoint, double current, double deadBand) {
    return ((current >= (setpoint - deadBand)) && (current <= (setpoint + deadBand)));
  }  


  /**
   * Returns a string of the name of the currently running command.
   * If no command is running, return "No Command".
   * @return
   */
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

  /*====================Public Methods=====================*/

  /**
   * Sets the hood position, turret position, and shooter velocity based on the shoot while move capability.
   * @return
   */
  public Command setShooterOutputs() {
    return runOnce(() -> {
      this.setHoodPosition(this.virtualHoodAngle);
      this.setTurretPosition(this.virtualTurretAngle);
      this.setShooterVelocity(this.virtualFlywheelVelocity);
    }).withName("setShooterOutputs");
  }

  /**
   * Sets the hood position, turret position, and shooter velocity based on the tuning parameters.
   * @return
   */
  public Command setTuningShooterOutputs() {
    return runOnce(() -> {
      this.setHoodPosition(this.tuningHoodAngle);
      this.setTurretPosition(this.tuningTurretAngle);
      this.setShooterVelocity(this.tuningFlywheelVelocity);
    }).withName("setTuningShooterOutputs");
  }

  /**
   * Sets the spindexer and kickup velocity based on the supplied state.
   */
  private Command setSpindexerAndKickup(ShooterState state) {
    return runOnce(() -> {
      this.setKickupVelocity(state.kickup());
      this.setSpindexerVelocity(state.spindexer());
    }).withName("setSpindeerAndKickup: " + state.name());
  }
 
  private void zeroTurret() {
    turretMotor.setControl(turretDutyCycle.withOutput(ShooterConstants.kTurretZeroDutyCycle)); //set a low constant speed
  }
  
  private boolean isTurretAtZero() {
    if (turretMotor.getStatorCurrent().getValueAsDouble() > ShooterConstants.kTurretZeroCurrentLimit) { //Check current draw for hard stop collision
      turretZeroed=true;  //Turret zeroing is complete because we passed the current limit threshold
      turretMotor.setPosition(ShooterConstants.kTurretZeroPosition); //set the encoder position on the motor to whatever it should be
      //Going to have to talk to trevor - how do we go "back" into tracking more turretMotor.setControl(.withOutput();      
    }
    return turretZeroed;  
  }

  private void stopKick(){
    kickupMotor.setControl(kickupDutyCycle.withOutput(ShooterConstants.kKickupZeroDutyCycle));
  }

  private void stopSpindex(){
    spindexerMotor.setControl(spindexerDutyCycle.withOutput(ShooterConstants.kSpindexerZeroDutyCycle));
  }

  private void stopShooting() {
    requestShoot=false;
    //shooterLeftMotor.setControl(shooterDutyCycle.withOutput(ShooterConstants.kShooterZeroDutyCycle));
    shooterRightMotor.setControl(shooterDutyCycle.withOutput(ShooterConstants.kShooterZeroDutyCycle));
  }

  private void startShooting() {
    requestShoot=true;
  }

  private boolean isSpindexerStopped() {
    boolean isSpindexerStopped=false;
    if (spindexerMotor.getVelocity().getValueAsDouble() < ShooterConstants.kSpindexerStoppedVelocityTolerance) {
      isSpindexerStopped=true;
    }
    return isSpindexerStopped;
  }

  private boolean isKickupStopped() {
    boolean isKickupStopped=false;
    if (kickupMotor.getVelocity().getValueAsDouble() < ShooterConstants.kKickupStoppedVelocityTolerance) {
      isKickupStopped=true;
    }
    return isKickupStopped;
  }

  //====================Public Methods=====================
	public Command reZeroTurret() {
	  turretZeroed=false;
	  return run(() -> {this.zeroTurret();})
	    .until(isTurretZeroed);
	}
  
  public Command stopSpindexer() {
    return run(() -> {this.stopSpindex();})
      .until(isSpindexerStopped);
  }

  public Command reverseSpindexer() {
    return run(() -> {this.setSpindexerVelocity(ShooterConstants.kSpindexerOuttake);});
  }

  public Command forwardSpindexer() {
    return run(() -> {this.setSpindexerVelocity(ShooterConstants.kSpindexerIntake);});
  }

  public Command stopKickup() {
    return run(() -> {this.stopKick();})
      .until(isKickupStopped);
  }

  public Command reverseKickup() {
    return run(() -> {this.setKickupVelocity(ShooterConstants.kKickupOuttake);});
  }

  public Command forwardKickup() {
    return run(() -> {this.setKickupVelocity(ShooterConstants.kKickupIntake);});
  }

  public Command setSpindexerAndKickupForward() {return this.setSpindexerAndKickup(ShooterState.FORWARD);};
  public Command setSpindexerAndKickupReverse() {return this.setSpindexerAndKickup(ShooterState.REVERSE);};
  public Command setSpindexerAndKickupIdle() {return this.setSpindexerAndKickup(ShooterState.IDLE);};

  public Command stopShoot() {
    return run(() -> {this.stopShooting();});
  }

  public Command startShoot() {
    return run(() -> {this.startShooting();});
  }

  public Command slowShoot() {
    return run(() -> {this.setShooterVelocity(ShooterConstants.kSlowShoot);}).until(isShooterAtVelocity);
  }

  public Command turretSlowShootPosition() {
    return run(() -> {this.setTurretPosition(ShooterConstants.kTurretSlowShootPosition);});
  }

  public Command turretClimbPosition() {
    return run(() -> {this.setTurretPosition(ShooterConstants.kTurretClimbPosition);});
  }

  //======================Triggers=========================
  public Trigger isHoodAtPosition = new Trigger(() -> {return this.hoodAtPosition;});
  public Trigger isTurretAtPosition = new Trigger(() -> {return this.turretAtPosition;});
  public Trigger isShooterAtVelocity = new Trigger(() -> {return this.shooterAtVelocity;});
  public Trigger isKickupAtVelocity = new Trigger(() -> {return this.kickupAtVelocity;});
  public Trigger isSpindexerAtVelocity = new Trigger(() -> {return this.spindexerAtVelocity;});
  public Trigger isReadyToFire = new Trigger(() -> {return this.readyToFire;});
  
  public Trigger isTurretZeroed = new Trigger(() -> {return this.isTurretAtZero();});
  public Trigger isSpindexerStopped = new Trigger(() -> {return this.isSpindexerStopped();});
  public Trigger isKickupStopped = new Trigger(() -> {return this.isKickupStopped();});

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Command", this::getCurrentCommandName, null);
    builder.addDoubleProperty("Distance to Target", () -> {return this.distanceToTarget;}, null);
    builder.addDoubleProperty("Distance to Virtual Target", () -> {return this.distanceToVirtualTarget;}, null);
    builder.addDoubleProperty("Virtual Hood Angle", () -> {return this.virtualHoodAngle;}, null);
    builder.addDoubleProperty("Virtual Flywheel Velocity", () -> {return this.virtualFlywheelVelocity;}, null);
    builder.addDoubleProperty("Virtual Turret Angle", () -> {return this.virtualTurretAngle;}, null);
    builder.addDoubleProperty("Desired Hood Position", () -> {return this.desiredHoodPosition;}, null);
    builder.addDoubleProperty("Desired Turret Position", () -> {return this.desiredTurretPosition;}, null);
    builder.addDoubleProperty("Desired Shooter Velocity", () -> {return this.desiredShooterVelocity;}, null);
    builder.addDoubleProperty("Desired Spindexer Velocity", () -> {return this.desiredSpindexerVelocity;}, null);
    builder.addDoubleProperty("Desired Kickup Velocity", () -> {return this.desiredKickupVelocity;}, null);
    builder.addDoubleProperty("Current Turret Angle", () -> {return this.currentTurretAngle;}, null);
    builder.addDoubleProperty("Current Hood Angle", () -> {return this.currentHoodAngle;}, null);
    builder.addDoubleProperty("Current Flywheel Velocity", () -> {return this.currentFlywheelVelocity;}, null);
    builder.addDoubleProperty("Tuning Hood Angle", () -> {return this.tuningHoodAngle;}, this::setTuningHoodPosition);
    builder.addDoubleProperty("Tuning Turret Angle", () -> {return this.tuningTurretAngle;}, this::setTuningTurretPosition);
    builder.addDoubleProperty("Tuning Flywheel Velocity", () -> {return this.tuningFlywheelVelocity;}, this::setTuningShooterVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Evaluate boolean conditions for triggers.
    this.hoodAtPosition = this.atSetpoint(this.desiredHoodPosition, this.getHoodPosition(), ShooterConstants.kHoodPositionDeadband);
    this.turretAtPosition = this.atSetpoint(this.desiredTurretPosition, this.getTurretPosition(), ShooterConstants.kTurretPositionDeadband);
    this.shooterAtVelocity = this.atSetpoint(this.desiredShooterVelocity, this.getShooterOneVelocity(), ShooterConstants.kShooterVelocityDeadband);
    this.kickupAtVelocity = this.atSetpoint(this.desiredKickupVelocity, this.getKickupSpeed(), ShooterConstants.kKickupVelocityDeadband);
    this.spindexerAtVelocity = this.atSetpoint(this.desiredSpindexerVelocity, this.getSpindexerVelocity(), ShooterConstants.kSpindexerVelocityDeadband);

    this.currentHoodAngle = this.getHoodPosition();
    this.currentTurretAngle = this.getTurretPosition();
    this.currentFlywheelVelocity = this.getShooterOneVelocity();
    
    // Signal that we are ready to fire if the hood and turret are at position, and the shooter is at velocity.
    this.readyToFire = this.hoodAtPosition && this.turretAtPosition && this.shooterAtVelocity;

    //First attempt of the shoot while moving calculation.
    this.distanceToTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveStateSupplier.get().Pose, ShooterConstants.kRedHubCenter);
    this.currentTarget = ShotCalculation.getInstance().getVirtualTarget(this.swerveStateSupplier.get().Speeds, ShooterConstants.timeOfFlightMap.get(this.distanceToTarget), ShooterConstants.kRedHubCenter);
    
    this.distanceToVirtualTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveStateSupplier.get().Pose, this.currentTarget);

    this.virtualHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToVirtualTarget);
    this.virtualFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToVirtualTarget);
    // this.virtualTurretAngle = swerveStateSupplier.get().Pose.getRotation().minus(this.currentTarget.getRotation()).getDegrees();
    this.virtualTurretAngle = this.currentTarget.minus(this.swerveStateSupplier.get().Pose).getTranslation().getAngle().getDegrees();

    // Every loop, update the odometry with the pose of the virtual target.
    this.targetingField.setRobotPose(this.currentTarget);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    this.sim.simulationPeriodic();
    this.sim.updateShooterHoodVis(this.currentFlywheelVelocity, this.currentHoodAngle);
  }
}
