// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This subsystem manages the spindexer, kickup, turret, hood, shooter motors 

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
//import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.ShotCalculation;
import frc.robot.config.ShooterConfig;
import frc.robot.sim.ShooterSim;

import frc.robot.constants.MotorEnableConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.MotorEnableConstants.LogLevel;

/**
 * The shooter subsystem.  Contains the flywheel, turret, hood adjustment, spindexer, and ball kickup.
 */
public class Shooter extends SubsystemBase {

/*==================Variables=======================*/

  private TalonFX shooterLeftMotor;   // Motor type definition
  private TalonFX shooterRightMotor;   // Motor type definition
  private TalonFX turretMotor;     // Motor type definition
  private TalonFX hoodMotor;       // Motor type definition

  private VelocityTorqueCurrentFOC shooterMotorMode;   // Motor control type definition

  private PositionTorqueCurrentFOC turretMotorMode; // Motor control type definition
  private PositionTorqueCurrentFOC hoodMotorMode;   // Motor control type definition

  private ShooterConfig shooterConfig;  // Create an object of type shooter subsystem config used to configure motors

  public Pose2d currentTarget;
  public Pose2d hubTarget;

  private double desiredHoodAngle;
  private double desiredHoodMotorRotations;
  private double desiredTurretAngle;
  private double desiredTurretMotorRotations;
  private double desiredShooterVelocity;

  private boolean hoodAtPosition;
  private boolean turretAtPosition;
  private boolean shooterAtVelocity;
  private boolean readyToFire;

  private Supplier<SwerveDriveState> swerveStateSupplier;
  private double distanceToTarget;
  private double distanceToVirtualTarget;

  private double virtualHoodAngle;
  private double virtualFlywheelVelocity;
  private double virtualTurretAngle;

  private double whileMoveHoodAngle;
  private double whileMoveFlywheelVelocity;
  private double whileMoveTurretAngle;

  private double currentHoodAngle;
  private double currentHoodRotations;
  private double currentTurretAngle;
  private double currentTurretRotations;
  private double currentShooterVelocity;

  private double tuningHoodAngle;
  private double tuningTurretAngle;
  private double tuningFlywheelVelocity;
  private boolean tuningShotSuccessful;

  private ShooterSim sim;
  private TalonFXSimState shooterLeftMotorSim;
  private TalonFXSimState shooterRightMotorSim;
  private TalonFXSimState turretMotorSim;
  private TalonFXSimState hoodMotorSim;
  
  public DutyCycleOut turretDutyCycle;
  public DutyCycleOut shooterDutyCycle;

  private double simTime;

  private LinearFilter velocityFilter = LinearFilter.movingAverage(3);

  String allianceColor = "Blue";
  public Pose2d targetLocation = new Pose2d(11.912, 4.028, Rotation2d.fromDegrees(0)); //Default it to blue

  //boolean turretZeroed;
  boolean requestShoot;

  public Field2d targetingField = new Field2d();

  /* SysId routine for characterizing the hood motor. This is used to find PID gains for the hood motor. */
  private SysIdRoutine sysIdRoutineHood = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,        // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdHood_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      volts -> this.hoodMotor.setControl(new VoltageOut(0).withOutput(volts)),
      null,
      this
    )
  );

  /* SysId routine for characterizing the hood motor. This is used to find PID gains for the hood motor. */
  private SysIdRoutine sysIdRoutineTurret = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,        // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdHood_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      volts -> this.turretMotor.setControl(new VoltageOut(0).withOutput(volts)),
      null,
      this
    )
  );

  /* SysId routine for characterizing the hood motor. This is used to find PID gains for the hood motor. */
  private SysIdRoutine sysIdRoutineFlywheel = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,        // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdHood_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      volts -> this.shooterRightMotor.setControl(new VoltageOut(0).withOutput(volts)),
      null,
      this
    )
  );

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
   
  this.turretMotor = new TalonFX(ShooterConfig.kTurretMotorCANID, "canivore");        // Create the turret rotate motor.
  this.turretMotorMode = new PositionTorqueCurrentFOC(0);                                         // Set the control mode for the turret motor.
  this.configureMechanism(this.turretMotor, this.shooterConfig.turretMotorConfig);
    
  this.hoodMotor = new TalonFX(ShooterConfig.kHoodMotorCANID, "canivore");            // Create hood adjustment motor.
  this.hoodMotorMode = new PositionTorqueCurrentFOC(0);                                           // Set the contorl mode for the adjustment motor.
  this.configureMechanism(this.hoodMotor, this.shooterConfig.hoodMotorConfig);

  this.shooterLeftMotorSim = this.shooterLeftMotor.getSimState();
  this.shooterRightMotorSim = this.shooterRightMotor.getSimState();
  this.turretMotorSim = this.turretMotor.getSimState();
  this.hoodMotorSim = this.hoodMotor.getSimState();

  this.sim = new ShooterSim(
    this.shooterConfig,
    this.hoodMotorSim,
    this.turretMotorSim,
    this.shooterLeftMotorSim,
    this.shooterRightMotorSim
    );

  // Publish subsystem data to SmartDashboard.
  SmartDashboard.putData("Shooter", this);
  SmartDashboard.putData("Shooter/Pose", this.targetingField);
  //SmartDashboard.putData("Shooter/Sim", this.sim.getVis());

  //turretZeroed = true;
  turretDutyCycle = new DutyCycleOut(0.0);
  shooterDutyCycle = new DutyCycleOut(0.0);

  requestShoot = false;

  this.hoodMotor.setPosition(0);
  this.turretMotor.setPosition(0);

  this.setHoodAngle(0);
  this.setTurretAngle(0);
  this.desiredShooterVelocity=-10;
  

  // Initialize the tuning parameters.
  // This is not for PID tuning, but instead for interpolation tuning.
  this.tuningHoodAngle = 0.0;
  this.tuningTurretAngle = 0.0;
  this.tuningFlywheelVelocity = 0.0;
  this.tuningShotSuccessful = false;

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

  //=========================================================
  /*===================Private Methods=====================*/
  //===========================================================

  //===================Hood Private Methods=======================
  /**
   * Set the position of turret hood.
   * @param position - The desired hood position, in angle of hood
   */

    private void setHoodAngle(double position) {
    // Always store the setpoint, to track the desired position.
    this.desiredHoodAngle = position;

    //Convert to Rotations
    this.desiredHoodMotorRotations = this.desiredHoodAngle / 360 * ShooterConstants.kHoodGearRatio;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kHoodMotorEnabled) {
      // Check if the desired hood position is within the allowable safety range.
      // Do not update the hood position if it is out of range.
      if (this.isSetpointWithinSafetyRange(this.desiredHoodAngle, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend)) {
        this.hoodMotor.setControl(this.hoodMotorMode.withPosition(this.desiredHoodMotorRotations));
      } else {
        // Log a fault with DogLog if the desired hood position was out of range.
        // Temporary - loop overruns    DogLog.logFault(ShooterFault.HOOD_SETPOINT_OUT_OF_RANGE);
      }
    }
  }

  /**
   * Return the current position of the turret hood.
   * @return - The current position of the turret hood, in rotations.
   */
  private double getHoodAngle() {
      //Convert motor rotations and return Angle
    return (this.hoodMotor.getPosition().getValueAsDouble() * 360 / ShooterConstants.kHoodGearRatio);
  }

  private void stopShooting() {
    this.shooterLeftMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.shooterRightMotor.setControl(this.shooterDutyCycle.withOutput(0));
    this.desiredShooterVelocity=0;
  }

  private double getHoodRotations() {
      //Convert motor rotations and return Angle
    return (this.hoodMotor.getPosition().getValueAsDouble());
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

  //==============================Turret Private Methods===========================


  /**
   * Set the position of the turret angle.
   * @param position - The desired position of the turret, in rotations.
   */
  private void setTurretAngle(double position) {
    // Always store the setpoint, to track the desired position.
    this.desiredTurretAngle = position;

    this.desiredTurretMotorRotations = this.desiredTurretAngle / 360 * ShooterConstants.kTurretGearRatio;

    if (MotorEnableConstants.kTurretMotorEnabled) {     // Use this constant to enable or disable motor output for debugging.
      if (this.isSetpointWithinSafetyRange(this.desiredTurretAngle, ShooterConstants.kTurretSafeClockwise, ShooterConstants.kTurretSafeCounterClockwise)) {
        this.turretMotor.setControl(this.turretMotorMode.withPosition(this.desiredTurretMotorRotations));
      } else {
        // Log a fault with DogLog if the desired turret position was out of range.
        // Temporary loop overruns - DogLog.logFault(ShooterFault.TURRET_SETPOINT_OUT_OF_RANGE);        
      }
    }
  }

  /**
   * Return the current position of the turret angle.
   * @return - The current position of the turret, in rotations.
   */
  private double getTurretAngle() {
    //Convert Motor Rotations to Angle and Return it
    return (this.turretMotor.getPosition().getValueAsDouble() * 360 / ShooterConstants.kTurretGearRatio);
  }

  private double getTurretRotations() {
    //Convert Motor Rotations to Angle and Return it
    return (this.turretMotor.getPosition().getValueAsDouble());
  }

  /**
   * Updates the turret angle setpoint to use while tuning the shooter interpolation.
   * @param tuningTurretAngle
   */
  private void setTuningTurretPosition(double tuningTurretAngle) {
    this.tuningTurretAngle = tuningTurretAngle;
  }

  //==================================Shooter Private Methods========================================

  /**
   * Set the velocity of both shooter motors.
   * @param velocity - The desired velocity of the shooter motors, in rotations per second.
   */
  private void setShooterVelocity(double velocity) {
    // Always store the setpoint, to track the desired velocity.
    this.desiredShooterVelocity = velocity;

    // Use this constant to enable or disable motor output for debugging.
    if (MotorEnableConstants.kShooterLeftMotorEnabled && MotorEnableConstants.kShooterRightMotorEnabled) {
      this.shooterRightMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
      this.shooterLeftMotor.setControl(this.shooterMotorMode.withVelocity(this.desiredShooterVelocity));
    }
  }

  /**
   * Return the current velocity of the first shooter motor.
   * @return - The current velocity of the first shooter motor, in rotations per second.
   */
  private double getShooterVelocity() {
    double velocity = velocityFilter.calculate(this.shooterRightMotor.getVelocity().getValueAsDouble());
    return velocity;
  }

  /**
   * Updates the flywheel velocity setpoint to use while tuning the shooter interpolation.
   * @param tuningFlywheelVelocity
   */
  private void setTuningShooterVelocity(double tuningFlywheelVelocity) {
    this.tuningFlywheelVelocity = tuningFlywheelVelocity;
  }

  private void setTargetAllianceCornerRight() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      //targetLocation = ShooterConstants.kRedRight;
    } else if (allianceColor == "Blue") {
      //targetLocation = ShooterConstants.kBlueRight;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
  }

  private void setTargetAllianceCornerLeft() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      //targetLocation = ShooterConstants.kRedLeft;
    } else if (allianceColor == "Blue") {
      //targetLocation = ShooterConstants.kBlueLeft;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
  }

  private void setTargetAllianceHub() {
    allianceColor = DriverStation.getAlliance().get().toString();
    if (allianceColor == "Red") {
      //targetLocation = ShooterConstants.kRedHubCenter;
    } else if (allianceColor == "Blue") {
      //targetLocation = ShooterConstants.kBlueHubCenter;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
    //May need to import the driver station to use it for allianceColor
  }

  //===============================Misc. Private Methods===================

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

  /**
   * Logs variables from the subsystem via DogLog.  The amount of variables logged can be controlled with the logLevel parameter.
   * @param logLevel - The level of logging to enable.
   */
  private void log(MotorEnableConstants.LogLevel logLevel) {
    switch (logLevel) {
      case NONE:
        break;
      case FULL:
        break;
      default:
        break;
    }
  }

  /**
   * Set the result of the last tuning shot.  True is successful, false is unsuccessful.
   * @param tuningShotSuccessful
   */
  private void setTuningShotSuccessful(boolean tuningShotSuccessful) {
    this.tuningShotSuccessful = tuningShotSuccessful;
  }

  /**
   * Log the latest results of the tuning shot.  Remember to always update the boolean flag before running this method.
   */
  private void logTuningResult() {
    /*  Temporary - loop overruns
    DogLog.log("Tuning Hood Angle", this.tuningHoodAngle);
    DogLog.log("Tuning Turret Angle", this.tuningTurretAngle);
    DogLog.log("Tuning Flywheel Velocity", this.tuningFlywheelVelocity);
    DogLog.log("Shot Successful", this.tuningShotSuccessful);
    */
  }

  /**
   * Converts the desired angle of the turret (-180 to 180) to an angle that the motor can use.
   * If the counterclockwise limit is greater than 180.0, this will convert a negative desired angle to an equivalent positive angle for use by the setTurret method.
   * @return
   */
  private double convertTurretOverturn(double desiredTurretAngle) {
    // WPI rotation wraps at 180 to -180, but the turret motor expects rotations.
    // If the angle determined goes from + to - at this point, the motor should accept it when converted down to a rotation.
    // The amount that the turret can turn past 180 was calculated in the constants file.
    double negativeLimit = -180.0 + ShooterConstants.kTurretOverturn;

    // If the desired angle is greater than 180 (which is actually -180 or less in this reference) but less than the overturn, add the difference to 180 for the 'turret motor friendly' angle.
    if(this.isSetpointWithinSafetyRange(desiredTurretAngle, -180.0, negativeLimit)) {
      return 180.0 + (180.0 - Math.abs(desiredTurretAngle));
    }
    else return desiredTurretAngle;
  }

  //=========================================================
  /*====================Public Methods=====================*/
  //=========================================================

  /**
   * Sets the hood position, turret position, and shooter velocity based on the shoot while move capability.
   * @return
   */
  /* I don't want anything else setting turret / hood / shooter speed angles during debug
   public Command setShooterOutputs() {
    
    return runOnce(() -> {
      this.setHoodAngle(this.virtualHoodAngle);
      this.setTurretAngle(this.virtualTurretAngle);
      this.setShooterVelocity(this.virtualFlywheelVelocity);
    }).withName("setShooterOutputs");
  }
  */

  /**
   * Sets the hood position, turret position, and shooter velocity based on the tuning parameters.
   * @return
   */
  /* I don't want anything else potentially accessing turret / hood angles right now  
   public Command setTuningShooterOutputs() {
    return run(() -> {
      
      this.setHoodAngle(this.tuningHoodAngle);
      this.setTurretAngle(this.tuningTurretAngle);
      this.setShooterVelocity(this.tuningFlywheelVelocity);
      }).withName("setTuningShooterOutputs");
      
  }
  */

  /**
   * Log the latest results of the tuning shot.
   * @return
   */
  public Command logLatestTuningResult() {
    return runOnce(() -> {
      this.logTuningResult();
    });
  }
 
  private void requestStopShooting() {
    requestShoot=false;
  }

  private void requestStartShooting() {
    requestShoot=true;
  }

  private boolean isShooterAtSpeed() {  //Modified to make sure we are above speed only.
    return ((this.getShooterVelocity() > (this.desiredShooterVelocity - ShooterConstants.kShooterVelocityDeadband)));
  }

  private boolean isTurretAtPosition() {
    return ((this.getTurretAngle() < (this.desiredTurretAngle + ShooterConstants.kTurretPositionDeadband)) 
    && (this.getTurretAngle() > (this.desiredTurretAngle - ShooterConstants.kTurretPositionDeadband)));
  }

  private boolean isHoodAtPosition() {
    return ((this.getHoodAngle() < (this.desiredHoodAngle + ShooterConstants.kHoodPositionDeadband)) 
    && (this.getHoodAngle() > (this.desiredHoodAngle - ShooterConstants.kHoodPositionDeadband)));
  }

  //===================Public Shoot Commands=====================
  public Command requestStopShoot() {
    return runOnce(() -> {this.requestStopShooting();});
  }

  public Command requestStartShoot() {
    return runOnce(() -> {this.requestStartShooting();});
  }

  public Command stopShoot(){
    return runOnce(() -> {this.stopShooting();});
  }

  public Command startShootStatic(){
    return run(() -> {this.setShooterVelocity(75);}).until(isShooterAtVelocity);
  }

  public Command startShootFast(){
    return run(() -> {this.setShooterVelocity(80);}).until(isShooterAtVelocity);
  }

  /**
   * Set the shooter velocity based on the distance to the hub.
   * @return
   */
  public Command autoShoot() {
    return runOnce(() -> {this.setShooterVelocity(this.virtualFlywheelVelocity);});
  }

  /**
   * Set the shooter velocity based on the distance to the estimated hub for the shoot while move.
   * @return
   */
  public Command whileMoveShoot() {
    return runOnce(() -> {this.setShooterVelocity(this.whileMoveFlywheelVelocity);});
  }

  public Command setTargetToAllianceCornerRight() {
    return runOnce(() -> {this.setTargetAllianceCornerRight();});
  }

  public Command setTargetToAllianceCornerLeft() {
    return runOnce(() -> {this.setTargetAllianceCornerLeft();});
  }

  public Command setTargetToAllianceHub() {
    return runOnce(() -> {this.setTargetAllianceHub();});
  }

  //===================Public Turret Commands=====================
  public Command turretCounterClockwise45() {
    return runOnce(() -> {this.setTurretAngle(45);});
  }

  public Command turret0() {
    return runOnce(() -> {this.setTurretAngle(0);});
  }

  public Command turretClockwise45() {
    return runOnce(() -> {this.setTurretAngle(-45);});
  }

  public Command turretClimbPosition() {
    return runOnce(() -> {this.setTurretAngle(ShooterConstants.kTurretClimbPosition);})
    .until(isTurretAtPosition);
  }


  private String getAlliance() {
    return DriverStation.getAlliance().get().toString();

  }

  public Command getOurAlliance () {
    return runOnce(() -> {getAlliance();});
  }

  /**
   * Development command.  Use this command to set the turret angle based on where the robot thinks the blue hub is, and where the robot thinks it is.
   * IMPORTANT - This assumes the turret faces the intake (the back of the robot) when it is zeroed.  If this is not true, remove the unaryMinus() method from the virtualTurretAngle calculation.
   * ALSO IMPORTANT - Consider keeping the allowable range for turret angle low while testing this.
   * @return
   */
  public Command turretTrackToBlueHub() {
    return runOnce(() -> {this.setTurretAngle(this.virtualTurretAngle);})
      .until(this.isTurretAtPosition);
  }

  /**
   * Set the turret based on the angle to the hub.
   * @return
   */
  public Command autoTurret() {
    return runOnce(() -> {this.setTurretAngle(this.virtualTurretAngle);});
  }

  /**
   * Set the turret based on the angle to the estimated hub for shoot while moving.
   * @return
   */
  public Command whileMoveTurret() {
    return runOnce(() -> {this.setTurretAngle(this.whileMoveTurretAngle);});
  }

  //=====================Public Hood Commands================
  public Command hood0() {
    return runOnce(() -> {this.setHoodAngle(0);});
  }

  public Command hood30() {
    return runOnce(() -> {this.setHoodAngle(30);});
  }

  /**
   * Set the hood based on the distance to the hub.
   * @return
   */
  public Command autoHood() {
    return runOnce(() -> {this.setHoodAngle(this.virtualHoodAngle);});
  }

  /**
   * Set the hood based on the estimated distance to the hub for the shoot while move.
   * @return
   */
  public Command whileMoveHood() {
    return runOnce(() -> {this.setHoodAngle(this.whileMoveHoodAngle);});
  }

  //======================Triggers=========================
  public Trigger isHoodAtPosition = new Trigger(() -> {return isHoodAtPosition();});
  public Trigger isTurretAtPosition = new Trigger(() -> {return isTurretAtPosition();});
  public Trigger isShooterAtVelocity = new Trigger(() -> {return isShooterAtSpeed();});
  public Trigger isReadyToFire = new Trigger(() -> {return this.readyToFire;});

  @Override
  public void initSendable(SendableBuilder builder) {
    //builder.addStringProperty("Command", this::getCurrentCommandName, null);
    //builder.addDoubleProperty("Distance to Target", () -> {return this.distanceToTarget;}, null);
    //builder.addDoubleProperty("Distance to Virtual Target", () -> {return this.distanceToVirtualTarget;}, null);
    
    // Issues with too many sendables overruning loop.  Removed these for now to test
    /*
    builder.addDoubleProperty("Virtual Hood Angle", () -> {return this.virtualHoodAngle;}, null);
    builder.addDoubleProperty("Virtual Flywheel Velocity", () -> {return this.virtualFlywheelVelocity;}, null);
    builder.addDoubleProperty("Virtual Turret Angle", () -> {return this.virtualTurretAngle;}, null);
    
    builder.addDoubleProperty("Desired Hood Angle", () -> {return this.desiredHoodAngle;}, null);
    builder.addDoubleProperty("Desired Turret Angle", () -> {return this.desiredTurretAngle;}, null);
    builder.addDoubleProperty("Desired Hood Motor Rotations", () -> {return this.desiredHoodMotorRotations;}, null);
    builder.addDoubleProperty("Desired Turret Motor Rotations", () -> {return this.desiredTurretMotorRotations;}, null);
    builder.addDoubleProperty("Desired Shooter Velocity", () -> {return this.desiredShooterVelocity;}, null);
    builder.addDoubleProperty("Current Turrent Rotations", () -> {return this.currentTurretRotations;},null);
    builder.addDoubleProperty("Current Hood Rotations", () -> {return this.currentHoodRotations;},null);
    builder.addDoubleProperty("Current Turret Angle", () -> {return this.currentTurretAngle;}, null);
    builder.addDoubleProperty("Current Hood Angle", () -> {return this.currentHoodAngle;}, null);
    builder.addDoubleProperty("Current Shooter Velocity", () -> {return this.currentShooterVelocity;}, null);

    builder.addBooleanProperty("Shooter At Velocity", () -> {return this.shooterAtVelocity;}, null);
    builder.addDoubleProperty("Debug Turret Angle", () -> {return this.virtualTurretAngle;}, null);
    */
    builder.addStringProperty("Alliance:", () -> {return DriverStation.getAlliance().get().toString();},null );
    builder.addStringProperty("Target", () -> {return targetLocation.toString();},null );

    /*  Overruning sendable loop
    builder.addDoubleProperty("Tuning Hood Angle", () -> {return this.tuningHoodAngle;}, this::setTuningHoodPosition);
    builder.addDoubleProperty("Tuning Turret Angle", () -> {return this.tuningTurretAngle;}, this::setTuningTurretPosition);
    builder.addDoubleProperty("Tuning Flywheel Velocity", () -> {return this.tuningFlywheelVelocity;}, this::setTuningShooterVelocity);
    builder.addBooleanProperty("Tuning Shot Successful", () -> {return this.tuningShotSuccessful;}, this::setTuningShotSuccessful);
    */
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Evaluate boolean conditions for triggers.
    //this.hoodAtPosition = this.atSetpoint(this.desiredHoodAngle, this.getHoodAngle(), ShooterConstants.kHoodPositionDeadband);
    //this.turretAtPosition = this.atSetpoint(this.desiredTurretAngle, this.getTurretAngle(), ShooterConstants.kTurretPositionDeadband);
    //this.shooterAtVelocity = this.atSetpoint(this.desiredShooterVelocity, this.getShooterVelocity(), ShooterConstants.kShooterVelocityDeadband);

    this.currentHoodAngle = this.getHoodAngle();
    this.currentHoodRotations = this.getHoodRotations();
    this.currentTurretAngle = this.getTurretAngle();
    this.currentTurretRotations = this.getTurretRotations();
    this.currentShooterVelocity = this.getShooterVelocity();
    
    allianceColor=DriverStation.getAlliance().get().toString();
    
    //Set our target based on our alliance color
    if (allianceColor == "Red") {
      targetLocation = ShooterConstants.kRedHubCenter;
    } else if (allianceColor == "Blue") {
      targetLocation = ShooterConstants.kBlueHubCenter;
    } else {
      // Code to handle the case where the alliance color is not yet available
    }
    

    // Signal that we are ready to fire if the hood and turret are at position, and the shooter is at velocity.
    this.readyToFire = this.hoodAtPosition && this.turretAtPosition && this.shooterAtVelocity;

    //First attempt of the shoot while moving calculation.
    this.distanceToTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveStateSupplier.get().Pose.transformBy(ShooterConstants.kRobotToTurret), targetLocation);
    //this.distanceToTarget = ShotCalculation.getInstance().getTargetDistance(this.swerveStateSupplier.get().Pose.transformBy(ShooterConstants.kRobotToTurret), ShooterConstants.kBlueHubCenter);
    // this.currentTarget = ShotCalculation.getInstance().getVirtualTarget(this.swerveStateSupplier.get().Speeds, this.swerveStateSupplier.get().Pose.transformBy(ShooterConstants.kRobotToTurret), ShooterConstants.timeOfFlightMap.get(this.distanceToTarget), ShooterConstants.kRedHubCenter);
    
    this.distanceToVirtualTarget = ShotCalculation.getInstance().getDistanceToVirtualTarget(this.swerveStateSupplier.get().Speeds, this.swerveStateSupplier.get().Pose, targetLocation);

    this.virtualHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToTarget);
    this.virtualFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToTarget);
    // this.virtualTurretAngle = swerveStateSupplier.get().Pose.getRotation().minus(this.currentTarget.getRotation()).getDegrees();
    this.virtualTurretAngle = this.convertTurretOverturn(targetLocation.minus(this.swerveStateSupplier.get().Pose.transformBy(ShooterConstants.kRobotToTurret)).getTranslation().getAngle().getDegrees());

    this.whileMoveHoodAngle = ShooterConstants.hoodAngleMap.get(this.distanceToVirtualTarget);
    this.whileMoveFlywheelVelocity = ShooterConstants.flywheelSpeedMap.get(this.distanceToVirtualTarget);
    this.whileMoveTurretAngle = this.convertTurretOverturn(ShotCalculation.getInstance().getVirtualTarget().minus(this.swerveStateSupplier.get().Pose.transformBy(ShooterConstants.kRobotToTurret)).getTranslation().getAngle().getDegrees());

    // Every loop, update the odometry with the pose of the virtual target.
    this.targetingField.getObject("Hub Target").setPose(ShotCalculation.getInstance().getVirtualTarget());
    this.targetingField.getObject("Angler").setPose(0.0,0.0,Rotation2d.kZero);
    this.log(LogLevel.NONE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //this.sim.simulationPeriodic();
    //this.sim.updateShooterHoodVis(this.currentShooterVelocity, this.currentHoodAngle, this.hoodAtPosition);
  }
}
