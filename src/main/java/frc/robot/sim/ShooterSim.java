package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.config.ShooterConfig;
import frc.robot.constants.ShooterConstants;

public class ShooterSim implements AutoCloseable {

    public ShooterConfig shooterConfig;
    public TalonFXSimState hood;
    public TalonFXSimState turret;
    public TalonFXSimState shooterLeft;
    public TalonFXSimState shooterRight;
    public TalonFXSimState spindexer;
    public TalonFXSimState kickup;

    public DCMotor shooterGearbox = DCMotor.getKrakenX60Foc(2);
    public DCMotor hoodAdjust = DCMotor.getKrakenX44Foc(1);
    public DCMotor turretRotate = DCMotor.getKrakenX44Foc(1);
    public DCMotor spindexerRotate = DCMotor.getKrakenX60Foc(1);
    public DCMotor kickupRotate = DCMotor.getKrakenX60Foc(1);

    public FlywheelSim shooterFlywheel;
    public DCMotorSim hoodAdjustSim;
    public DCMotorSim turretRotateSim;
    public DCMotorSim spindexerRotateSim;
    public DCMotorSim kickupRotateSim;

    private double flywheelVelocity;
    private double flywheelPosition;

    private double hoodVelocity;
    private double hoodPositionDelta;
    private double hoodPosition;

    private double turretVelocity;
    private double turretPositionDelta;
    private double turretPosition;

    private double spindexerVelocity;
    private double spindexerPosition;

    private double kickupVelocity;
    private double kickupPosition;

    public double simVoltage;
    public double simPeriod = 0.02;

    public Mechanism2d shooter_vis;
    public MechanismRoot2d shooter_root;
    public MechanismObject2d shooter_turret;
    public MechanismLigament2d shooter_hood;

     /*new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 100.0), gearbox);*/
    public ShooterSim(ShooterConfig config, TalonFXSimState hood, TalonFXSimState turret, TalonFXSimState shooterLeft, TalonFXSimState shooterRight, TalonFXSimState spindexer, TalonFXSimState kickup) {
        this.shooterConfig = config;
        this.hood = hood;
        this.turret = turret;
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;
        this.spindexer = spindexer;
        this.kickup = kickup;

        this.hoodPosition = 0.0;
        this.turretPosition = 0.0;

        this.shooterFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(this.shooterGearbox, 0.001, ShooterConstants.kShooterFlywheelGearing),
            this.shooterGearbox
        );
        this.hoodAdjustSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.hoodAdjust, 0.001, ShooterConstants.kHoodGearing), this.hoodAdjust);
        this.turretRotateSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.turretRotate, 0.001, ShooterConstants.kTurretGearing), this.turretRotate);
        this.spindexerRotateSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.spindexerRotate, 0.001, ShooterConstants.kSpindexerGearing), this.spindexerRotate);
        this.kickupRotateSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.kickupRotate, 0.001, ShooterConstants.kKickupGearing), this.kickupRotate);
        
        this.shooter_vis = new Mechanism2d(20.0, 20.0);
        this.shooter_root = this.shooter_vis.getRoot("Origin", 10.0, 10.0);
        this.shooter_hood = shooter_root.append(new MechanismLigament2d("Hood", 0.0, 0.0));
    }

    public void simulationPeriodic() {
        // Set motor and sensor voltage.
        this.simVoltage = RoboRioSim.getVInVoltage();
        this.hood.setSupplyVoltage(this.simVoltage);
        this.turret.setSupplyVoltage(this.simVoltage);
        this.shooterLeft.setSupplyVoltage(this.simVoltage);
        this.shooterRight.setSupplyVoltage(this.simVoltage);
        this.spindexer.setSupplyVoltage(this.simVoltage);
        this.kickup.setSupplyVoltage(this.simVoltage);

        // Run the simulation and update it.
        this.shooterFlywheel.setInput(shooterRight.getMotorVoltage());
        this.shooterFlywheel.update(this.simPeriod);

        this.hoodAdjustSim.setInput(hood.getMotorVoltage());
        this.hoodAdjustSim.update(this.simPeriod);

        this.turretRotateSim.setInput(turret.getMotorVoltage());
        this.turretRotateSim.update(this.simPeriod);

        this.spindexerRotateSim.setInput(spindexer.getMotorVoltage());
        this.spindexerRotateSim.update(this.simPeriod);

        this.kickupRotateSim.setInput(kickup.getMotorVoltage());
        this.kickupRotateSim.update(this.simPeriod);

        // Update sensor positions.
        this.flywheelVelocity = this.outputRPMToInputRPS(this.shooterFlywheel.getAngularVelocityRPM(), ShooterConstants.kShooterFlywheelGearing);
        this.flywheelPosition = this.flywheelVelocity * this.simPeriod;
        this.shooterLeft.setRotorVelocity(this.flywheelVelocity);
        this.shooterLeft.addRotorPosition(this.flywheelPosition);
        this.shooterRight.setRotorVelocity(this.flywheelVelocity);
        this.shooterRight.addRotorPosition(this.flywheelPosition);

        this.hoodVelocity = this.outputRPMToInputRPS(this.hoodAdjustSim.getAngularVelocityRPM(), ShooterConstants.kHoodGearing);
        this.hoodPositionDelta = this.hoodVelocity * this.simPeriod;
        this.hoodPosition += this.hoodPositionDelta;
        this.hood.setRotorVelocity(this.hoodVelocity);
        this.hood.addRotorPosition(this.hoodPositionDelta);
        // If the simulation is overshooting the physical range, clamp it.
        if(!this.inRange(this.hoodPosition, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend)) {
            this.turret.setRawRotorPosition(MathUtil.clamp(this.hoodPosition, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend));
        }
        
        this.turretVelocity = this.outputRPMToInputRPS(this.turretRotateSim.getAngularVelocityRPM(), ShooterConstants.kTurretGearing);
        this.turretPositionDelta = this.turretVelocity * this.simPeriod;
        this.turretPosition += this.turretPositionDelta;
        this.turret.setRotorVelocity(this.turretVelocity);
        this.turret.addRotorPosition(this.turretPositionDelta);
        // If the simulation is overshooting the physical range, clamp it.
        if(!this.inRange(this.turretPosition, ShooterConstants.kTurretSafeCounterClockwise, ShooterConstants.kTurretSafeClockwise)) {
            this.turret.setRawRotorPosition(MathUtil.clamp(this.turretPosition, ShooterConstants.kTurretSafeCounterClockwise, ShooterConstants.kTurretSafeClockwise));
        }

        this.spindexerVelocity = this.outputRPMToInputRPS(this.spindexerRotateSim.getAngularVelocityRPM(), ShooterConstants.kSpindexerGearing);
        this.spindexerPosition = this.spindexerVelocity * this.simPeriod;
        this.spindexer.setRotorVelocity(this.spindexerVelocity);
        this.spindexer.addRotorPosition(this.spindexerPosition);
        
        this.kickupVelocity = this.outputRPMToInputRPS(this.kickupRotateSim.getAngularVelocityRPM(), ShooterConstants.kKickupGearing);
        this.kickupPosition = this.kickupVelocity * this.simPeriod;
        this.kickup.setRotorVelocity(this.kickupVelocity);
        this.kickup.addRotorPosition(this.kickupPosition);

        // Divide by 60 for rotations per second.
        // Multiply by simPeriod for rotation delta for this loop.
        // Divide by the gear ratio to convert the output to the input.

        //this.shooter_hood.setLength((this.flywheelVelocity / ShooterConstants.kShooterMaxSpeed * 10.0));

    }

    /**
     * Converts the simulated output velocity to the simulated input velocity.
     * @param velocity - The output velocity, in rotations per minute.
     * @param gearRatio - The gear ratio of the mechanism.
     * @return - The velocity of the input, in rotations per second.
     */
    private double outputRPMToInputRPS(double velocity, double gearRatio) {
        return (velocity / 60.0) / gearRatio;
    }

    /**
     * Checks if the setpoint is between the lower limit and upper limit.  Use this to determine when the position needs to be clamped.
     * @param setpoint - The setpoint to check.
     * @param lowerLimit - The lower limit (minimum).
     * @param upperLimit - The upper limit (maximum).
     * @return
     */
    private boolean inRange(double setpoint, double lowerLimit, double upperLimit) {
        return (setpoint >= lowerLimit) && (setpoint <= upperLimit);
    }

    public Mechanism2d getVis() {
        return this.shooter_vis;
    }

    public void updateShooterHoodVis(double velocity, double angle) {
        this.shooter_hood.setLength((velocity / ShooterConstants.kShooterMaxSpeed) * 10.0);
        this.shooter_hood.setAngle(angle);
    }

    @Override
    public void close() {

    }
}
