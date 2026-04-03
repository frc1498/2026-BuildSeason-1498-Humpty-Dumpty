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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.config.ShooterConfig;
import frc.robot.constants.ShooterConstants;

public class ShooterSim implements AutoCloseable {

    public ShooterConfig shooterConfig;
    public TalonFXSimState hood;
    public TalonFXSimState shooterLeft;
    public TalonFXSimState shooterRight;

    public DCMotor shooterGearbox = DCMotor.getKrakenX60Foc(2);
    public DCMotor hoodAdjust = DCMotor.getKrakenX44Foc(1);

    public FlywheelSim shooterFlywheel;
    public DCMotorSim hoodAdjustSim;

    private double flywheelVelocity;
    private double flywheelPosition;

    private double hoodVelocity;
    private double hoodPositionDelta;
    private double hoodPosition;

    public double simVoltage;
    public double simPeriod = 0.02;

    public Mechanism2d shooter_vis;
    public MechanismRoot2d shooter_root;
    public MechanismObject2d shooter_turret;
    public MechanismLigament2d shooter_hood;

    private enum VisState {
    AT_VELOCITY(new Color8Bit(Color.kGreen)),
    AT_POSITION(new Color8Bit(Color.kGreen)),
    FORWARD(new Color8Bit(Color.kAliceBlue)),
    REVERSE(new Color8Bit(Color.kRed));

    private Color8Bit color;

    /**
     * Constructor for the VisState enumeration.
     * @param color - The intended color of the visual.
     */
    VisState(Color8Bit color) {
      this.color = color;
    }

    /**
     * Returns the color for this state.
     * @return - Desired color for the visualization.
     */
    public Color8Bit color() {
      return this.color;
    }
  }

    /**
     * Constructs the simulation of the shooter subsystem.
     * @param config - The configuration for the shooter subsystem.
     * @param hood - The SimState of the hood motor.
     * @param shooterLeft - The SimState of the left shooter motor.
     * @param shooterRight - The SimState of the right shooter motor.
     */
    public ShooterSim(ShooterConfig config, TalonFXSimState hood, TalonFXSimState shooterLeft, TalonFXSimState shooterRight) {
        this.shooterConfig = config;
        this.hood = hood;
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;

        this.hoodPosition = 0.0;

        this.shooterFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(this.shooterGearbox, 0.001, ShooterConstants.kShooterFlywheelGearing),
            this.shooterGearbox
        );
        this.hoodAdjustSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.hoodAdjust, 0.001, ShooterConstants.kHoodGearing), this.hoodAdjust);
        
        this.shooter_vis = new Mechanism2d(20.0, 20.0);
        this.shooter_root = this.shooter_vis.getRoot("Origin", 10.0, 10.0);
        this.shooter_hood = shooter_root.append(new MechanismLigament2d("Hood", 0.0, 0.0));
    }

    /**
     * Runs the simulation of the shooter subsystem.  Updates the simulation state based on inputs from the code.
     */
    public void simulationPeriodic() {
        // Set motor and sensor voltage.
        this.simVoltage = RoboRioSim.getVInVoltage();
        this.hood.setSupplyVoltage(this.simVoltage);
        this.shooterLeft.setSupplyVoltage(this.simVoltage);
        this.shooterRight.setSupplyVoltage(this.simVoltage);

        // Run the simulation and update it.
        this.shooterFlywheel.setInput(shooterRight.getMotorVoltage());
        this.shooterFlywheel.update(this.simPeriod);

        this.hoodAdjustSim.setInput(hood.getMotorVoltage());
        this.hoodAdjustSim.update(this.simPeriod);

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
            this.hood.setRawRotorPosition(MathUtil.clamp(this.hoodPosition, ShooterConstants.kHoodSafeRetract, ShooterConstants.kHoodSafeExtend));
        }

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

    /**
     * Returns the simulation visualization for use within the subsystem the simulation is run in.
     * @return - The SmartDashboard visualization of the shooter subsystem.
     */
    public Mechanism2d getVis() {
        return this.shooter_vis;
    }

    /**
     * Updates the simulation visualization.
     * @param velocity - The velocity of the flywheel.  Normalized against the maximum speed, then multiplied by 10.
     * @param angle - The angle of the hood.
     * @param atVelocity - Changes the color of the velocity vector if true.
     */
    public void updateShooterHoodVis(double velocity, double angle, boolean atVelocity) {
        this.shooter_hood.setLength((velocity / ShooterConstants.kShooterMaxSpeed) * 10.0);
        this.shooter_hood.setAngle(angle);
        this.shooter_hood.setColor(atVelocity ? VisState.AT_POSITION.color() : VisState.REVERSE.color());
    }

    @Override
    public void close() {

    }
}
