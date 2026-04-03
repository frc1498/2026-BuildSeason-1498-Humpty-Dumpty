package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConfig {
    //Constants go here
    public static final int kClimbMotorCANID = 17;

    //Variables
    public TalonFXConfiguration ClimbMotorConfig;
    public TalonFXConfiguration ClimbMotorZeroConfig;

    /**
     * Constructor for the climber motor configuration.
     */
    public ClimberConfig() {
        ClimbMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureClimber(ClimbMotorConfig);        //Fill in framework

        ClimbMotorZeroConfig = new TalonFXConfiguration();
        this.configureClimberZero(ClimbMotorZeroConfig);

    }

    /**
     * Sets up the base TalonFX configuration for the climber motor.
     * @param climber - The TalonFX configuration to apply the settings to.
     */
    public void configureClimber(TalonFXConfiguration climber) {
        //configure motor
        climber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climber.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        climber.MotorOutput.PeakForwardDutyCycle = 1;
        climber.MotorOutput.PeakReverseDutyCycle = -1;

        climber.CurrentLimits.StatorCurrentLimit = 150; //30 for testing - 120 for actual climb
        climber.CurrentLimits.StatorCurrentLimitEnable = true;
        climber.CurrentLimits.SupplyCurrentLimit = 40;    //5 for testing - 60 for actual climb
        climber.CurrentLimits.SupplyCurrentLimitEnable = true;
        climber.CurrentLimits.SupplyCurrentLowerLimit = 40; //60 for testing
        climber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        climber.Slot0.kP = 5.0;  // An error of 1 rotation per second results in 2V output
        climber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        climber.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        climber.Slot0.kS = 0;
        climber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        climber.Slot0.kA = 0;
        climber.Slot0.kG = 0;
        climber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        climber.Voltage.PeakForwardVoltage = 11;
        climber.Voltage.PeakReverseVoltage = -11;

        climber.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the TalonFX configuration for the climber motor to use while running the zero-ing routine.
     * @param climber - The TalonFX configuration to apply the settings to.
     */
    public void configureClimberZero(TalonFXConfiguration climber) {
        //configure motor
        climber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climber.MotorOutput.PeakForwardDutyCycle = 0.25;    // A Maximum output of 3 V would be a 25% duty cycle, assuming a 12 V input.
        climber.MotorOutput.PeakReverseDutyCycle = -0.25;

        climber.CurrentLimits.StatorCurrentLimit = 50.0;
        climber.CurrentLimits.StatorCurrentLimitEnable = true;
        climber.CurrentLimits.SupplyCurrentLimit = 5.0;
        climber.CurrentLimits.SupplyCurrentLimitEnable = true;
        climber.CurrentLimits.SupplyCurrentLowerLimit = 5.0;
        climber.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        //Slot 0 Configs
        climber.Slot0.kP = 30;  // An error of 1 rotation per second results in 2V output
        climber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        climber.Slot0.kD = 0.9;  // A change of 1 rotation per second squared results in 0.01 volts output
        climber.Slot0.kS = 0;
        climber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        climber.Slot0.kA = 0;
        climber.Slot0.kG = 0;
        climber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        climber.Voltage.PeakForwardVoltage = 3;
        climber.Voltage.PeakReverseVoltage = -3;

        climber.Audio.AllowMusicDurDisable = true;
    }

}
