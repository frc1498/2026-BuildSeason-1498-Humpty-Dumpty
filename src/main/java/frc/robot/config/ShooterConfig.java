package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConfig {
    //Constants go Here
    public static final int kShooterTopLeftMotorCANID = 13;
    public static final int kShooterBottomLeftMotorCANID = 14;
    public static final int kShooterTopRightMotorCANID = 15;
    public static final int kShooterBottomRightMotorCANID = 20;
    public static final int kHoodMotorCANID = 16;

    //Variables
    public TalonFXConfiguration hoodMotorConfig; //x44 motor
    public TalonFXConfiguration shooterTopLeftMotorConfig; //kraken motor (x60)
    public TalonFXConfiguration shooterBottomLeftMotorConfig; //x60 motor
    public TalonFXConfiguration shooterTopRightMotorConfig; //kraken motor (x60)
    public TalonFXConfiguration shooterBottomRightMotorConfig; //x60 motor

    /**
     * Constructor for the climber motor configuration.
     */
    public ShooterConfig(){
        hoodMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureHoodMotor(hoodMotorConfig); //Fill in framework, requires a method below

        shooterTopLeftMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureShooterMotorTopLeft(shooterTopLeftMotorConfig); //Fill in framework, requires a method below

        shooterBottomLeftMotorConfig = new TalonFXConfiguration();
        this.configureShooterMotorBottomLeft(shooterBottomLeftMotorConfig);

        shooterTopRightMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureShooterMotorTopRight(shooterTopRightMotorConfig); //Fill in framework, requires a method below

        shooterBottomRightMotorConfig = new TalonFXConfiguration();
        this.configureShooterMotorBottomRight(shooterBottomRightMotorConfig);
    }

    /**
     * Sets up the base TalonFX configuration for the hood adjustment motor.
     * @param hood - The TalonFX configuration to apply the settings to.
     */
    public void configureHoodMotor(TalonFXConfiguration hood) {
        //Configure Motor
        hood.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;  //Set 2-17-26
        hood.MotorOutput.NeutralMode=NeutralModeValue.Coast;  //Set 2-17-26
        hood.MotorOutput.PeakForwardDutyCycle = 1;
        hood.MotorOutput.PeakReverseDutyCycle = -1;

        hood.CurrentLimits.StatorCurrentLimit = 180;  //Set 2-17-26 to 120
        hood.CurrentLimits.StatorCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLimit = 15;    //Set 2-17-26 to 20
        hood.CurrentLimits.SupplyCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLowerLimit = 15; //Set 2-17-26 to 20
        hood.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Config
        hood.Slot0.kP = 10.0;  // An error of 1 rotation per second results in 2V output
        hood.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        hood.Slot0.kD = 0.1;  // A change of 1 rotation per second squared results in 0.01 volts output
        hood.Slot0.kS = 0;
        hood.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        hood.Slot0.kA = 0;
        hood.Slot0.kG = 0;

        hood.Voltage.PeakForwardVoltage = 11;
        hood.Voltage.PeakReverseVoltage = -11;

        hood.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the base TalonFX configuration for the top left shooter motor.
     * @param shooterLeft - The TalonFX configuration to apply the settings to.
     */
    public void configureShooterMotorTopLeft(TalonFXConfiguration shooterLeft) {
        //configure motor
        shooterLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26

        shooterLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        shooterLeft.MotorOutput.PeakForwardDutyCycle = 1;
        shooterLeft.MotorOutput.PeakReverseDutyCycle = -1;

        shooterLeft.CurrentLimits.StatorCurrentLimit = 180.0; //3-26 was 120
        shooterLeft.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLimit = 30;    //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Configs
        shooterLeft.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        shooterLeft.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterLeft.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterLeft.Slot0.kS = 0;
        shooterLeft.Slot0.kV = 0.145;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterLeft.Slot0.kA = 0;
        shooterLeft.Slot0.kG = 0.0;

        shooterLeft.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterLeft.Voltage.PeakForwardVoltage = 11;
        shooterLeft.Voltage.PeakReverseVoltage = -11;

        shooterLeft.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the base TalonFX configuration for the bottom left shooter motor.
     * @param shooterLeft - The TalonFX configuration to apply the settings to.
     */
    public void configureShooterMotorBottomLeft(TalonFXConfiguration shooterLeft) {
        //configure motor
        shooterLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26

        shooterLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        shooterLeft.MotorOutput.PeakForwardDutyCycle = 1;
        shooterLeft.MotorOutput.PeakReverseDutyCycle = -1;

        shooterLeft.CurrentLimits.StatorCurrentLimit = 180.0; //3-26 was 120
        shooterLeft.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLimit = 30;    //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Configs
        shooterLeft.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        shooterLeft.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterLeft.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterLeft.Slot0.kS = 0;
        shooterLeft.Slot0.kV = 0.145;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterLeft.Slot0.kA = 0;
        shooterLeft.Slot0.kG = 0.0;

        shooterLeft.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterLeft.Voltage.PeakForwardVoltage = 11;
        shooterLeft.Voltage.PeakReverseVoltage = -11;

        shooterLeft.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the base TalonFX configuration for the top right shooter motor.
     * @param shooterRight - The TalonFX configuration to apply the settings to.
     */
    public void configureShooterMotorTopRight(TalonFXConfiguration shooterRight) {
        //configure motor
        shooterRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //Set 2-17-26
        shooterRight.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Set 2-17-26
        shooterRight.MotorOutput.PeakForwardDutyCycle = 1;
        shooterRight.MotorOutput.PeakReverseDutyCycle = -1;

        shooterRight.CurrentLimits.StatorCurrentLimit = 180.0; // 3-26 was 120.
        shooterRight.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLimit = 30;    //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Configs
        shooterRight.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        shooterRight.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterRight.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterRight.Slot0.kS = 0;
        shooterRight.Slot0.kV = 0.145;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterRight.Slot0.kA = 0;
        shooterRight.Slot0.kG = 0;
        shooterRight.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterRight.Voltage.PeakForwardVoltage = 11;
        shooterRight.Voltage.PeakReverseVoltage = -11;

        shooterRight.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the base TalonFX configuration for the bottom right shooter motor.
     * @param shooterRight - The TalonFX configuration to apply the settings to.
     */
    public void configureShooterMotorBottomRight(TalonFXConfiguration shooterRight) {
        //configure motor
        shooterRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //Set 2-17-26
        shooterRight.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Set 2-17-26
        shooterRight.MotorOutput.PeakForwardDutyCycle = 1;
        shooterRight.MotorOutput.PeakReverseDutyCycle = -1;

        shooterRight.CurrentLimits.StatorCurrentLimit = 180.0; // 3-26 was 120.
        shooterRight.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLimit = 30;    //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Configs
        shooterRight.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        shooterRight.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterRight.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterRight.Slot0.kS = 0;
        shooterRight.Slot0.kV = 0.145;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterRight.Slot0.kA = 0;
        shooterRight.Slot0.kG = 0;
        shooterRight.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterRight.Voltage.PeakForwardVoltage = 11;
        shooterRight.Voltage.PeakReverseVoltage = -11;

        shooterRight.Audio.AllowMusicDurDisable = true;
    }
}
