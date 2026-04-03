package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfig {
    //Constants go here
    public static final int kIntakeRightCANID = 9;
    public static final int kIntakeLeftCANID = 20;
   
    //Variables
    public TalonFXConfiguration intakeRightMotorConfig;  //Create variable of type TalonFXConfiguration
    public TalonFXConfiguration intakeLeftMotorConfig;  //Create variable of type TalonFXConfiguration

    /**
     * Constructor for the climber motor configuration.
     */
    public IntakeConfig() {
        intakeRightMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureIntakeMotorRight(intakeRightMotorConfig);  //Fill in framework 

        intakeLeftMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureIntakeMotorLeft(intakeLeftMotorConfig);  //Fill in framework 
    }

    /**
     * Sets up the base TalonFX configuration for the right intake motor.
     * @param intakeRight - The TalonFX configuration to apply the settings to.
     */
     public void configureIntakeMotorRight(TalonFXConfiguration intakeRight) {

        //configure motor
        intakeRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRight.MotorOutput.PeakForwardDutyCycle = 1;
        intakeRight.MotorOutput.PeakReverseDutyCycle = -1;

        intakeRight.CurrentLimits.StatorCurrentLimit = 150;  //Updated 2-24-26 - is correct
        intakeRight.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRight.CurrentLimits.SupplyCurrentLimit = 40.0;    //Changed 3/26 to try to help current consumption. Was 60.
        intakeRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRight.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Changed 3/26 to try to help current consumption. Was 60.
        intakeRight.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        intakeRight.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        intakeRight.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        intakeRight.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        intakeRight.Slot0.kS = 0;
        intakeRight.Slot0.kV = 0.14;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        intakeRight.Slot0.kA = 0;
        intakeRight.Slot0.kG = 0;

        intakeRight.Voltage.PeakForwardVoltage = 11;
        intakeRight.Voltage.PeakReverseVoltage = -11;

        intakeRight.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the base TalonFX configuration for the left intake motor.
     * @param intakeLeft - The TalonFX configuration to apply the settings to.
     */
    public void configureIntakeMotorLeft(TalonFXConfiguration intakeLeft) {

        //configure motor
        intakeLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeLeft.MotorOutput.PeakForwardDutyCycle = 1;
        intakeLeft.MotorOutput.PeakReverseDutyCycle = -1;

        intakeLeft.CurrentLimits.StatorCurrentLimit = 150;  //Updated 2-24-26 - is correct
        intakeLeft.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeLeft.CurrentLimits.SupplyCurrentLimit = 40;    //Updated 2-24-26 - is correct
        intakeLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeLeft.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
        intakeLeft.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        intakeLeft.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        intakeLeft.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        intakeLeft.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        intakeLeft.Slot0.kS = 0;
        intakeLeft.Slot0.kV = 0.14;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        intakeLeft.Slot0.kA = 0;
        intakeLeft.Slot0.kG = 0;

        intakeLeft.Voltage.PeakForwardVoltage = 11;
        intakeLeft.Voltage.PeakReverseVoltage = -11;

        intakeLeft.Audio.AllowMusicDurDisable = true;
    }

}
