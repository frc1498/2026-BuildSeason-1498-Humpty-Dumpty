package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodConfig {
    //Constants go Here
    public static final int kHoodMotorCANID = 16;

    //Variables
    public TalonFXConfiguration hoodMotorConfig; //x44 motor

    /**
     * Constructor for the hood motor configuration.
     */
    public HoodConfig(){
        hoodMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureHoodMotor(hoodMotorConfig); //Fill in framework, requires a method below
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
}
