package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConfig {
    //Constants go here
    public static final int kHopperExtendCANID = 10;
   
    //Variables
    public TalonFXConfiguration hopperConfig;  //Create variable of type TalonFXConfiguration
    public TalonFXConfiguration hopperZeroConfig;

    /**
     * Constructor for the climber motor configuration.
     */
    public HopperConfig() {
        hopperConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureHopper(hopperConfig);         //Fill in framework 
    }

    /**
     * Sets up the base TalonFX configuration for the hopper motor.
     * @param hopper - The TalonFX configuration to apply the settings to.
     */
    public void configureHopper(TalonFXConfiguration hopper) {

        //super low current limit to create software spring

        //configure motor
        hopper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //Set 2-17-26
        hopper.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        hopper.MotorOutput.PeakForwardDutyCycle = 1;
        hopper.MotorOutput.PeakReverseDutyCycle = -1;

        hopper.CurrentLimits.StatorCurrentLimit = 120.0;  //70
        hopper.CurrentLimits.StatorCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLimit = 20.0;    //Set 2/25/26
        hopper.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLowerLimit = 15.0;  //Set 2/25/26
        hopper.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        hopper.Slot0.kP = 20;  // 8
        hopper.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        hopper.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        hopper.Slot0.kS = 0;
        hopper.Slot0.kV = 0.1666666;  // .1666666
        hopper.Slot0.kA = 0.05; //0.05
        hopper.Slot0.kG = 0;

        //Motion Magic
        hopper.MotionMagic.MotionMagicAcceleration = 80;
        hopper.MotionMagic.MotionMagicCruiseVelocity = 25;

        hopper.Voltage.PeakForwardVoltage = 11;
        hopper.Voltage.PeakReverseVoltage = -11;

        hopper.Audio.AllowMusicDurDisable = true;
    }

    /**
     * Sets up the TalonFX configuration for the hopper motor to use while running the zero-ing routine.
     * @param hopper - The TalonFX configuration to apply the settings to.
     */
    public void configureZeroHopper(TalonFXConfiguration hopper){

        //super low current limit to create software spring

        //configure motor
        hopper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //Set 2-17-26
        hopper.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        hopper.MotorOutput.PeakForwardDutyCycle = 1;
        hopper.MotorOutput.PeakReverseDutyCycle = -1;

        hopper.CurrentLimits.StatorCurrentLimit = 120.0;  //70
        hopper.CurrentLimits.StatorCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLimit = 30.0;    //Set 2/25/26
        hopper.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLowerLimit = 30.0;  //Set 2/25/26
        hopper.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        hopper.Slot0.kP = 20;  // 8
        hopper.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        hopper.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        hopper.Slot0.kS = 0;
        hopper.Slot0.kV = 0.1666666;  // .1666666
        hopper.Slot0.kA = 0.05; //0.05
        hopper.Slot0.kG = 0;

        //Motion Magic
        hopper.MotionMagic.MotionMagicAcceleration = 80;
        hopper.MotionMagic.MotionMagicCruiseVelocity = 30;

        hopper.Voltage.PeakForwardVoltage = 11;
        hopper.Voltage.PeakReverseVoltage = -11;

        hopper.Audio.AllowMusicDurDisable = true;
    }

}
