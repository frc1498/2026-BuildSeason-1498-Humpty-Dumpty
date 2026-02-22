package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConfig {
    //Constants go here
    public static final int kHopperExtendCANID = 10;
   
    //Variables
    public TalonFXConfiguration hopperConfig;  //Create variable of type TalonFXConfiguration

    //Constructor
    public HopperConfig(){
        hopperConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureHopper(hopperConfig);  //Fill in framework 
    }

      public void configureHopper(TalonFXConfiguration hopper){

        //super low current limit to create software spring

        //configure motor
        hopper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //Set 2-17-26
        hopper.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        hopper.MotorOutput.PeakForwardDutyCycle = 1;
        hopper.MotorOutput.PeakReverseDutyCycle = -1;

        hopper.CurrentLimits.StatorCurrentLimit = 120.0;
        hopper.CurrentLimits.StatorCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26
        hopper.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopper.CurrentLimits.SupplyCurrentLowerLimit = 20.0;  //Set 2-17-26
        hopper.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        hopper.Slot0.kP = 28;  // An error of 1 rotation per second results in 2V output
        hopper.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        hopper.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        hopper.Slot0.kS = 0;
        hopper.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        hopper.Slot0.kA = 0;
        hopper.Slot0.kG = 0;

        hopper.Voltage.PeakForwardVoltage = 11;
        hopper.Voltage.PeakReverseVoltage = -11;

        hopper.Audio.AllowMusicDurDisable = true;

    }

}
