package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RearKickupConfig {
    //Constants go Here
    public static final int kRearKickupMotorCANID = 19;
   
    //Variables
    public TalonFXConfiguration rearKickupMotorConfig; //kraken motor (x60)

    /**
     * Constructor for the rear kickup motor configuration.
     */
    public RearKickupConfig(){
        rearKickupMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureRearKickupMotor(rearKickupMotorConfig); //Fill in framework, requires a method below
    }

    /**
     * Sets up the base TalonFX configuration for the rear kickup motor.
     * @param rearKickup - The TalonFX configuration to apply the settings to.
     */
    public void configureRearKickupMotor(TalonFXConfiguration rearKickup){
        //configure motor
        rearKickup.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 4-10
        rearKickup.MotorOutput.NeutralMode = NeutralModeValue.Brake;  //Set 4-10
        rearKickup.MotorOutput.PeakForwardDutyCycle = 1;
        rearKickup.MotorOutput.PeakReverseDutyCycle = -1;

        rearKickup.CurrentLimits.StatorCurrentLimit = 180.0;
        rearKickup.CurrentLimits.StatorCurrentLimitEnable = true;
        rearKickup.CurrentLimits.SupplyCurrentLimit = 15;    //Set 2-17-26
        rearKickup.CurrentLimits.SupplyCurrentLimitEnable = true;
        rearKickup.CurrentLimits.SupplyCurrentLowerLimit = 15.0;  //Was 80.  Turned down for current consumption issues.
        rearKickup.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Configs
        rearKickup.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        rearKickup.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rearKickup.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rearKickup.Slot0.kS = 8;
        rearKickup.Slot0.kV = 0.127;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rearKickup.Slot0.kA = 0;
        rearKickup.Slot0.kG = 0.0;

        rearKickup.Voltage.PeakForwardVoltage = 11;
        rearKickup.Voltage.PeakReverseVoltage = -11;

        rearKickup.Audio.AllowMusicDurDisable = true;
    }

}
