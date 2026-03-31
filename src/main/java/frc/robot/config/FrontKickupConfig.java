package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class FrontKickupConfig {
    //Constants go Here
    public static final int kKickupMotorCANID = 12;
   
    //Variables
    public TalonFXConfiguration frontKickupMotorConfig; //kraken motor (x60)

    //Constructor - only runs one
    public FrontKickupConfig(){
        frontKickupMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureKickupMotor(frontKickupMotorConfig); //Fill in framework, requires a method below

    }
    
    public void configureKickupMotor(TalonFXConfiguration frontKickup){
        //configure motor
        frontKickup.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26
        frontKickup.MotorOutput.NeutralMode = NeutralModeValue.Brake;  //Set 2-17-26
        frontKickup.MotorOutput.PeakForwardDutyCycle = 1;
        frontKickup.MotorOutput.PeakReverseDutyCycle = -1;

        frontKickup.CurrentLimits.StatorCurrentLimit = 120.0;
        frontKickup.CurrentLimits.StatorCurrentLimitEnable = true;
        frontKickup.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26
        frontKickup.CurrentLimits.SupplyCurrentLimitEnable = true;
        frontKickup.CurrentLimits.SupplyCurrentLowerLimit = 20.0;  //Was 80.  Turned down for current consumption issues.
        frontKickup.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        frontKickup.Slot0.kP = 10;  // An error of 1 rotation per second results in 2V output
        frontKickup.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        frontKickup.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        frontKickup.Slot0.kS = 8;
        frontKickup.Slot0.kV = 0.15;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        frontKickup.Slot0.kA = 0;
        frontKickup.Slot0.kG = 0.0;

        frontKickup.Voltage.PeakForwardVoltage = 11;
        frontKickup.Voltage.PeakReverseVoltage = -11;

        frontKickup.Audio.AllowMusicDurDisable = true;
    }

}
