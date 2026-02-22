package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfig {
    //Constants go here
    public static final int kIntakeCANID = 9;
   
    //Variables
    public TalonFXConfiguration intakeConfig;  //Create variable of type TalonFXConfiguration

    //Constructor
    public IntakeConfig(){
        intakeConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureIntake(intakeConfig);  //Fill in framework 
    }

     public void configureIntake(TalonFXConfiguration intake){

        //configure motor
        intake.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intake.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intake.MotorOutput.PeakForwardDutyCycle = 1;
        intake.MotorOutput.PeakReverseDutyCycle = -1;

        intake.CurrentLimits.StatorCurrentLimit = 120.0;
        intake.CurrentLimits.StatorCurrentLimitEnable = true;
        intake.CurrentLimits.SupplyCurrentLimit = 20;    //Was 20
        intake.CurrentLimits.SupplyCurrentLimitEnable = true;
        intake.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        intake.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        intake.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        intake.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        intake.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        intake.Slot0.kS = 0;
        intake.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        intake.Slot0.kA = 0;
        intake.Slot0.kG = 0;

        intake.Voltage.PeakForwardVoltage = 11;
        intake.Voltage.PeakReverseVoltage = -11;

        intake.Audio.AllowMusicDurDisable = true;
    }


}
