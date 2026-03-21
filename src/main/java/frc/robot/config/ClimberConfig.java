package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConfig {
    //Constants go here
    public static final int kLiftClimbMotorCANID = 17;

    //Variables
    public TalonFXConfiguration liftClimbMotorConfig;
    public TalonFXConfiguration liftClimbMotorZeroConfig;
    public CANcoderConfiguration hookRotateCANcoderConfig;
    public CANcoderConfiguration liftCANcoderConfig;


    //Constructor
    public ClimberConfig(){
        liftClimbMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureLiftClimber(liftClimbMotorConfig);  //Fill in framework

        liftClimbMotorZeroConfig = new TalonFXConfiguration();
        this.configureLiftClimberZero(liftClimbMotorZeroConfig);

    }


    public void configureLiftClimber(TalonFXConfiguration liftClimber){
        //configure motor
        liftClimber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        liftClimber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        liftClimber.MotorOutput.PeakForwardDutyCycle = 1;
        liftClimber.MotorOutput.PeakReverseDutyCycle = -1;

        liftClimber.CurrentLimits.StatorCurrentLimit = 150; //30 for testing - 120 for actual climb
        liftClimber.CurrentLimits.StatorCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLimit = 40;    //5 for testing - 60 for actual climb
        liftClimber.CurrentLimits.SupplyCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLowerLimit = 40; //60 for testing
        liftClimber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        liftClimber.Slot0.kP = 5.0;  // An error of 1 rotation per second results in 2V output
        liftClimber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        liftClimber.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        liftClimber.Slot0.kS = 0;
        liftClimber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        liftClimber.Slot0.kA = 0;
        liftClimber.Slot0.kG = 0;
        liftClimber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        liftClimber.Voltage.PeakForwardVoltage = 11;
        liftClimber.Voltage.PeakReverseVoltage = -11;

        liftClimber.Audio.AllowMusicDurDisable = true;
    }

    public void configureLiftClimberZero(TalonFXConfiguration liftClimber){
        //configure motor
        liftClimber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        liftClimber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        liftClimber.MotorOutput.PeakForwardDutyCycle = 0.25;    // A Maximum output of 3 V would be a 25% duty cycle, assuming a 12 V input.
        liftClimber.MotorOutput.PeakReverseDutyCycle = -0.25;

        liftClimber.CurrentLimits.StatorCurrentLimit = 50.0;
        liftClimber.CurrentLimits.StatorCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLimit = 5.0;
        liftClimber.CurrentLimits.SupplyCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLowerLimit = 5.0;
        liftClimber.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        //Slot 0 Configs
        liftClimber.Slot0.kP = 30;  // An error of 1 rotation per second results in 2V output
        liftClimber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        liftClimber.Slot0.kD = 0.9;  // A change of 1 rotation per second squared results in 0.01 volts output
        liftClimber.Slot0.kS = 0;
        liftClimber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        liftClimber.Slot0.kA = 0;
        liftClimber.Slot0.kG = 0;
        liftClimber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        liftClimber.Voltage.PeakForwardVoltage = 3;
        liftClimber.Voltage.PeakReverseVoltage = -3;

        liftClimber.Audio.AllowMusicDurDisable = true;
    }

}
