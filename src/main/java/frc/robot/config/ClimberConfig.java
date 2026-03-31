package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

    //Constructor
    public ClimberConfig(){
        ClimbMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureClimber(ClimbMotorConfig);  //Fill in framework

        ClimbMotorZeroConfig = new TalonFXConfiguration();
        this.configureClimberZero(ClimbMotorZeroConfig);

    }


    public void configureClimber(TalonFXConfiguration Climber){
        //configure motor
        Climber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        Climber.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Climber.MotorOutput.PeakForwardDutyCycle = 1;
        Climber.MotorOutput.PeakReverseDutyCycle = -1;

        Climber.CurrentLimits.StatorCurrentLimit = 150; //30 for testing - 120 for actual climb
        Climber.CurrentLimits.StatorCurrentLimitEnable = true;
        Climber.CurrentLimits.SupplyCurrentLimit = 40;    //5 for testing - 60 for actual climb
        Climber.CurrentLimits.SupplyCurrentLimitEnable = true;
        Climber.CurrentLimits.SupplyCurrentLowerLimit = 40; //60 for testing
        Climber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        Climber.Slot0.kP = 5.0;  // An error of 1 rotation per second results in 2V output
        Climber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        Climber.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        Climber.Slot0.kS = 0;
        Climber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        Climber.Slot0.kA = 0;
        Climber.Slot0.kG = 0;
        Climber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        Climber.Voltage.PeakForwardVoltage = 11;
        Climber.Voltage.PeakReverseVoltage = -11;

        Climber.Audio.AllowMusicDurDisable = true;
    }

    public void configureClimberZero(TalonFXConfiguration Climber){
        //configure motor
        Climber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        Climber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Climber.MotorOutput.PeakForwardDutyCycle = 0.25;    // A Maximum output of 3 V would be a 25% duty cycle, assuming a 12 V input.
        Climber.MotorOutput.PeakReverseDutyCycle = -0.25;

        Climber.CurrentLimits.StatorCurrentLimit = 50.0;
        Climber.CurrentLimits.StatorCurrentLimitEnable = true;
        Climber.CurrentLimits.SupplyCurrentLimit = 5.0;
        Climber.CurrentLimits.SupplyCurrentLimitEnable = true;
        Climber.CurrentLimits.SupplyCurrentLowerLimit = 5.0;
        Climber.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        //Slot 0 Configs
        Climber.Slot0.kP = 30;  // An error of 1 rotation per second results in 2V output
        Climber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        Climber.Slot0.kD = 0.9;  // A change of 1 rotation per second squared results in 0.01 volts output
        Climber.Slot0.kS = 0;
        Climber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        Climber.Slot0.kA = 0;
        Climber.Slot0.kG = 0;
        Climber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        Climber.Voltage.PeakForwardVoltage = 3;
        Climber.Voltage.PeakReverseVoltage = -3;

        Climber.Audio.AllowMusicDurDisable = true;
    }

}
