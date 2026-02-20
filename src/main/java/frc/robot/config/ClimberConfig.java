package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ClimberConfig {
    //Constants go here
    public static final int kLiftClimbMotorCANID=17;
    public static final int kRotateClimb1MotorCANID=18;
    public static final int kRotateClimb2MotorCANID=19;
    //public static final int kLiftEncoderCANID=0;
    //public static final int kHookRotateEncoderCANID=0;

    //Variables
    public TalonFXConfiguration liftClimbMotorConfig;
    public TalonFXConfiguration rotateClimb1MotorConfig;
    public TalonFXConfiguration rotateClimb2MotorConfig;
    public CANcoderConfiguration hookRotateCANcoderConfig;
    public CANcoderConfiguration liftCANcoderConfig;


    //Constructor
    public ClimberConfig(){
        liftClimbMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureLiftClimber(liftClimbMotorConfig);  //Fill in framework 

        rotateClimb1MotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureRotateClimber1(rotateClimb1MotorConfig);  //Fill in framework 

        rotateClimb2MotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureRotateClimber2(rotateClimb2MotorConfig);  //Fill in framework 

        hookRotateCANcoderConfig = new CANcoderConfiguration();
        this.configureHookRotateCANcoder(hookRotateCANcoderConfig);

        liftCANcoderConfig = new CANcoderConfiguration();
        this.configureLiftCANcoder(liftCANcoderConfig);
    }
    
    public void configureHookRotateCANcoder(CANcoderConfiguration CANcoderConfig){
        CANcoderConfig.MagnetSensor.MagnetOffset = 0;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }

    public void configureLiftCANcoder(CANcoderConfiguration CANcoderConfig){
        CANcoderConfig.MagnetSensor.MagnetOffset = 0;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }

    public void configureLiftClimber(TalonFXConfiguration liftClimber){
        //configure motor
        liftClimber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        liftClimber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        liftClimber.MotorOutput.PeakForwardDutyCycle = 1;
        liftClimber.MotorOutput.PeakReverseDutyCycle = -1;

        liftClimber.CurrentLimits.StatorCurrentLimit = 120.0;
        liftClimber.CurrentLimits.StatorCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLimit = 20;    //Was 20
        liftClimber.CurrentLimits.SupplyCurrentLimitEnable = true;
        liftClimber.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        liftClimber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        liftClimber.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
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

    public void configureRotateClimber1(TalonFXConfiguration rotateClimber){
        //configure motor
        rotateClimber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotateClimber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotateClimber.MotorOutput.PeakForwardDutyCycle = 1;
        rotateClimber.MotorOutput.PeakReverseDutyCycle = -1;

        rotateClimber.CurrentLimits.StatorCurrentLimit = 120.0;
        rotateClimber.CurrentLimits.StatorCurrentLimitEnable = true;
        rotateClimber.CurrentLimits.SupplyCurrentLimit = 20;    //Was 20
        rotateClimber.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotateClimber.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        rotateClimber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        rotateClimber.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        rotateClimber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotateClimber.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotateClimber.Slot0.kS = 0;
        rotateClimber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rotateClimber.Slot0.kA = 0;
        rotateClimber.Slot0.kG = 0;
        rotateClimber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        rotateClimber.Voltage.PeakForwardVoltage = 11;
        rotateClimber.Voltage.PeakReverseVoltage = -11;

        rotateClimber.Audio.AllowMusicDurDisable = true;   
    }

    public void configureRotateClimber2(TalonFXConfiguration rotateClimber){
        //configure motor
        rotateClimber.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotateClimber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotateClimber.MotorOutput.PeakForwardDutyCycle = 1;
        rotateClimber.MotorOutput.PeakReverseDutyCycle = -1;

        rotateClimber.CurrentLimits.StatorCurrentLimit = 120.0;
        rotateClimber.CurrentLimits.StatorCurrentLimitEnable = true;
        rotateClimber.CurrentLimits.SupplyCurrentLimit = 20;    //Was 20
        rotateClimber.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotateClimber.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        rotateClimber.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        rotateClimber.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        rotateClimber.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotateClimber.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotateClimber.Slot0.kS = 0;
        rotateClimber.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rotateClimber.Slot0.kA = 0;
        rotateClimber.Slot0.kG = 0;
        rotateClimber.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        rotateClimber.Voltage.PeakForwardVoltage = 11;
        rotateClimber.Voltage.PeakReverseVoltage = -11;

        rotateClimber.Audio.AllowMusicDurDisable = true;   
    }


}
