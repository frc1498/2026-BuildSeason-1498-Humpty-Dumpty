package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConfig {
    //Constants go Here
    public static final int kShooterLeftMotorCANID = 14;
    public static final int kShooterRightMotorCANID = 15;
    public static final int kHoodMotorCANID = 16;
    public static final int kTurretMotorCANID = 13;
    public static final int kSpindexerMotorCANID = 11;
    public static final int kKickupMotorCANID = 12;
   
    //Variables
    public TalonFXConfiguration hoodMotorConfig; //x44 motor
    public TalonFXConfiguration shooterLeftMotorConfig; //kraken motor (x60)
    public TalonFXConfiguration shooterRightMotorConfig; //kraken motor (x60)
    public TalonFXConfiguration turretMotorConfig; //x44 motor
    public TalonFXConfiguration spindexerMotorConfig; //kraken motor (x60)
    public TalonFXConfiguration kickupMotorConfig; //kraken motor (x60)

    //Constructor - only runs one
    public ShooterConfig(){
        hoodMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureHoodMotor(hoodMotorConfig); //Fill in framework, requires a method below

        shooterLeftMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureShooterMotorLeftFollower(shooterLeftMotorConfig); //Fill in framework, requires a method below

        shooterRightMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureShooterMotorRightLeader(shooterRightMotorConfig); //Fill in framework, requires a method below

        turretMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureTurretMotor(turretMotorConfig); //Fill in framework, requires a method below

        spindexerMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureSpindexerMotor(spindexerMotorConfig); //Fill in framework, requires a method below

        kickupMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureKickupMotor(kickupMotorConfig); //Fill in framework, requires a method below

    }

    public void configureHoodMotor(TalonFXConfiguration hood){
        //Configure Motor
        hood.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;  //Set 2-17-26
        hood.MotorOutput.NeutralMode=NeutralModeValue.Brake;  //Set 2-17-26
        hood.MotorOutput.PeakForwardDutyCycle = 1;
        hood.MotorOutput.PeakReverseDutyCycle = -1;

        hood.CurrentLimits.StatorCurrentLimit = 120.0;
        hood.CurrentLimits.StatorCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26
        hood.CurrentLimits.SupplyCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLowerLimit = 20.0; //Set 2-17-26
        hood.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Config
        hood.Slot0.kP = 30;  // An error of 1 rotation per second results in 2V output
        hood.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        hood.Slot0.kD = 1.0;  // A change of 1 rotation per second squared results in 0.01 volts output
        hood.Slot0.kS = 0;
        hood.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        hood.Slot0.kA = 0;
        hood.Slot0.kG = 0;

        hood.Voltage.PeakForwardVoltage = 11;
        hood.Voltage.PeakReverseVoltage = -11;

        hood.Audio.AllowMusicDurDisable = true;
    }

    public void configureShooterMotorLeftFollower(TalonFXConfiguration shooterLeftFollower){
        //configure motor
        //shooterLeftFollower.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26
        //Left motor intvert is removed because, as a follower, it should follow the follower inversion settings

        shooterLeftFollower.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        shooterLeftFollower.MotorOutput.PeakForwardDutyCycle = 1;
        shooterLeftFollower.MotorOutput.PeakReverseDutyCycle = -1;

        shooterLeftFollower.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterLeftFollower.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLeftFollower.CurrentLimits.SupplyCurrentLimit = 60;    //Set 2-17-26
        shooterLeftFollower.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLeftFollower.CurrentLimits.SupplyCurrentLowerLimit = 80.0;  //Set 2-17-26
        shooterLeftFollower.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        shooterLeftFollower.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        shooterLeftFollower.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterLeftFollower.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterLeftFollower.Slot0.kS = 0;
        shooterLeftFollower.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterLeftFollower.Slot0.kA = 0;
        shooterLeftFollower.Slot0.kG = 0.0;

        shooterLeftFollower.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterLeftFollower.Voltage.PeakForwardVoltage = 11;
        shooterLeftFollower.Voltage.PeakReverseVoltage = -11;

        shooterLeftFollower.Audio.AllowMusicDurDisable = true;
    }

    public void configureShooterMotorRightLeader(TalonFXConfiguration shooterRightLeader){
        //configure motor
        shooterRightLeader.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //Set 2-17-26
        shooterRightLeader.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Set 2-17-26
        shooterRightLeader.MotorOutput.PeakForwardDutyCycle = 1;
        shooterRightLeader.MotorOutput.PeakReverseDutyCycle = -1;

        shooterRightLeader.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterRightLeader.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRightLeader.CurrentLimits.SupplyCurrentLimit = 60;    //Set 2-17-26
        shooterRightLeader.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterRightLeader.CurrentLimits.SupplyCurrentLowerLimit = 80.0;  //Set 2-17-26
        shooterRightLeader.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        shooterRightLeader.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        shooterRightLeader.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterRightLeader.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterRightLeader.Slot0.kS = 0;
        shooterRightLeader.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterRightLeader.Slot0.kA = 0;
        shooterRightLeader.Slot0.kG = 0;
        shooterRightLeader.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterRightLeader.Voltage.PeakForwardVoltage = 11;
        shooterRightLeader.Voltage.PeakReverseVoltage = -11;

        shooterRightLeader.Audio.AllowMusicDurDisable = true;
    }


    public void configureTurretMotor(TalonFXConfiguration turret){
        //configure motor
        turret.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //Set 2-17-26
        turret.MotorOutput.NeutralMode = NeutralModeValue.Brake; //Set 2-17-26
        turret.MotorOutput.PeakForwardDutyCycle = 1;
        turret.MotorOutput.PeakReverseDutyCycle = -1;

        turret.CurrentLimits.StatorCurrentLimit = 120.0;
        turret.CurrentLimits.StatorCurrentLimitEnable = true;
        turret.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26
        turret.CurrentLimits.SupplyCurrentLimitEnable = true;
        turret.CurrentLimits.SupplyCurrentLowerLimit = 20.0;  //Set 2-17-26
        turret.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        turret.Slot0.kP = 13;  // An error of 1 rotation per second results in 2V output
        turret.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        turret.Slot0.kD = 0.75;  // A change of 1 rotation per second squared results in 0.01 volts output
        turret.Slot0.kS = 0;
        turret.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        turret.Slot0.kA = 0;
        turret.Slot0.kG = 0.0;

        turret.Voltage.PeakForwardVoltage = 11;
        turret.Voltage.PeakReverseVoltage = -11;

        turret.Audio.AllowMusicDurDisable = true;
    }

        public void configureSpindexerMotor(TalonFXConfiguration spindexer){
        //configure motor
        spindexer.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26
        spindexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        spindexer.MotorOutput.PeakForwardDutyCycle = 1;
        spindexer.MotorOutput.PeakReverseDutyCycle = -1;

        spindexer.CurrentLimits.StatorCurrentLimit = 120.0;
        spindexer.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexer.CurrentLimits.SupplyCurrentLimit = 40;    //Set 2-17-26
        spindexer.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexer.CurrentLimits.SupplyCurrentLowerLimit = 40.0;  //Set 2-17-26
        spindexer.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        spindexer.Slot0.kP = 0.3;  // An error of 1 rotation per second results in 2V output
        spindexer.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        spindexer.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        spindexer.Slot0.kS = 0;
        spindexer.Slot0.kV = 0;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        spindexer.Slot0.kA = 0;
        spindexer.Slot0.kG = 0.0;

        spindexer.Voltage.PeakForwardVoltage = 11;
        spindexer.Voltage.PeakReverseVoltage = -11;

        spindexer.Audio.AllowMusicDurDisable = true;
    }

        public void configureKickupMotor(TalonFXConfiguration kickup){
        //configure motor
        kickup.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26
        kickup.MotorOutput.NeutralMode = NeutralModeValue.Brake;  //Set 2-17-26
        kickup.MotorOutput.PeakForwardDutyCycle = 1;
        kickup.MotorOutput.PeakReverseDutyCycle = -1;

        kickup.CurrentLimits.StatorCurrentLimit = 120.0;
        kickup.CurrentLimits.StatorCurrentLimitEnable = true;
        kickup.CurrentLimits.SupplyCurrentLimit = 40;    //Set 2-17-26
        kickup.CurrentLimits.SupplyCurrentLimitEnable = true;
        kickup.CurrentLimits.SupplyCurrentLowerLimit = 80.0;  //Set 2-17-26
        kickup.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        kickup.Slot0.kP = 10;  // An error of 1 rotation per second results in 2V output
        kickup.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        kickup.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        kickup.Slot0.kS = 8;
        kickup.Slot0.kV = 0.15;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        kickup.Slot0.kA = 0;
        kickup.Slot0.kG = 0.0;

        kickup.Voltage.PeakForwardVoltage = 11;
        kickup.Voltage.PeakReverseVoltage = -11;

        kickup.Audio.AllowMusicDurDisable = true;
    }

}
