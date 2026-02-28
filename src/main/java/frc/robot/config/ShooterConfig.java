package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        this.configureShooterMotorLeft(shooterLeftMotorConfig); //Fill in framework, requires a method below

        shooterRightMotorConfig = new TalonFXConfiguration(); //Instantiate - make a framework
        this.configureShooterMotorRight(shooterRightMotorConfig); //Fill in framework, requires a method below

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
        hood.MotorOutput.NeutralMode=NeutralModeValue.Coast;  //Set 2-17-26
        hood.MotorOutput.PeakForwardDutyCycle = 1;
        hood.MotorOutput.PeakReverseDutyCycle = -1;

        hood.CurrentLimits.StatorCurrentLimit = 120;  //Set 2-17-26 to 120
        hood.CurrentLimits.StatorCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26 to 20
        hood.CurrentLimits.SupplyCurrentLimitEnable = true;
        hood.CurrentLimits.SupplyCurrentLowerLimit = 20; //Set 2-17-26 to 20
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

    public void configureShooterMotorLeft(TalonFXConfiguration shooterLeft){
        //configure motor
        shooterLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  //Set 2-17-26

        shooterLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;  //Set 2-17-26
        shooterLeft.MotorOutput.PeakForwardDutyCycle = 1;
        shooterLeft.MotorOutput.PeakReverseDutyCycle = -1;

        shooterLeft.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterLeft.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLimit = 60;    //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLeft.CurrentLimits.SupplyCurrentLowerLimit = 80.0;  //Set 2-17-26
        shooterLeft.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        shooterLeft.Slot0.kP = 13;  // An error of 1 rotation per second results in 2V output
        shooterLeft.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterLeft.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterLeft.Slot0.kS = 5;
        shooterLeft.Slot0.kV = 0.59;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterLeft.Slot0.kA = 0;
        shooterLeft.Slot0.kG = 0.0;

        shooterLeft.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterLeft.Voltage.PeakForwardVoltage = 11;
        shooterLeft.Voltage.PeakReverseVoltage = -11;

        shooterLeft.Audio.AllowMusicDurDisable = true;
    }

    public void configureShooterMotorRight(TalonFXConfiguration shooterRight){
        //configure motor
        shooterRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //Set 2-17-26
        shooterRight.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Set 2-17-26
        shooterRight.MotorOutput.PeakForwardDutyCycle = 1;
        shooterRight.MotorOutput.PeakReverseDutyCycle = -1;

        shooterRight.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterRight.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLimit = 60;    //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterRight.CurrentLimits.SupplyCurrentLowerLimit = 80.0;  //Set 2-17-26
        shooterRight.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        shooterRight.Slot0.kP = 13;  // An error of 1 rotation per second results in 2V output
        shooterRight.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        shooterRight.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        shooterRight.Slot0.kS = 5;
        shooterRight.Slot0.kV = 0.59;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        shooterRight.Slot0.kA = 0;
        shooterRight.Slot0.kG = 0;
        shooterRight.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        shooterRight.Voltage.PeakForwardVoltage = 11;
        shooterRight.Voltage.PeakReverseVoltage = -11;

        shooterRight.Audio.AllowMusicDurDisable = true;
    }


    public void configureTurretMotor(TalonFXConfiguration turret){
        //configure motor
        turret.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //Set 2-17-26
        turret.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Set 2-17-26
        turret.MotorOutput.PeakForwardDutyCycle = 1;
        turret.MotorOutput.PeakReverseDutyCycle = -1;

        turret.CurrentLimits.StatorCurrentLimit = 120; //Set 2-17-26 at 120 amps
        turret.CurrentLimits.StatorCurrentLimitEnable = true;
        turret.CurrentLimits.SupplyCurrentLimit = 20;    //Set 2-17-26 at 20 amps
        turret.CurrentLimits.SupplyCurrentLimitEnable = true;
        turret.CurrentLimits.SupplyCurrentLowerLimit = 20;  //Set 2-17-26 at 20 amps
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
        spindexer.Slot0.kP = 0.5;  // An error of 1 rotation per second results in 2V output
        spindexer.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        spindexer.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        spindexer.Slot0.kS = 0;
        spindexer.Slot0.kV = 0.123076926;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        spindexer.Slot0.kA = .0266666;
        spindexer.Slot0.kG = 0.0;

        spindexer.Voltage.PeakForwardVoltage = 11;
        spindexer.Voltage.PeakReverseVoltage = -11;

        spindexer.Audio.AllowMusicDurDisable = true;

        spindexer.MotionMagic.MotionMagicAcceleration=150;
        spindexer.MotionMagic.MotionMagicCruiseVelocity=80;
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
