package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FloorConfig {
    //Constants go Here
    public static final int kFloorMotorCANID = 11;

    //Variables
    public TalonFXConfiguration floorMotorConfig; //x60 motor


    /**
     * Constructor for the floor roller motor configuration.
     */
    public FloorConfig() {
        floorMotorConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureFloorMotor(floorMotorConfig);     //Fill in framework, requires a method below
    }

    /**
     * Sets up the base TalonFX configuration for the floor roller motor.
     * @param floor - The TalonFX configuration to apply the settings to.
     */
    public void configureFloorMotor(TalonFXConfiguration floor) {
        //Configure Motor
        floor.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;  //Set 2-17-26
        floor.MotorOutput.NeutralMode=NeutralModeValue.Coast;  //Set 2-17-26
        floor.MotorOutput.PeakForwardDutyCycle = 1;
        floor.MotorOutput.PeakReverseDutyCycle = -1;

        floor.CurrentLimits.StatorCurrentLimit = 180;  //Set 4-10
        floor.CurrentLimits.StatorCurrentLimitEnable = true;
        floor.CurrentLimits.SupplyCurrentLimit = 20;    //Set 4-10
        floor.CurrentLimits.SupplyCurrentLimitEnable = true;
        floor.CurrentLimits.SupplyCurrentLowerLimit = 20; //Set 4-10
        floor.CurrentLimits.SupplyCurrentLowerTime = 0;

        //Slot 0 Config
        floor.Slot0.kP = 1.2;  // An error of 1 rotation per second results in 2V output
        floor.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        floor.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        floor.Slot0.kS = 0;
        floor.Slot0.kV = .127;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        floor.Slot0.kA = 0;
        floor.Slot0.kG = 0;

        floor.Voltage.PeakForwardVoltage = 11;
        floor.Voltage.PeakReverseVoltage = -11;

        floor.Audio.AllowMusicDurDisable = true;
    }

}
