package frc.robot.util.Mechanism;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class ServoMechanism {

    private TalonFX servo;
    private TalonFXConfiguration config;

    public ServoMechanism() {

    }

    public void initialize(int CANID, CANBus CANBus) {
        this.servo = new TalonFX(CANID, CANBus);
    }

    public void configure(TalonFXConfiguration config) {
        this.config = config;
        // Start Configuring the motor with the supplied configuration.
        StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

        // Attempt to apply the configuration 5 times.  Immediately stop if the configuration was successful.
        for(int i = 0; i < 5; ++i) {
            mechanismStatus = this.servo.getConfigurator().apply(config);
            if (mechanismStatus.isOK()) {break;}
        }
        // If the configuration was still not successful, print an error to the console.
        if (!mechanismStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
        }
    }

    

}
