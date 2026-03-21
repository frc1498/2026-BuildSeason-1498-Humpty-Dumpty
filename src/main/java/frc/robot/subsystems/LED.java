package frc.robot.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorEnableConstants;

public class LED extends SubsystemBase{

    /** State enumeration for the LEDs.  Based on the state of the robot, the color and flash pattern of the LEDs should change. */
    private enum LEDState {
        IDLE(new RGBWColor(255, 255, 255, 0)),
        SPINUP(new RGBWColor(255, 255, 255, 0)),
        SHOOTING(new RGBWColor(255, 255, 255, 0)),
        CLIMBING(new RGBWColor(255, 255, 255, 0));

        private RGBWColor color;

        /** Set the LED color associated with the state. */
        LEDState(RGBWColor color) {
            this.color = color;
        }

        /**Return the color associated with the state. */
        public RGBWColor color() {
            return this.color;
        }
    }

    /** Create a new LED subsystem. */
    public LED() {

    }

    /**
     * Logs variables from the subsystem via DogLog.  The amount of variables logged can be controlled with the logLevel parameter.
     * @param logLevel - The level of logging to enable.
     */
    private void log(MotorEnableConstants.LogLevel logLevel) {
        switch (logLevel) {
        case NONE:
            break;
        case FULL:
            break;
        default:
            break;
        }
    }
    
    /*private Command setColor(LEDState state) {
        runOnce(() -> {
            candle.set(state.color,)
        });
    }*/

}
