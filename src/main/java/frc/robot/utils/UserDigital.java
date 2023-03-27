package frc.robot.utils;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * interface for digital (boolean) inputs to the robot, essentially a tailored version of a
 * {@link java.util.function.BooleanSupplier}
 */
public interface UserDigital {

    /**
     * @return value - a boolean value based on the input from the user or controller.
     */
    public boolean get();

    /**
     * Constructs a UserDigital from a UserAnalog using thresholded values
     * 
     * @param analog    - getter for the analog value
     * @param threshold - if analog val above or equal threshold = true, below=false
     * @param flip      - if true, flips the threshold from above = true to above = false
     * @return the constructed object
     */
    public static UserDigital fromAnalog(UserAnalog analog, double threshold, boolean flip) {
        return () -> (analog.get() >= threshold) ^ flip;
    }

    /**
     * creates a boolean event from the the given UserDigital values; boolean events contain many useful features that
     * go beyond polling, such as supporting toggling (rise/fall), debouncing, and composition with other BooleanEvents
     * 
     * @param m_Loop the command loop to which the event is going to be added.
     * @return a boolean event representing the given userdigital value
     */
    default BooleanEvent asBooleanEvent(EventLoop m_Loop) {
        return new BooleanEvent(m_Loop, () -> this.get());
    }

    /**
     * creates trigger from given UserDigital. Triggers have much of the features of BooleanEvents. However, unlike
     * boolean events, triggers work directly with commands instead of just with side-effectful functions.
     * 
     * @return resulting trigger value
     */
    default Trigger asTrigger() {
        return new Trigger(() -> this.get());
    }
}