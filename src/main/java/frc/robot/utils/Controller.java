package frc.robot.utils;

import java.util.NoSuchElementException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * contains controller mappings and related methods. Alternatively {@link edu.wpi.first.wpilibj.XboxController} will
 * probably provide an equal or better way to get inputs, since the used {@link edu.wpi.first.wpilibj.Joystick}s are
 * meant for different denices
 */
public class Controller {
    // Controller Map:
    public static final int AXIS_RTRIGGER = 3;
    public static final int AXIS_LTRIGGER = 2;
    public static final int BUTTON_LTRIGGER = 5;
    public static final int BUTTON_RTRIGGER = 6;
    public static final int BUTTON_RBUMPER = BUTTON_RTRIGGER;
    public static final int BUTTON_LBUMPER = BUTTON_LTRIGGER;
    public static final int AXIS_LY = 1;
    public static final int AXIS_LX = 0;
    public static final int AXIS_RY = 5;
    public static final int AXIS_RX = 4;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;

    // joystick ports
    public static final int PRIMARY = 0;
    public static final int SECONDARY = 1;

    public static Joystick primaryJoystick = null;
    public static Joystick secondaryJoystick = null;

    /**
     * intializes primary and secondary joysticks
     */
    public static void init() {
        primaryJoystick = new Joystick(PRIMARY);
        secondaryJoystick = new Joystick(SECONDARY);
    }

    /**
     * helper method to get the corresponding joystick for a given player
     * 
     * @param player player ID (USB port in driver station)
     * @return corresponding joystick
     */
    private static Joystick fromPlayer(int player) {
        // https://docs.oracle.com/en/java/javase/19/language/switch-expressions.html
        return switch (player) {
            case PRIMARY -> primaryJoystick;
            case SECONDARY -> secondaryJoystick;
            default -> throw new NoSuchElementException("Invalid Player Controller requested");
        };
    }

    /**
     * Constructs a UserAnalog that precisely mirrors the axis, with no tranformation.
     * 
     * @param player - one of either PRIMARY or SECONDARY. This value is validated, and an invalid parameter will return
     *               a UserAnalog that always gets 0
     * @param axis   - The axis on the Xbox controller to grab values from. This parameter is not validated, so make
     *               sure you have a valid axis!
     * @return the UserAnalog instance
     */
    public static UserAnalog simpleAxis(int player, int axis) {
        Joystick joystick = fromPlayer(player);
        return () -> MathUtil.applyDeadband(joystick.getRawAxis(axis), 0.1);
    }

    /**
     * Constructs a UserDigital that precisely mirrors the button value, with no tranformation.
     * 
     * @param player - one of either PRIMARY or SECONDARY. This value is validated, and an invalid parameter will return
     *               a UserDigital that always gets false.
     * @param button - The button on the Xbox controller to grab values from. This parameter is not validated, so make
     *               sure you have a valid button!
     * @return the UserDigital instance
     */
    public static UserDigital simpleButton(int player, int button) {
        Joystick joystick = fromPlayer(player);
        return () -> joystick.getRawButton(button);
    }

    /**
     * get joystick button from player and button ids, secondary if invalid player is given
     * 
     * @param player player id (secondary by default)
     * @param button button id
     * @return the corresponding joystick button
     */
    public static JoystickButton getJoystickButton(int player, int button) {
        Joystick joystick = fromPlayer(player);
        return new JoystickButton(joystick, button);
    }
}