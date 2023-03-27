package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controllers the grabber component of the robot, by extending and retracting the pneumatics
 */
public class Grabber extends SubsystemBase {
    // solenoids controlling the pneumatic cylinders
    private DoubleSolenoid dsol;

    public Grabber() {
        dsol = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.GRABBER_EXTEND,
            Constants.GRABBER_RETRACT
        );
    }

    /**
     * toggle the states of the pistons, extends if contracted and contract if extended
     */
    public void set(boolean value) {
        dsol.set(value ? Value.kForward : Value.kReverse);
    }

    public void shutOff() {
        dsol.set(Value.kOff);
    }
}
