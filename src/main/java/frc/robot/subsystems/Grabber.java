package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controllers the grabber component of the robot, by extending and retracting the pneumatics
 */
public class Grabber extends SubsystemBase {
    // solenoids controlling the pneumatic cylinders
    private Solenoid solenoid;

    public Grabber() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.GRABBER);
    }

    /**
     * toggle the states of the pistons, extends if contracted and contract if extended
     */
    public void toggle() {
        solenoid.toggle(); 
    }
}
