package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controllers the grabber component of the robot, by extending and retracting the pneumatics
 */
public class Grabber extends SubsystemBase {
    // solenoids controlling the pneumatic cylinders
    private Solenoid leftCylinder, rightCylinder;
    // whether the pistons are extended
    private boolean extended;

    public Grabber() {
        leftCylinder = new Solenoid(PneumaticsModuleType.REVPH, 0);
        rightCylinder = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    /**
     * toggle the states of the pistons, extends if contracted and contract if extended
     */
    public void toggle() {
        extended = !extended;
        leftCylinder.set(extended);
        rightCylinder.set(extended);
    }
}
