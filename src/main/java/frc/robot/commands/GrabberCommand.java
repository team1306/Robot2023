package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserDigital;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.utils.Controller;
import frc.robot.subsystems.Grabber;

/**
 * Command to drive the robot based on controller input
 */
public class GrabberCommand extends CommandBase {

    private UserDigital rightTrigger;
    private boolean isTriggered;
    private Grabber grabber;
    private boolean released;

    /**
     * initalizes drive command from given drivetrain, speed, and rotation
     * @param isTriggered initial trigger status
     */
    public GrabberCommand(
        Grabber grabber,
        boolean isTriggered
    ) {
        this.rightTrigger = Controller.simpleButton(1, 6);
        this.isTriggered = isTriggered;
        this.grabber = grabber;
        released = true;
        this.addRequirements(grabber);
    }

    /**
     * called repeatedly when command is schedules to run
     */

    @Override
    public void execute() {
        isTriggered = rightTrigger.get();

        if (isTriggered && released){
            grabber.toggle();
            released = false;
        } else if (released == false && isTriggered == false){
            released = true;
        }
    }
}
