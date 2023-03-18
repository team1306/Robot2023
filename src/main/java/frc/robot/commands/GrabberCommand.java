package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.utils.UserDigital;

public class GrabberCommand extends CommandBase {
    private Grabber grabber;
    private UserDigital input;
    // create a debouncer
    private Debouncer deb = new Debouncer(0.25);

    public GrabberCommand(Grabber grabber, UserDigital input) {
        this.grabber = grabber;
        this.input = input;
    }

    @Override
    public void execute() {

    }

}
