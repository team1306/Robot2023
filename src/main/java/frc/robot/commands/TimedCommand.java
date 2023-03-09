package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * template for any command that should run for some set amount of time
 */
public class TimedCommand extends CommandBase {
    private Timer timer;
    private double timeoutS;

    public TimedCommand(double timeoutS) {
        this.timeoutS = timeoutS;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeoutS);
    }
}
