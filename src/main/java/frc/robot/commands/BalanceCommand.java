package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to balance the robot based on navx roll angle
 */
public class BalanceCommand extends CommandBase {

    // out = KP * error + KI * accumErr + KD * deltaErr
    // note that error is in degrees (probably ~[-15,15]), and output is gonna be in[-1,1] so make constants quite small
    private static final double KP = 0.03;
    private static final double KI = 0;
    private static final double KD = 0.003;

    private double desiredAngle;
    private DriveTrain driveTrain;

    // accumulated and previous errors to compute derivative and integral components
    private double accumErr = 0;
    private double prevErr = Double.MAX_VALUE;

    // timer to compute delta time between executions
    private Timer timer = new Timer();
    private double prevTime = timer.get();

    public BalanceCommand(double idealAngle, DriveTrain driveTrain) {
        this.desiredAngle = idealAngle;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
        timer.start();
    }


    @Override
    public void initialize() {
        // reset timer and other values
        timer.reset();
        accumErr = 0;
        prevErr = Double.MAX_VALUE; // dummy value to indicate uninitialized
    }

    @Override
    public void execute() {
        double curAng = RobotContainer.navx.getRoll();
        double error = desiredAngle - curAng;

        // initialize previous error if necessary
        if (prevErr == Double.MAX_VALUE)
            prevErr = error;

        // compute error
        double deltaErr = (error - prevErr) / (timer.get() - prevTime);
        accumErr += error;

        // compute output from 3 components
        double output = KP * error + KI * accumErr + KD * deltaErr;

        // clamp in case of monkey business
        output = MathUtil.clamp(output, -1, 1);
        driveTrain.arcadeDrive(output, 0);

        // update values for next execution
        prevErr = error;
        prevTime = timer.get();
    }

    @Override
    public boolean isFinished() {
        // run in perpetuity until interrupted by unpressing
        return false;
    }
}
