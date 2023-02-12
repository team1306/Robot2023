package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
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

    private DriveTrain driveTrain;
    private PIDController pid;

    public BalanceCommand(double idealAngle, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        pid = new PIDController(KP, KI, KD);
        pid.setSetpoint(idealAngle);
        addRequirements(driveTrain);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double curAng = RobotContainer.navx.getRoll();

        var out = MathUtil.clamp(pid.calculate(curAng), -1, 1);
        System.out.printf("%.02f : %.02f\n", pid.getSetpoint(), out);
        driveTrain.arcadeDrive(out, 0);
    }

    @Override
    public boolean isFinished() {
        // run in perpetuity until interrupted by unpressing
        return false;
    }
}
