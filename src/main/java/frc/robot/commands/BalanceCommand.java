package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to balance the robot based on navx roll angle
 */
public class BalanceCommand extends CommandBase {

    // out = KP * error + KI * accumErr + KD * deltaErr
    // note that error is in degrees (probably ~[-15,15]), and output is gonna be in[-1,1] so make constants quite small
    private static double KP = 0.0875;
    private static double KI = 0;
    private static double KD = 0.00008;

    private final double MaxOutput = 0.5;
    // TODO remove when constants are tuned
    static {
        SmartDashboard.putNumber("Balance kP", KP);
        SmartDashboard.putNumber("Balance kI", KI);
        SmartDashboard.putNumber("Balance kD", KD);
    }

    private static void updateConsts() {
        // for live testing sake
        // TODO remove below when constants are tuned
        KP = SmartDashboard.getNumber("Balance kP", KP);
        KI = SmartDashboard.getNumber("Balance kI", KI);
        KD = SmartDashboard.getNumber("Balance kD", KD);
    }


    private DriveTrain driveTrain;
    // maybe use a profiled pid controller for better control?
    private PIDController pid;

    public BalanceCommand(double idealAngle, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // todo remove below when tuned
        updateConsts();
        pid = new PIDController(KP, KI, KD);
        pid.setSetpoint(idealAngle);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        pid.reset();
        // todo remove below when tuned
        updateConsts();
        pid.setPID(KP, KI, KD);
        System.out.println(List.of(KP, KI, KD));
        pid.enableContinuousInput(-1, 1);
    }

    @Override
    public void execute() {
        // get current angle, and give that to the controller as feedback
        double currentAngle = -DriveTrain.gyro.getRoll();
        // calcuate output, and clamp to a reasonble angle
        var out = MathUtil.clamp(pid.calculate(currentAngle), -MaxOutput, MaxOutput);
        SmartDashboard.putNumber("balance pid out", out);
        driveTrain.arcadeDrive(out, 0);
    }

    // maybe?
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0, 0);
    }
}
