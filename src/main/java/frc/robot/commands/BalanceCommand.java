package frc.robot.commands;

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
    public static double KP = 0.00925;
    public static double KI = 0;
    public static double KD = 0.0005;

    private final double MaxOutput = 0.5;

    private DriveTrain driveTrain;
    // maybe use a profiled pid controller for better control?
    private PIDController pid;

    /**
     * creates a command using a PID controller to balance the robot using its orientation from the ideal angle as the
     * error
     * 
     * @param idealAngle target value of the roll angle
     * @param driveTrain drivetrain to move
     */
    public BalanceCommand(double idealAngle, DriveTrain driveTrain) {
        SmartDashboard.putNumber("ideal", idealAngle);
        this.driveTrain = driveTrain;
        pid = new PIDController(KP, KI, KD);
        pid.setSetpoint(idealAngle);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setPID(KP, KI, KD);
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
