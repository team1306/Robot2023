package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to drive the robot based on controller input
 */
public class DriveCommand extends CommandBase {
    // turbo (no speed limit by default; good movement easily early in game)
    public boolean turbo = true;
    private DriveTrain driveTrain;
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog joystickRotation;

    /**
     * initlizes general driving command for the drivetrain object
     * 
     * @param driveTrain drivetrain to bind
     * @param backwards  backwards input, subtracted from forwards to get general movement input
     * @param forwards   forwards input
     * @param rotation   rotation input -- how fast to spin
     */
    public DriveCommand(
        DriveTrain driveTrain,
        UserAnalog backwards,
        UserAnalog forwards,
        UserAnalog rotation
    ) {
        this.driveTrain = driveTrain;
        this.backwardsTurbo = backwards;
        this.forwardTurbo = forwards;
        this.joystickRotation = rotation;
        this.addRequirements(driveTrain);
    }

    /**
     * called repeatedly when command is schedules to run
     */
    @Override
    public void execute() {
        // apply deadband to both inputs
        double spd = MathUtil.applyDeadband(forwardTurbo.get() - backwardsTurbo.get(), 0.05);
        double rotation = MathUtil.applyDeadband(joystickRotation.get(), 0.05);

        // turbo or balancing speed
        double maxSpeed = turbo ? 1 : 0.3;
        double maxRotation = turbo ? 1 : 0.3;
        // maxspeed and maxrotation checks moved into Drivetrain class
        driveTrain.arcadeDrive(spd * maxSpeed, rotation * maxRotation);
    }
}
