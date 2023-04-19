package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to drive the robot based on controller input
 */
public class DriveCommand extends CommandBase {
    // turbo (no speed limit by default; good movement easily early in game)
    public boolean turbo = true;
    private static boolean IS_OUTREACH = true;
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

        SmartDashboard.putNumber("maxSpd", 0.3);
        SmartDashboard.putNumber("maxRot", 0.3);
    }

    /**
     * called repeatedly when command is schedules to run
     */
    @Override
    public void execute() {
        // apply deadband to both inputs

        double maxSpd = SmartDashboard.getNumber("maxSpd", 0.3);
        double maxRot = SmartDashboard.getNumber("maxRot", 0.3);



        double spd = MathUtil.applyDeadband(forwardTurbo.get() - backwardsTurbo.get(), 0.05);
        double rotation = MathUtil.applyDeadband(joystickRotation.get(), 0.05);

        // turbo or balancing speed
        double maxSpeed = turbo && !IS_OUTREACH ? 1 : maxSpd;
        double maxRotation = turbo && !IS_OUTREACH ? 1 : maxRot;

        System.out.println(List.of(maxSpeed, maxRotation));
        // maxspeed and maxrotation checks moved into Drivetrain class
        driveTrain.arcadeDrive(spd * maxSpeed, rotation * maxRotation);
    }
}
