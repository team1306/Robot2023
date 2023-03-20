package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to drive the robot based on controller input
 */
public class DriveCommand extends CommandBase {

    private DriveTrain driveTrain;
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog joystickRotation;

    /**
     * initalizes drive command from given drivetrain, speed, and rotation
     * 
     * @param driveTrain    drivetrain to bind
     * @param speed         initial speed
     * @param leftRotation  initial left rotation
     * @param rightRotation initial right rotation
     */
    public DriveCommand(
        DriveTrain driveTrain,
        UserAnalog backwardsTurbo,
        UserAnalog forwardTurbo,
        UserAnalog joystickRotation
    ) {
        this.driveTrain = driveTrain;
        this.backwardsTurbo = backwardsTurbo;
        this.forwardTurbo = forwardTurbo;
        this.joystickRotation = joystickRotation;
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

        // maxspeed and maxrotation checks moved into Drivetrain class
        driveTrain.arcadeDrive(
            spd,
            rotation
        );
    }
}
