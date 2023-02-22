package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.utils.MotorUtils.*;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Used by DriveTrain command to move robot Calculates output for each side of the drivetrain
 */
public class DriveTrain extends SubsystemBase implements AutoCloseable {
    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;

    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;

    /**
     * Initializing drive train and talonFX settings
     */
    public DriveTrain() {
        // TODO find actual ids when drivetrain is built
        leftLeader = initSparkMax(SPARK_CENTER);
        leftFollower = initSparkMax(SPARK_CENTER);
        rightLeader = initSparkMax(SPARK_CENTER);
        rightFollower = initSparkMax(SPARK_CENTER);

        // have the followers follow the leaders (they will output the same values)
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
    }

    /**
     * Sets the talonFX speeds for the given speed and rotation
     * 
     * @param speed    speed from a joystick input
     * @param rotation rotation from joystick triggers
     */
    public void arcadeDrive(double speed, double rotation) {
        // clamp inputs for no funny business
        speed = MathUtil.clamp(speed, -1, 1);
        rotation = MathUtil.clamp(rotation, -1, 1);

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        double leftMotorOutput, rightMotorOutput;

        // speed is -1 to 1, rotation is also -1 to 1
        if (speed >= 0) {
            if (rotation >= 0) {
                // Quadrant 1 (+R, +S)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                // Quadrant 2 (-R, +S)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rotation >= 0) {
                // Quadrant 4 (+R, -S)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                // Quadrant 3 (-R, -S)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }

        // invert right output to account for motors being flipped around on robots
        // since followers are following leaders, they will also have these outputs
        leftLeader.set(leftMotorOutput);
        rightLeader.set(-rightMotorOutput);
    }

    @Override
    public void close() throws Exception {
        leftLeader.close();
        rightLeader.close();
        leftFollower.close();
        rightFollower.close();
    }
}
