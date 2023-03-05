package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.utils.MotorUtils.*;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        leftLeader = initSparkMax(SPARK_FAR_LEFT);
        leftFollower = initSparkMax(SPARK_NEAR_LEFT);
        rightLeader = initSparkMax(SPARK_FAR_RIGHT);
        rightFollower = initSparkMax(SPARK_NEAR_RIGHT);

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

        SmartDashboard.putNumber("left", leftMotorOutput);
        SmartDashboard.putNumber("right", -rightMotorOutput);
        // invert right output to account for motors being flipped around on robots
        // since followers are following leaders, they will also have these outputs
        leftLeader.set(leftMotorOutput);
        rightLeader.set(-rightMotorOutput);
    }

    // test individual sides for testings (best description poggers)
    public void testDrive(double left, double right) {
        leftLeader.set(left);
        rightLeader.set(right);
    }

    @Override
    public void close() throws Exception {
        leftLeader.close();
        rightLeader.close();
        leftFollower.close();
        rightFollower.close();
    }
}
