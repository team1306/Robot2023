package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.utils.MotorUtils.*;

import java.util.concurrent.locks.LockSupport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Used by DriveTrain command to move robot Calculates output for each side of the drivetrain
 */
@SuppressWarnings("unused")
public class DriveTrain extends SubsystemBase implements AutoCloseable {
    private WPI_TalonSRX leftLeader;
    private WPI_VictorSPX leftFollower;

    private WPI_TalonSRX rightLeader;
    private WPI_VictorSPX rightFollower;
    private final DifferentialDriveOdometry m_odometry;
    private PIDController controller;

    private BuiltInAccelerometer accelerometer;

    /**
     * Initializing drive train and talonmFX settings
     */
    public DriveTrain() {
        leftLeader = initWPITalonSRX(LEFT_DRIVETRAIN_TALON);
        rightLeader = initWPITalonSRX(RIGHT_DRIVETRAIN_TALON);
        leftFollower = initWPIVictorSPX(LEFT_DRIVETRAIN_VICTOR);
        rightFollower = initWPIVictorSPX(RIGHT_DRIVETRAIN_VICTOR);

        accelerometer = new BuiltInAccelerometer();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    }

    /**
     * Sets the talonFX speeds for the given speed and rotation
     * 
     * @param speed    speed from a joystick input
     * @param rotation rotation from joystick triggers
     */
    public void arcadeDrive(double speed, double rotation) {
        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        double leftMotorOutput, rightMotorOutput;

        // speed is -1 to 1, rotation is also -1 to 1
        if (speed >= 0) {
            if (rotation >= 0) {
                // Quadrant 1 (R > 0, S > 0)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                // Quadrant 2 (R < 0, S > 0)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rotation >= 0) {
                // Quadrant 4 (R > 0, S < 0)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                // Quadrant 3 (R < 0, S < 0)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }

        // invert right output to account for motors being flipped around on robots
        leftLeader.set(leftMotorOutput);
        rightLeader.set(-rightMotorOutput);
    }

    /**
     * take encoder ticks displacement of left wheels and convert into meters
     * 
     * @return displacement of left wheels in meters
     */
    public double getLeftDistance() {
        double encoderTicks = leftLeader.getSelectedSensorPosition();
        return encoderTicks / 2048 / 5.88 * Constants.WHEEL_CIRCUMFERENCE;
    }

    /**
     * Take encoder ticks displacement of right wheels and convert into meters
     * 
     * @return displacement of right wheels in meters
     */
    public double getRightDistance() {
        double encoderTicks = rightLeader.getSelectedSensorPosition();
        return encoderTicks / 2048 / 5.88 * Constants.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void periodic() {
        m_odometry.update(new Rotation2d(), getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // encoder ticks per 100 ms
        // rotations per second
        // meters per second
        double leftSpeed = leftLeader.getSelectedSensorVelocity()
            * (10.0 / 2048 / 5.88) // dm -> m, et -> rot, gear ratio
            * Constants.WHEEL_CIRCUMFERENCE;
        double rightSpeed = rightLeader.getSelectedSensorVelocity()
            * (10.0 / 2048 / 5.88)
            * Constants.WHEEL_CIRCUMFERENCE;

        return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(new Rotation2d(), 0, 0, pose);
    }

    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    @Override
    public void close() throws Exception {
        leftLeader.close();
        rightLeader.close();
        leftFollower.close();
        rightFollower.close();
    }
}
