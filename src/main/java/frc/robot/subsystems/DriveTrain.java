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
    // private AHRS gyro = new AHRS();
    private final DifferentialDriveOdometry m_odometry;
    private PIDController controller;

    private BuiltInAccelerometer accelerometer;
    private double previousRightPercentOutput;

    private double previousLeftPercentOutput;

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

        previousRightPercentOutput = 0;
        previousLeftPercentOutput = 0;
    }

    /**
     * Sets the talonmFX speeds for the given speed and rotation
     * 
     * @param speed    speed from a joystick input
     * @param rotation rotation from joystick triggers
     */
    public void arcadeDrive(double speed, double rotation) {
        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        double leftMotorOutput;
        double rightMotorOutput;

        // speed is -1 to 1, rotation is also -1 to 1
        if (speed >= 0) {
            if (rotation >= 0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rotation >= 0) {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }

        leftMotorOutput = limitAcceleration(leftMotorOutput, previousLeftPercentOutput);
        rightMotorOutput = limitAcceleration(rightMotorOutput, previousRightPercentOutput);

        leftLeader.set(-leftMotorOutput);
        rightLeader.set(rightMotorOutput);

        previousLeftPercentOutput = leftMotorOutput;
        previousRightPercentOutput = rightMotorOutput;
    }

    /**
     * limits acceleration for the robot, should prevent rocking does not prevent rapid decceleration
     * 
     * @param currentTargetPercentOutput target output for motor that user inputed
     * @param previousPercentOutput      previous outputed target for motor
     * @return a target motor value
     */
    public double limitAcceleration(
        double currentTargetPercentOutput,
        double previousPercentOutput
    ) {
        // increment
        final double INCR = Constants.TIME_PER_LOOP / Constants.TIME_TO_FULL_SPEED;
        // change that the user wants
        double error = currentTargetPercentOutput - previousPercentOutput;

        // target is going towards 0
        boolean isDecel = Math.abs(currentTargetPercentOutput) < .05;

        if (isDecel)
            return currentTargetPercentOutput;

        // divide that change over a period of time
        // if the change in acceleration is too large positively, accelerate slower
        if (error > INCR)
            return previousPercentOutput + INCR;

        // the change in acceleration is too large negatively, accelerate to the
        // negative direction slower
        if (error < -INCR)
            return previousPercentOutput - INCR;

        return currentTargetPercentOutput;
    }

    /**
     * take encoder ticks displacement of left wheels and convert into meters
     * 
     * @return displacement of left wheels in meters
     */
    public double getLeftDistance() {
        double encoderTicks = leftLeader.getSelectedSensorPosition();
        double distance = encoderTicks / 2048 / 5.88 * Constants.WHEEL_CIRCUMFERENCE;
        return distance;
    }

    /**
     * Take encoder ticks displacement of right wheels and convert into meters
     * 
     * @return displacement of right wheels in meters
     */
    public double getRightDistance() {
        double encoderTicks = rightLeader.getSelectedSensorPosition();
        double distance = encoderTicks / 2048 / 5.88 * Constants.WHEEL_CIRCUMFERENCE;
        return distance;
    }

    @Override
    public void periodic() {

        // sketchy distance

        m_odometry.update(new Rotation2d(), getLeftDistance(), getRightDistance());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // encoder ticks per 100 ms
        // rotations per second
        // meters per second
        double leftSpeed = leftLeader.getSelectedSensorVelocity()
            * (10.0 / 2048 / 5.88) // dm -> m, et -> rot, gear ratio
            * Constants.WHEEL_CIRCUMFERENCE;
        // SmartDashboard.putNumber("leftSpeed", leftSpeed);
        double rightSpeed = rightLeader.getSelectedSensorVelocity()
            * (10.0 / 2048 / 5.88)
            * Constants.WHEEL_CIRCUMFERENCE;
        // SmartDashboard.putNumber("rightSpeed", rightSpeed);

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
