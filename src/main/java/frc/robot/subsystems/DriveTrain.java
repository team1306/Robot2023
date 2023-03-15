package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.utils.MotorUtils.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * Used by DriveTrain command to move robot Calculates output for each side of the drivetrain
 */
public class DriveTrain extends SubsystemBase implements AutoCloseable {

    public static final AHRS gyro = new AHRS();
    // track width of 24 inches
    private static final DifferentialDriveKinematics dk = new DifferentialDriveKinematics(
        Units.inchesToMeters(24)
    );
    private DifferentialDriveOdometry odo;

    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;

    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;


    private RelativeEncoder lEncoder, rEncoder;

    /**
     * Initializing drive train and talonFX settings
     */
    public DriveTrain() {
        leftLeader = initSparkMax(SPARK_FAR_LEFT);
        leftFollower = initSparkMax(SPARK_NEAR_LEFT);
        rightLeader = initSparkMax(SPARK_FAR_RIGHT);
        rightFollower = initSparkMax(SPARK_NEAR_RIGHT);

        // invert outputs b/c motors are mirrored
        rightLeader.setInverted(true);
        rightFollower.setInverted(true);

        lEncoder = leftLeader.getEncoder();
        rEncoder = rightLeader.getEncoder();

        // reset encoders
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

        odo = new DifferentialDriveOdometry(
            gyro.getRotation2d(),
            // 6 inch diameter wheels
            lEncoder.getPosition() * Units.inchesToMeters(6) * Math.PI,
            rEncoder.getPosition() * Units.inchesToMeters(6) * Math.PI
        );

        // have the followers follow the leaders (they will output the same values)
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
    }


    @Override
    public void periodic() {
        var angle = gyro.getRotation2d();
        odo.update(
            angle,
            lEncoder.getPosition() * Units.inchesToMeters(6) * Math.PI,
            rEncoder.getPosition() * Units.inchesToMeters(6) * Math.PI
        );

        SmartDashboard.putNumber("x", odo.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odo.getPoseMeters().getY());
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

        leftLeader.set(leftMotorOutput);
        rightLeader.set(rightMotorOutput);
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
