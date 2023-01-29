package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

/**
 * A class for holding constant values in a single editable spot. What goes in this file: -Robot Motorcontroller ports
 * and IDS -Robot phsyical attributes -Field attributes
 * 
 * Things that don't go in this file: -Subsystem-specific PID values -Subsystem-specific sensor thresholds
 * 
 * Constants is interface as interface fields are public, static, and final by default
 * 
 */
@SuppressWarnings("unused")
public interface Constants {
    int FRONT_SHOOTER_ID = 2;
    int BACK_SHOOTER_ID = 1;
    int SHOOTER_KICKER_ID = 7;
    int FRONT_INDEX_ID = 4;
    int BACK_INDEX_ID = 3;
    int EXTERNAL_INTAKE_ID = 10;


    public static final int LEFT_DRIVETRAIN_TALON = 1;
    public static final int RIGHT_DRIVETRAIN_TALON = 2;
    public static final int LEFT_DRIVETRAIN_VICTOR = 3;
    public static final int RIGHT_DRIVETRAIN_VICTOR = 4;

    // TODO set actual climber id
    int CLIMBER_MOTOR_ID = 0;

    double CLIMBER_VELOCITY = 0.5;

    double K_S = .61784;
    double K_V = 2.0067;
    double K_A = .28138;

    double K_P = 2.4821;
    double K_I = 0;
    double K_D = 0;
    double K_RAMSETE_B = 2;
    double K_RAMSETE_ZETA = .7;

    double TRACK_WIDTH_METERS = .60;
    double MAX_SPEED_MPS = 4.67;
    double MAX_ACCELERATION_MPSS = 3;
    double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

    double MAX_DIFF_ACCELERATION = 0;

    double TIME_TO_FULL_SPEED = .25;
    double TIME_PER_LOOP = .02;
}
